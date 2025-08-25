import serial
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import queue
import math
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink # MODIFIED: Use URDFLink
import pygame
import re
import time
# 添加matplotlib相关导入
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
# 添加OpenCV用于摄像头
import cv2
from PIL import Image, ImageTk

class RoboticArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("7-DOF Robotic Arm IK Controller with G-code")
        self.ser = None
        self.feedback_queue = queue.Queue()
        self.raw_serial_queue = queue.Queue() # New queue for raw serial data
        self.serial_thread = None
        self.thread_stop_event = threading.Event()
        self.GEAR_RATIO = 50.0

        # --- Keyboard and Gamepad Control State ---
        self.selected_motor_var = tk.IntVar(value=-1)
        self.arrow_key_pressed = None
        self.last_feedback_pos = [0.0] * 7

        # --- Gamepad State ---
        self.joystick = None
        self.is_polling_gamepad = False
        # 重复执行相关状态
        self.repeat_enabled = False
        self.repeat_count = 1
        self.current_repeat = 0
        self.infinite_repeat = False
        # --- G-code State ---
        self.gcode_lines = []
        self.current_line_index = 0
        self.is_gcode_running = False
        self.is_gcode_paused = False
        self.gcode_thread = None
        self.gcode_stop_event = threading.Event()
        
        # Mach3兼容的坐标系统状态
        self.coordinate_mode = 'G90'  # G90=绝对坐标, G91=相对坐标
        self.unit_mode = 'G21'        # G21=毫米, G20=英寸
        self.current_position = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}  # 当前位置
        self.feed_rate = 100.0        # 进给速度 mm/min

        # --- Pygame Initialization ---
        pygame.init()
        pygame.joystick.init()

        # --- Robot Arm Definition (IKPy) ---
        self.arm_chain = self.create_robot_arm()
        self.last_angles_rad = np.zeros(8)

        # --- GUI Setup ---
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # --- Serial Connection Frame ---
        conn_frame = ttk.LabelFrame(main_frame, text="Serial Connection")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        ttk.Label(conn_frame, text="Port:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar(value='COM15')
        ttk.Entry(conn_frame, textvariable=self.port_var, width=10).pack(side=tk.LEFT)
        ttk.Label(conn_frame, text="Baud:").pack(side=tk.LEFT, padx=5)
        self.baud_var = tk.StringVar(value='115200')
        ttk.Entry(conn_frame, textvariable=self.baud_var, width=10).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=5)
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        # --- Serial Monitor Button ---
        self.serial_monitor_button = ttk.Button(conn_frame, text="Show Serial Monitor", command=self.create_serial_monitor)
        self.serial_monitor_button.pack(side=tk.LEFT, padx=5)
        self.serial_monitor_window = None

        # --- G-code Control Frame ---
        gcode_frame = ttk.LabelFrame(main_frame, text="G-code Program Control")
        gcode_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        gcode_frame.columnconfigure(0, weight=1)  # G-code区域（收窄）
        gcode_frame.columnconfigure(1, weight=1)  # TCP轨迹区域
        gcode_frame.columnconfigure(2, weight=1)  # 摄像头区域
        
        # 创建三个主要区域
        left_frame = ttk.Frame(gcode_frame)  # G-code区域
        left_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        left_frame.columnconfigure(1, weight=1)
        
        middle_frame = ttk.Frame(gcode_frame)  # TCP轨迹区域
        middle_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        right_frame = ttk.Frame(gcode_frame)  # 摄像头区域
        right_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        
        # 文件选择区域（左侧）
        file_frame = ttk.Frame(left_frame)
        file_frame.grid(row=0, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        file_frame.columnconfigure(1, weight=1)
        
        ttk.Button(file_frame, text="Load File", command=self.load_gcode_file).grid(row=0, column=0, padx=5)
        self.gcode_file_label = ttk.Label(file_frame, text="No file loaded", foreground="gray")
        self.gcode_file_label.grid(row=0, column=1, sticky="w", padx=5)
        
        # 执行控制按钮（左侧）
        control_frame = ttk.Frame(left_frame)
        control_frame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        
        self.start_button = ttk.Button(control_frame, text="Start", command=self.start_gcode_execution, state="disabled")
        self.start_button.pack(side=tk.LEFT, padx=2)
        
        self.pause_button = ttk.Button(control_frame, text="Pause", command=self.pause_gcode_execution, state="disabled")
        self.pause_button.pack(side=tk.LEFT, padx=2)
        
        self.stop_button = ttk.Button(control_frame, text="Stop", command=self.stop_gcode_execution, state="disabled")
        self.stop_button.pack(side=tk.LEFT, padx=2)
        
        self.step_button = ttk.Button(control_frame, text="Single Step", command=self.step_gcode_execution, state="disabled")
        self.step_button.pack(side=tk.LEFT, padx=2)
        
        # 重复执行控制区域（左侧）
        repeat_frame = ttk.LabelFrame(left_frame, text="Repeat Control")
        repeat_frame.grid(row=2, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        
        # 启用重复执行复选框
        self.repeat_enabled_var = tk.BooleanVar()
        ttk.Checkbutton(repeat_frame, text="Enable Repeat", variable=self.repeat_enabled_var, 
                       command=self.toggle_repeat_mode).pack(anchor="w", padx=5, pady=2)
        
        # 重复次数设置
        repeat_count_frame = ttk.Frame(repeat_frame)
        repeat_count_frame.pack(fill="x", padx=5, pady=2)
        ttk.Label(repeat_count_frame, text="Repeat Count:").pack(side="left")
        self.repeat_count_var = tk.IntVar(value=1)
        repeat_count_spinbox = ttk.Spinbox(repeat_count_frame, from_=1, to=999, width=8, 
                                         textvariable=self.repeat_count_var)
        repeat_count_spinbox.pack(side="left", padx=5)
        
        # 无限循环选项
        self.infinite_repeat_var = tk.BooleanVar()
        ttk.Checkbutton(repeat_frame, text="Infinite Loop", variable=self.infinite_repeat_var,
                       command=self.toggle_infinite_repeat).pack(anchor="w", padx=5, pady=2)
        
        # 当前循环显示
        self.current_repeat_var = tk.StringVar(value="Current: 0/1")
        ttk.Label(repeat_frame, textvariable=self.current_repeat_var, foreground="blue").pack(anchor="w", padx=5, pady=2)
        
        # 进度显示（左侧）
        progress_frame = ttk.Frame(left_frame)
        progress_frame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        progress_frame.columnconfigure(1, weight=1)
        
        ttk.Label(progress_frame, text="Progress:").grid(row=0, column=0, sticky="w")
        self.progress_var = tk.StringVar(value="0/0 (0%)")
        ttk.Label(progress_frame, textvariable=self.progress_var).grid(row=0, column=1, sticky="w", padx=5)
        
        ttk.Label(progress_frame, text="Current Line:").grid(row=1, column=0, sticky="w")
        self.current_line_var = tk.StringVar(value="N/A")
        ttk.Label(progress_frame, textvariable=self.current_line_var, foreground="blue").grid(row=1, column=1, sticky="w", padx=5)
        
        # G-code显示区域（左侧，进一步收窄）
        text_frame = ttk.Frame(left_frame)
        text_frame.grid(row=4, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(4, weight=1)
        
        self.gcode_text = tk.Text(text_frame, height=8, width=30, wrap=tk.NONE, state=tk.DISABLED)  # 从40减小到30
        gcode_scrollbar_y = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.gcode_text.yview)
        gcode_scrollbar_x = ttk.Scrollbar(text_frame, orient=tk.HORIZONTAL, command=self.gcode_text.xview)
        self.gcode_text.config(yscrollcommand=gcode_scrollbar_y.set, xscrollcommand=gcode_scrollbar_x.set)
        
        self.gcode_text.grid(row=0, column=0, sticky="nsew")
        gcode_scrollbar_y.grid(row=0, column=1, sticky="ns")
        gcode_scrollbar_x.grid(row=1, column=0, sticky="ew")
        
        # 配置高亮标签
        self.gcode_text.tag_configure("current_line", background="yellow")
        self.gcode_text.tag_configure("executed_line", background="lightgreen")
        
        # TCP轨迹显示区域（中间）
        trajectory_frame = ttk.LabelFrame(middle_frame, text="TCP Trajectory Visualization")
        trajectory_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # 创建matplotlib图形（调整尺寸适合中间列）
        self.trajectory_fig = Figure(figsize=(4, 4), dpi=80)
        self.trajectory_ax = self.trajectory_fig.add_subplot(111, projection='3d')
        self.trajectory_canvas = FigureCanvasTkAgg(self.trajectory_fig, trajectory_frame)
        self.trajectory_canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # 初始化轨迹数据
        self.trajectory_points = []
        self.current_trajectory_point = 0
        self.trajectory_line = None
        self.current_point_marker = None
        
        # 初始化实时轨迹相关变量
        self.realtime_trajectory_points = []
        self.realtime_trajectory_line = None
        self.current_tcp_marker = None
        
        # 初始化实时轨迹数据
        self.realtime_trajectory_points = []
        
        # 设置3D图形的初始状态
        self.setup_trajectory_plot()
        
        # 摄像头显示区域（右侧）
        camera_frame = ttk.LabelFrame(right_frame, text="Camera View")
        camera_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # 摄像头控制区域
        camera_control_frame = ttk.Frame(camera_frame)
        camera_control_frame.pack(fill="x", padx=5, pady=5)
        
        self.camera_button = ttk.Button(camera_control_frame, text="Start Camera", command=self.toggle_camera)
        self.camera_button.pack(side=tk.LEFT, padx=5)
        
        self.camera_status_label = ttk.Label(camera_control_frame, text="Status: Disconnected", foreground="red")
        self.camera_status_label.pack(side=tk.LEFT, padx=5)
        
        # 摄像头分辨率选择
        ttk.Label(camera_control_frame, text="Resolution:").pack(side=tk.LEFT, padx=5)
        self.camera_resolution_var = tk.StringVar(value="640x480")
        resolution_combo = ttk.Combobox(camera_control_frame, textvariable=self.camera_resolution_var, 
                                      values=["320x240", "640x480", "800x600", "1024x768"], 
                                      width=10, state="readonly")
        resolution_combo.pack(side=tk.LEFT, padx=5)
        resolution_combo.bind("<<ComboboxSelected>>", self.on_resolution_change)
        
        # 摄像头显示区域
        self.camera_display_frame = ttk.Frame(camera_frame)
        self.camera_display_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # 摄像头画面标签
        self.camera_label = ttk.Label(self.camera_display_frame, text="Camera Not Connected", 
                                    background="black", foreground="white", font=("Arial", 12))
        self.camera_label.pack(fill="both", expand=True)
        


        # --- IK Control Frame ---
        ik_frame = ttk.LabelFrame(main_frame, text="Inverse Kinematics (TCP Target)")
        ik_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.target_vars = {}
        for i, axis in enumerate(["X", "Y", "Z"]):
            ttk.Label(ik_frame, text=f"{axis} (mm):").grid(row=0, column=i*2, padx=5, pady=2)
            self.target_vars[axis] = tk.DoubleVar(value=150 if axis == 'X' else 0)
            ttk.Entry(ik_frame, textvariable=self.target_vars[axis], width=8).grid(row=0, column=i*2+1, padx=5, pady=2)
        self.ik_button = ttk.Button(ik_frame, text="Calculate & Move", command=self.calculate_and_move)
        self.ik_button.grid(row=0, column=6, padx=10, pady=2)

        # --- Gamepad Control Frame ---
        gamepad_frame = ttk.LabelFrame(main_frame, text="Gamepad Control")
        gamepad_frame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.gamepad_connect_button = ttk.Button(gamepad_frame, text="Connect Gamepad", command=self.toggle_gamepad_connection)
        self.gamepad_connect_button.pack(side=tk.LEFT, padx=5)
        self.gamepad_status_label = ttk.Label(gamepad_frame, text="Status: Not Connected", foreground="red")
        self.gamepad_status_label.pack(side=tk.LEFT, padx=5)

        # --- Joint Control & Feedback Frame ---
        joint_main_frame = ttk.LabelFrame(main_frame, text="Joint Control & Feedback")
        joint_main_frame.grid(row=4, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        joint_main_frame.columnconfigure(0, weight=1)  # 只有关节控制区域
        
        # 关节控制区域
        joint_frame = ttk.Frame(joint_main_frame)
        joint_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        joint_frame.columnconfigure(2, weight=1)  # 滑块列可扩展
        
        # 关节控制表头
        headers = ["Joint", "Select", "Manual Control", "Target", "Feedback", "Voltage"]
        header_widths = [6, 6, 20, 8, 8, 8]
        for i, (header, width) in enumerate(zip(headers, header_widths)):
            label = ttk.Label(joint_frame, text=header, font="-weight bold", width=width)
            label.grid(row=0, column=i, padx=2, pady=2)

        self.joint_vars = []
        self.joint_labels = {}
        for i in range(7):
            # 关节标签
            ttk.Label(joint_frame, text=f"M{i}", width=6).grid(row=i+1, column=0, padx=2)
            
            # 选择按钮
            ttk.Radiobutton(joint_frame, variable=self.selected_motor_var, value=i, width=6).grid(row=i+1, column=1, padx=2)
            
            # 滑块控制
            var = tk.DoubleVar(value=0)
            slider = ttk.Scale(joint_frame, from_=-180, to=180, orient=tk.HORIZONTAL, variable=var, 
                             command=lambda v, j=i: self.update_joint_label(j, v), length=150)
            slider.grid(row=i+1, column=2, sticky="w", padx=2)
            slider.bind("<ButtonRelease-1>", self.send_manual_angles)
            self.joint_vars.append(var)
            
            # 反馈标签
            self.joint_labels[i] = {
                "target": ttk.Label(joint_frame, text="0.00", width=8),
                "feedback_pos": ttk.Label(joint_frame, text="N/A", width=8, foreground="blue"),
                "feedback_volt": ttk.Label(joint_frame, text="N/A", width=8, foreground="green")
            }
            self.joint_labels[i]["target"].grid(row=i+1, column=3, padx=2)
            self.joint_labels[i]["feedback_pos"].grid(row=i+1, column=4, padx=2)
            self.joint_labels[i]["feedback_volt"].grid(row=i+1, column=5, padx=2)
        
        # 初始化摄像头相关变量
        self.camera = None
        self.camera_running = False
        self.camera_thread = None

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.process_queue()
        self.root.bind("<KeyPress>", self.handle_key_press)
        self.root.bind("<KeyRelease>", self.handle_key_release)

    # G-code相关方法
    def load_gcode_file(self):
        """加载G-code文件"""
        file_path = filedialog.askopenfilename(
            title="Select G-code File",
            filetypes=[
                ("G-code files", "*.gcode *.nc *.tap *.txt"),
                ("TAP files", "*.tap"),
                ("NC files", "*.nc"),
                ("All files", "*.*")
            ]
        )
        
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as file:
                    content = file.read()
                
                # 分割成行并过滤空行
                self.gcode_lines = [line.strip() for line in content.split('\n') if line.strip()]
                
                # 显示在文本框中
                self.gcode_text.config(state=tk.NORMAL)
                self.gcode_text.delete(1.0, tk.END)
                self.gcode_text.insert(1.0, content)
                self.gcode_text.config(state=tk.DISABLED)
                
                # 更新文件标签
                filename = file_path.split('/')[-1]
                self.gcode_file_label.config(text=f"Loaded: {filename}", foreground="blue")
                
                # 启用执行按钮
                self.start_button.config(state="normal")
                self.step_button.config(state="normal")
                
                # 重置执行状态
                self.current_line_index = 0
                self.update_progress_display()
                
                # 解析并显示轨迹
                self.parse_gcode_trajectory()
                
                # messagebox.showinfo("Success", f"G-code file loaded successfully!\nTotal lines: {len(self.gcode_lines)}")  # 移除此行
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load G-code file: {e}")
    
    def update_progress_display(self):
        """更新进度显示"""
        total_lines = len(self.gcode_lines)
        if total_lines > 0:
            progress_percent = (self.current_line_index / total_lines) * 100
            self.progress_var.set(f"{self.current_line_index}/{total_lines} ({progress_percent:.1f}%)")
            
            if self.current_line_index < total_lines:
                self.current_line_var.set(self.gcode_lines[self.current_line_index])
            else:
                self.current_line_var.set("Program Complete")
        else:
            self.progress_var.set("0/0 (0%)")
            self.current_line_var.set("N/A")
    
    def start_gcode_execution(self):
        """开始执行G-code程序"""
        if not self.gcode_lines:
            messagebox.showwarning("No Program", "Please load a G-code file first.")
            return
        
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Serial Error", "Please connect to serial port first.")
            return
        
        self.is_gcode_running = True
        self.is_gcode_paused = False
        self.gcode_stop_event.clear()
        
        # 更新按钮状态
        self.start_button.config(state="disabled")
        self.pause_button.config(state="normal")
        self.stop_button.config(state="normal")
        
        # 启动执行线程
        self.gcode_thread = threading.Thread(target=self.execute_gcode_program, daemon=True)
        self.gcode_thread.start()
    
    def pause_gcode_execution(self):
        """暂停/恢复G-code执行"""
        if self.is_gcode_paused:
            self.is_gcode_paused = False
            self.pause_button.config(text="Pause")
        else:
            self.is_gcode_paused = True
            self.pause_button.config(text="Resume")
    
    def stop_gcode_execution(self):
        """停止G-code执行"""
        self.is_gcode_running = False
        self.is_gcode_paused = False
        self.gcode_stop_event.set()
        
        # 更新按钮状态
        self.start_button.config(state="normal")
        self.pause_button.config(state="disabled", text="Pause")
        self.stop_button.config(state="disabled")
    
    def step_gcode_execution(self):
        """单步执行G-code"""
        if not self.gcode_lines or self.current_line_index >= len(self.gcode_lines):
            return
        
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Serial Error", "Please connect to serial port first.")
            return
        
        # 执行当前行
        self.execute_gcode_line(self.gcode_lines[self.current_line_index])
        self.current_line_index += 1
        self.update_progress_display()
        self.highlight_current_line()

    def create_robot_arm(self):
        # MODIFIED: This function is updated for ikpy versions > 3.0
        # The parameters for defining links have been renamed.
        link_length = 0.15  # 修改为米单位：150mm = 0.15m
        joint_limits = (-math.pi, math.pi)
        links = [
            OriginLink(),
            URDFLink(name="J1", origin_translation=[0, 0, 0], origin_orientation=[0, 0, 0], rotation=[0, 0, 1], bounds=joint_limits),
            URDFLink(name="J2", origin_translation=[0, 0, link_length], origin_orientation=[0, 0, 0], rotation=[0, 1, 0], bounds=joint_limits),
            URDFLink(name="J3", origin_translation=[0, 0, link_length], origin_orientation=[0, 0, 0], rotation=[0, 0, 1], bounds=joint_limits),
            URDFLink(name="J4", origin_translation=[0, 0, link_length], origin_orientation=[0, 0, 0], rotation=[0, 1, 0], bounds=joint_limits),
            URDFLink(name="J5", origin_translation=[0, 0, link_length], origin_orientation=[0, 0, 0], rotation=[0, 0, 1], bounds=joint_limits),
            URDFLink(name="J6", origin_translation=[0, 0, link_length], origin_orientation=[0, 0, 0], rotation=[0, 1, 0], bounds=joint_limits),
            URDFLink(name="J7", origin_translation=[0, 0, link_length], origin_orientation=[0, 0, 0], rotation=[0, 0, 1], bounds=joint_limits)
        ]
        return Chain(links, active_links_mask=[False, True, True, True, True, True, True, True])

    def calculate_and_move(self):
        target_pos = [self.target_vars["X"].get(), self.target_vars["Y"].get(), self.target_vars["Z"].get()]
        try:
            ik_angles_rad = self.arm_chain.inverse_kinematics(
                target_position=target_pos,
                initial_position=self.last_angles_rad
            )
            self.last_angles_rad = ik_angles_rad
            angles_deg = [math.degrees(angle) for angle in ik_angles_rad[1:]]
            self.update_all_joint_labels(angles_deg)
            self.send_angles(angles_deg)
        except Exception as e:
            messagebox.showerror("IK Error", f"Could not find a solution.\n{e}")

    def update_all_joint_labels(self, angles_deg):
        for i, angle in enumerate(angles_deg):
            self.joint_vars[i].set(angle)
            self.joint_labels[i]["target"].config(text=f"{angle:.2f}")

    def update_joint_label(self, joint_index, value):
        self.joint_labels[joint_index]["target"].config(text=f"{float(value):.2f}")

    def send_manual_angles(self, event=None):
        angles = [var.get() for var in self.joint_vars]
        self.send_angles(angles)

    def send_angles(self, angles_deg):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Serial Error", "Not connected.")
            return
        # Apply gear ratio before sending
        angles_with_ratio = [angle * self.GEAR_RATIO for angle in angles_deg]
        command = ",".join([f"{angle:.2f}" for angle in angles_with_ratio]) + "\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except (ValueError, serial.SerialException) as e:
            messagebox.showerror("Serial Error", f"Error writing to serial port: {e}")
            self.toggle_connection()

    def process_queue(self):
        try:
            while True:
                line = self.feedback_queue.get_nowait()
                if line == "SERIAL_ERROR":
                    self.toggle_connection()
                    messagebox.showerror("Serial Error", "Lost connection to serial port.")
                    break
                elif line.startswith("ID:"):
                    # 新的数据格式: "ID:1,POS:123.45,VOL:12.34"
                    parts = line.split(',')
                    motor_id = None
                    position = None
                    voltage = None
                    
                    for part in parts:
                        if part.startswith('ID:'):
                            motor_id = int(part.split(':')[1])
                        elif part.startswith('POS:'):
                            position = float(part.split(':')[1])
                        elif part.startswith('VOL:'):
                            voltage = float(part.split(':')[1])
                    
                    if motor_id is not None and position is not None:
                        # 更新反馈位置（索引从0开始）
                        if 0 <= motor_id - 1 < len(self.last_feedback_pos):
                            self.last_feedback_pos[motor_id - 1] = position
                        
                        # 更新关节标签显示
                        if 0 <= motor_id - 1 < 7:
                            self.joint_labels[motor_id - 1]["feedback_pos"].config(text=f"{position:.2f}")
                            if voltage is not None:
                                self.joint_labels[motor_id - 1]["feedback_volt"].config(text=f"{voltage:.2f}")
                        
                        # 移除条件检查，只要有反馈数据就更新实时TCP轨迹
                        self.update_realtime_tcp_trajectory()
                elif line.startswith("FB:"):
                    # 保持对旧格式的兼容性
                    parts = line[3:].split(',')
                    if len(parts) == 3:
                        try:
                            motor_id, pos, volt = int(parts[0]), float(parts[1]), float(parts[2])
                            # Apply gear ratio to feedback position for display
                            output_pos = pos / self.GEAR_RATIO
                            if 0 <= motor_id < 7:
                                self.last_feedback_pos[motor_id] = output_pos # Store latest feedback
                                self.joint_labels[motor_id]["feedback_pos"].config(text=f"{output_pos:.2f}")
                                self.joint_labels[motor_id]["feedback_volt"].config(text=f"{volt:.2f}")
                                
                                # 移除条件检查，只要有反馈数据就更新实时TCP轨迹
                                self.update_realtime_tcp_trajectory()
                        except (ValueError, IndexError):
                            pass
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_queue)

    def create_serial_monitor(self):
        if self.serial_monitor_window and self.serial_monitor_window.winfo_exists():
            self.serial_monitor_window.lift()
            return

        self.serial_monitor_window = tk.Toplevel(self.root)
        self.serial_monitor_window.title("Serial Monitor")
        self.serial_monitor_window.geometry("600x400")

        serial_text_area = tk.Text(self.serial_monitor_window, wrap=tk.WORD, state=tk.DISABLED)
        serial_text_area.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)

        def update_serial_monitor():
            try:
                while True:
                    line = self.raw_serial_queue.get_nowait()
                    serial_text_area.config(state=tk.NORMAL)
                    serial_text_area.insert(tk.END, line + '\n')
                    serial_text_area.see(tk.END)
                    serial_text_area.config(state=tk.DISABLED)
            except queue.Empty:
                pass
            finally:
                if self.serial_monitor_window.winfo_exists():
                    self.serial_monitor_window.after(100, update_serial_monitor)
        
        update_serial_monitor()

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.thread_stop_event.set()
            if self.serial_thread: self.serial_thread.join()
            self.ser.close()
            self.status_label.config(text="Status: Disconnected", foreground="red")
            self.connect_button.config(text="Connect")
        else:
            try:
                self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=1)
                self.status_label.config(text=f"Status: Connected", foreground="green")
                self.connect_button.config(text="Disconnect")
                self.thread_stop_event.clear()
                self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.serial_thread.start()
            except (serial.SerialException, ValueError) as e:
                messagebox.showerror("Connection Error", f"Failed to connect: {e}")

    def read_serial_data(self):
        while not self.thread_stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.raw_serial_queue.put(line) # Put raw line in new queue
                    self.feedback_queue.put(line)
            except (serial.SerialException, UnicodeDecodeError):
                if not self.thread_stop_event.is_set():
                    self.raw_serial_queue.put("SERIAL_ERROR") # Also notify monitor of error
                    self.feedback_queue.put("SERIAL_ERROR")
                break

    def toggle_gamepad_connection(self):
        if self.is_polling_gamepad:
            self.is_polling_gamepad = False
            pygame.joystick.quit()
            self.gamepad_status_label.config(text="Status: Not Connected", foreground="red")
            self.gamepad_connect_button.config(text="Connect Gamepad")
            # self.last_gamepad_event_label.config(text="Last Event: N/A") # This line caused the error and is now removed
        else:
            pygame.joystick.init() # Re-init in case it was quit
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                messagebox.showerror("Gamepad Error", "No gamepad detected. Please connect a gamepad and try again.")
                return
            
            # Use the first available joystick
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            
            self.gamepad_status_label.config(text=f"Status: Connected to {self.joystick.get_name()}", foreground="green")
            self.gamepad_connect_button.config(text="Disconnect Gamepad")
            
            self.is_polling_gamepad = True
            self.poll_gamepad_events()

    def poll_gamepad_events(self):
        if not self.is_polling_gamepad or not self.joystick:
            return

        # It's crucial to process the event queue, otherwise the window may become unresponsive.
        pygame.event.pump()

        # --- Velocity Control Logic ---
        # Instead of waiting for events, we directly query the joystick state in each cycle.
        
        gamepad_control_active = False
        speed_factor = 2.0  # Degrees to move per poll cycle at max stick deflection

        # Motor 0 (Left Stick X-axis: 0)
        left_stick_x = self.joystick.get_axis(0)
        if abs(left_stick_x) > 0.15:  # Apply deadzone
            current_angle = self.joint_vars[0].get()
            change = left_stick_x * speed_factor
            new_angle = max(-180.0, min(180.0, current_angle + change))
            self.joint_vars[0].set(new_angle)
            self.update_joint_label(0, new_angle)
            gamepad_control_active = True

        # Motor 1 (Right Stick X-axis: 2)
        right_stick_x = self.joystick.get_axis(2)
        if abs(right_stick_x) > 0.15:  # Apply deadzone
            current_angle = self.joint_vars[1].get()
            change = right_stick_x * speed_factor
            new_angle = max(-180.0, min(180.0, current_angle + change))
            self.joint_vars[1].set(new_angle)
            self.update_joint_label(1, new_angle)
            gamepad_control_active = True

        # If any motor was moved by the gamepad, send the updated angles
        if gamepad_control_active:
            self.send_manual_angles()

        # Schedule the next poll
        self.root.after(20, self.poll_gamepad_events)

    def handle_key_press(self, event):
        # Motor selection with number keys 1-7
        if event.keysym.isdigit() and 1 <= int(event.keysym) <= 7:
            self.selected_motor_var.set(int(event.keysym) - 1)
            return

        motor_id = self.selected_motor_var.get()
        if motor_id == -1 or self.arrow_key_pressed:
            return # Do nothing if no motor is selected or a key is already held down

        if event.keysym in ["Left", "Right"]:
            self.arrow_key_pressed = event.keysym
            target_angle = -180 if event.keysym == "Left" else 180
            
            self.joint_vars[motor_id].set(target_angle)
            self.update_joint_label(motor_id, target_angle)
            self.send_manual_angles()

    def handle_key_release(self, event):
        if event.keysym != self.arrow_key_pressed:
            return # Only act on the release of the key that was pressed

        motor_id = self.selected_motor_var.get()
        if motor_id == -1:
            self.arrow_key_pressed = None
            return

        last_pos = self.last_feedback_pos[motor_id]
        adjustment = 6 if self.arrow_key_pressed == 'Right' else -6
        new_target = max(-180, min(180, last_pos + adjustment)) # Clamp value

        self.joint_vars[motor_id].set(new_target)
        self.update_joint_label(motor_id, new_target)
        self.send_manual_angles()
        
        self.arrow_key_pressed = None # Reset after action

    def toggle_camera(self):
        """切换摄像头开关"""
        if not self.camera_running:
            self.start_camera()
        else:
            self.stop_camera()
    
    def start_camera(self):
        """启动摄像头"""
        try:
            print("正在连接摄像头...")
            
            # 使用与测试脚本相同的方式
            self.camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            
            if not self.camera.isOpened():
                messagebox.showerror("Camera Error", "无法打开摄像头")
                return
            
            # 测试读取一帧
            ret, frame = self.camera.read()
            if not ret or frame is None:
                messagebox.showerror("Camera Error", "摄像头无法读取画面")
                self.camera.release()
                return
            
            print(f"摄像头连接成功，画面尺寸: {frame.shape[1]}x{frame.shape[0]}")
            
            # 设置分辨率
            resolution = self.camera_resolution_var.get().split('x')
            width, height = int(resolution[0]), int(resolution[1])
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            self.camera_running = True
            self.camera_button.config(text="Stop Camera")
            self.camera_status_label.config(text="Status: Connected", foreground="green")
            
            # 启动摄像头线程
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            
        except Exception as e:
            print(f"摄像头启动异常: {e}")
            messagebox.showerror("Camera Error", f"启动摄像头失败: {e}")
            if hasattr(self, 'camera') and self.camera:
                self.camera.release()
                self.camera = None
    
    def stop_camera(self):
        """停止摄像头"""
        self.camera_running = False
        
        if self.camera:
            self.camera.release()
            self.camera = None
        
        self.camera_button.config(text="Start Camera")
        self.camera_status_label.config(text="Status: Disconnected", foreground="red")
        self.camera_label.config(image="", text="Camera Not Connected")

    
    def on_resolution_change(self, event=None):
        """分辨率改变事件"""
        if self.camera_running and self.camera:
            # 重新设置摄像头分辨率
            resolution = self.camera_resolution_var.get().split('x')
            width, height = int(resolution[0]), int(resolution[1])
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    def camera_loop(self):
        """摄像头循环线程"""
        consecutive_failures = 0
        max_failures = 10
        
        while self.camera_running and self.camera:
            try:
                ret, frame = self.camera.read()
                if ret and frame is not None:
                    consecutive_failures = 0  # 重置失败计数
                    
                    # 转换颜色格式（OpenCV使用BGR，PIL使用RGB）
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    # 调整图像大小以适应显示区域
                    display_width = 720   # 进一步增加到720，最大化利用显示区域
                    display_height = 540  # 增加到540，保持4:3比例
                    frame_resized = cv2.resize(frame_rgb, (display_width, display_height))
                    
                    # 转换为PIL图像
                    pil_image = Image.fromarray(frame_resized)
                    
                    # 转换为Tkinter可显示的格式
                    tk_image = ImageTk.PhotoImage(pil_image)
                    
                    # 更新GUI（必须在主线程中执行）
                    self.root.after(0, self.update_camera_display, tk_image)
                    
                    # 控制帧率（约30fps）
                    time.sleep(0.033)
                else:
                    consecutive_failures += 1
                    print(f"摄像头读取失败 {consecutive_failures}/{max_failures}")
                    
                    if consecutive_failures >= max_failures:
                        print("摄像头连续读取失败，停止摄像头")
                        self.root.after(0, lambda: messagebox.showwarning(
                            "Camera Warning", 
                            "摄像头连续读取失败，已自动停止。\n请检查摄像头连接后重新启动。"))
                        break
                    
                    time.sleep(0.1)  # 失败时稍微等待
                    
            except Exception as e:
                consecutive_failures += 1
                print(f"Camera loop error: {e}")
                
                if consecutive_failures >= max_failures:
                    print("摄像头异常过多，停止摄像头")
                    self.root.after(0, lambda: messagebox.showerror(
                        "Camera Error", 
                        f"摄像头出现异常: {e}\n已自动停止摄像头。"))
                    break
                
                time.sleep(0.1)
        
        # 清理
        self.camera_running = False
        if self.camera:
            self.camera.release()
            self.camera = None
        
        # 更新GUI状态
        self.root.after(0, lambda: [
            self.camera_button.config(text="Start Camera"),
            self.camera_status_label.config(text="Status: Disconnected", foreground="red"),
            self.camera_label.config(image="", text="Camera Disconnected")
        ])
    
    def update_camera_display(self, tk_image):
        """更新摄像头显示（在主线程中调用）"""
        if self.camera_running:
            self.camera_label.config(image=tk_image, text="")
            self.camera_label.image = tk_image  # 保持引用防止垃圾回收

    def on_closing(self):
        """程序关闭时的清理工作"""
        # 停止G-code执行
        if hasattr(self, 'is_gcode_running') and self.is_gcode_running:
            self.stop_gcode_execution()
        
        # 停止摄像头
        if hasattr(self, 'camera_running') and self.camera_running:
            self.stop_camera()
        
        # 关闭串口
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
        
        # 停止游戏手柄
        if hasattr(self, 'is_polling_gamepad') and self.is_polling_gamepad:
            self.is_polling_gamepad = False
            pygame.quit()
        
        # 关闭串口监视器窗口
        if hasattr(self, 'serial_monitor_window') and self.serial_monitor_window:
            self.serial_monitor_window.destroy()
        
        # 停止线程
        if hasattr(self, 'thread_stop_event'):
            self.thread_stop_event.set()
        
        self.root.destroy()
    
    def execute_gcode_program(self):
        """在单独线程中执行G-code程序（支持重复执行）"""
        try:
            # 从GUI获取重复执行设置
            repeat_enabled = self.repeat_enabled_var.get()
            infinite_repeat = self.infinite_repeat_var.get()
            repeat_count = self.repeat_count_var.get()
            
            # 调试信息
            print(f"重复执行设置: enabled={repeat_enabled}, infinite={infinite_repeat}, count={repeat_count}")
            
            # 初始化重复执行状态
            if repeat_enabled:
                if infinite_repeat:
                    total_repeats = float('inf')
                else:
                    total_repeats = repeat_count
            else:
                total_repeats = 1
            
            print(f"总重复次数: {total_repeats}")
            self.current_repeat = 0
            
            # 重复执行循环
            while (self.current_repeat < total_repeats) and not self.gcode_stop_event.is_set():
                self.current_repeat += 1
                print(f"开始第 {self.current_repeat} 次执行")
                self.root.after(0, self.update_repeat_display)
                
                # 重置到程序开始
                self.current_line_index = 0
                
                # 执行一遍完整程序
                while self.current_line_index < len(self.gcode_lines) and not self.gcode_stop_event.is_set():
                    if self.is_gcode_paused:
                        time.sleep(0.1)
                        continue
                    
                    line = self.gcode_lines[self.current_line_index]
                    
                    # 高亮当前行
                    self.root.after(0, self.highlight_current_line)
                    
                    # 执行G-code行
                    success = self.execute_gcode_line(line)
                    if not success:
                        print(f"Execution failed at line {self.current_line_index + 1}: {line}")
                        # 可以选择是否继续执行
                        # break  # 取消注释此行以在错误时停止执行
                    
                    self.current_line_index += 1
                    
                    # 更新进度显示
                    self.root.after(0, self.update_progress_display)
                    
                    # 添加小延迟以便观察执行过程
                    time.sleep(0.1)
                # 如果不是无限循环且不是最后一次，在重复之间添加延迟
                if not infinite_repeat and self.current_repeat < total_repeats and not self.gcode_stop_event.is_set():
                    print(f"等待下一次重复执行...")
                    time.sleep(0.5)  # 重复间隔
            
            print(f"所有重复执行完成，总共执行了 {self.current_repeat} 次")
            
        except Exception as e:
            print(f"G-code execution error: {e}")
            self.root.after(0, lambda: messagebox.showerror("Execution Error", f"G-code execution failed: {e}"))
        finally:
            # 执行完成后的清理
            self.is_gcode_running = False
            self.is_gcode_paused = False
            
            # 更新按钮状态
            self.root.after(0, lambda: (
                self.start_button.config(state="normal"),
                self.pause_button.config(state="disabled", text="Pause"),
                self.stop_button.config(state="disabled")
            ))
            
            # 保持最终的重复计数显示，不要重置为0
            self.root.after(0, self.update_repeat_display)
            
            # 显示完成消息
            self.root.after(0, lambda: messagebox.showinfo("G-code Execution", f"G-code program execution completed! Executed {self.current_repeat} time(s)."))

    def execute_gcode_line(self, line):
        """执行单行G-code指令"""
        try:
            if not line or line.startswith(';'):
                return True
            
            # 移除注释
            if ';' in line:
                line = line[:line.index(';')]
            
            line = line.strip().upper()
            if not line:
                return True
            
            # 解析G-code指令
            commands = self.parse_gcode_line(line)
            
            for command in commands:
                self.execute_gcode_command(command)
            
            return True
            
        except Exception as e:
            print(f"Error executing G-code line '{line}': {e}")
            # 可以选择是否继续执行或停止
            return False  # 返回False表示执行失败
    
    def parse_gcode_line(self, line):
        """解析G-code行，返回命令列表"""
        commands = []
        
        # 使用正则表达式提取所有命令
        pattern = r'([GMXYZFIJKRS])([+-]?\d*\.?\d*)'
        matches = re.findall(pattern, line)
        
        current_command = {}
        
        for letter, value in matches:
            if letter in ['G', 'M']:
                # 如果遇到新的G或M命令，保存之前的命令
                if current_command:
                    commands.append(current_command)
                current_command = {letter: float(value) if value else 0}
            else:
                # 添加参数到当前命令
                if value:
                    current_command[letter] = float(value)
        
        # 添加最后一个命令
        if current_command:
            commands.append(current_command)
        
        return commands
    
    def execute_gcode_command(self, command):
        """执行单个G-code命令"""
        if 'G' in command:
            self.execute_g_command(command)
        elif 'M' in command:
            self.execute_m_command(command)
    
    def execute_g_command(self, command):
        """执行G指令"""
        g_code = int(command['G'])
        
        if g_code == 0:  # G00 - 快速定位
            self.execute_linear_move(command, rapid=True)
        elif g_code == 1:  # G01 - 直线插补
            self.execute_linear_move(command, rapid=False)
        elif g_code == 2:  # G02 - 顺时针圆弧插补
            self.execute_arc_move(command, clockwise=True)
        elif g_code == 3:  # G03 - 逆时针圆弧插补
            self.execute_arc_move(command, clockwise=False)
        elif g_code == 20:  # G20 - 英寸单位
            self.unit_mode = 'G20'
        elif g_code == 21:  # G21 - 毫米单位
            self.unit_mode = 'G21'
        elif g_code == 90:  # G90 - 绝对坐标
            self.coordinate_mode = 'G90'
        elif g_code == 91:  # G91 - 相对坐标
            self.coordinate_mode = 'G91'
        elif g_code == 28:  # G28 - 回零
            self.execute_home_command()
        else:
            print(f"Unsupported G-code: G{g_code:02d}")
    
    def execute_m_command(self, command):
        """执行M指令"""
        m_code = int(command['M'])
        
        if m_code == 0:  # M00 - 程序暂停
            self.is_gcode_paused = True
            self.pause_button.config(text="Resume")
        elif m_code == 1:  # M01 - 可选暂停
            pass  # 可以根据需要实现
        elif m_code == 2 or m_code == 30:  # M02/M30 - 程序结束
            # 修改：不调用stop_gcode_execution，而是设置一个标志来结束当前循环
            print(f"遇到程序结束指令 M{m_code:02d}，当前循环结束")
            # 通过设置current_line_index来结束当前循环
            self.current_line_index = len(self.gcode_lines)
        else:
            print(f"Unsupported M-code: M{m_code:02d}")
    
    def execute_linear_move(self, command, rapid=False):
        """执行直线移动"""
        target_pos = self.current_position.copy()
        
        # 解析目标坐标
        for axis in ['X', 'Y', 'Z']:
            if axis in command:
                value = command[axis]
                
                # 单位转换
                if self.unit_mode == 'G20':  # 英寸转毫米
                    value *= 25.4
                
                # 坐标模式处理
                if self.coordinate_mode == 'G90':  # 绝对坐标
                    target_pos[axis] = value
                else:  # G91 相对坐标
                    target_pos[axis] += value
        
        # 更新进给速度
        if 'F' in command:
            self.feed_rate = command['F']
        
        # 执行移动
        self.move_to_position(target_pos, rapid)
        
        # 更新当前位置
        self.current_position = target_pos
    
    def execute_arc_move(self, command, clockwise=True):
        """执行圆弧移动（简化实现）"""
        # 这里实现简化的圆弧插补
        # 实际应用中需要更复杂的圆弧计算
        target_pos = self.current_position.copy()
        
        # 解析终点坐标
        for axis in ['X', 'Y', 'Z']:
            if axis in command:
                value = command[axis]
                if self.unit_mode == 'G20':
                    value *= 25.4
                
                if self.coordinate_mode == 'G90':
                    target_pos[axis] = value
                else:
                    target_pos[axis] += value
        
        # 简化处理：直接移动到终点（实际应该插补圆弧）
        self.move_to_position(target_pos, False)
        self.current_position = target_pos
    
    def execute_home_command(self):
        """执行回零命令"""
        home_position = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.move_to_position(home_position, rapid=True)
        self.current_position = home_position
    
    def move_to_position(self, target_pos, rapid=False):
        """移动到指定位置"""
        try:
            # 使用逆运动学计算关节角度
            target_position = [target_pos['X'], target_pos['Y'], target_pos['Z']]
            
            ik_angles_rad = self.arm_chain.inverse_kinematics(
                target_position=target_position,
                initial_position=self.last_angles_rad
            )
            
            self.last_angles_rad = ik_angles_rad
            angles_deg = [math.degrees(angle) for angle in ik_angles_rad[1:]]
            
            # 更新GUI显示
            self.root.after(0, lambda: self.update_all_joint_labels(angles_deg))
            
            # 发送到机械臂
            self.send_angles(angles_deg)
            
            # 根据是否为快速移动添加适当延时
            if not rapid:
                # 根据进给速度计算延时
                delay = 60.0 / self.feed_rate  # 简化的时间计算
                time.sleep(min(delay, 1.0))  # 最大延时1秒
            
            return True
            
        except Exception as e:
            print(f"IK calculation failed for position {target_pos}: {e}")
            # 显示错误信息给用户
            self.root.after(0, lambda: messagebox.showerror("Motion Error", f"Failed to move to position {target_pos}: {e}"))
            return False
    
    def highlight_current_line(self):
        """高亮显示当前执行的G-code行"""
        if not hasattr(self, 'gcode_lines') or self.current_line_index >= len(self.gcode_lines):
            return
        
        # 清除之前的高亮
        self.gcode_text.tag_remove("current_line", "1.0", tk.END)
        
        # 高亮当前行
        line_start = f"{self.current_line_index + 1}.0"
        line_end = f"{self.current_line_index + 1}.end"
        self.gcode_text.tag_add("current_line", line_start, line_end)
        
        # 滚动到当前行
        self.gcode_text.see(line_start)
        
        # 更新轨迹显示中的当前位置
        self.update_current_trajectory_position(self.current_line_index)

    def on_gcode_execution_finished(self):
        """G-code执行完成后的处理（现在主要用于单步执行）"""
        # 执行完成，重置状态
        self.is_gcode_running = False
        self.is_gcode_paused = False
        
        # 更新按钮状态
        self.start_button.config(state="normal")
        self.pause_button.config(state="disabled", text="Pause")
        self.stop_button.config(state="disabled")

    def toggle_repeat_mode(self):
        """切换重复执行模式"""
        self.update_repeat_display()

    def toggle_infinite_repeat(self):
        """切换无限重复模式"""
        # 注释掉有问题的代码，因为repeat_count_spinbox不是实例变量
        # if self.infinite_repeat_var.get():
        #     # 禁用重复次数输入
        #     self.repeat_count_spinbox.config(state="disabled")
        # else:
        #     # 启用重复次数输入
        #     self.repeat_count_spinbox.config(state="normal")
        self.update_repeat_display()

    def update_repeat_display(self):
        """更新重复执行显示"""
        if self.infinite_repeat_var.get():
            self.current_repeat_var.set(f"Current: {self.current_repeat}/∞")
        else:
            total = self.repeat_count_var.get() if self.repeat_enabled_var.get() else 1
            self.current_repeat_var.set(f"Current: {self.current_repeat}/{total}")

    def setup_trajectory_plot(self):
        """设置3D轨迹图的初始状态"""
        self.trajectory_ax.clear()
        self.trajectory_ax.set_xlabel('X (mm)')
        self.trajectory_ax.set_ylabel('Y (mm)')
        self.trajectory_ax.set_zlabel('Z (mm)')
        self.trajectory_ax.set_title('TCP Trajectory')
        
        # 设置坐标轴范围
        self.trajectory_ax.set_xlim([-200, 200])
        self.trajectory_ax.set_ylim([-200, 200])
        self.trajectory_ax.set_zlim([0, 400])
        
        # 添加网格
        self.trajectory_ax.grid(True)
        
        self.trajectory_canvas.draw()
    
    def parse_gcode_trajectory(self):
        """预解析G-code文件，提取轨迹点"""
        if not hasattr(self, 'gcode_lines') or not self.gcode_lines:
            return
        
        self.trajectory_points = []
        current_pos = {'X': 0, 'Y': 0, 'Z': 0}  # 起始位置
        
        for line in self.gcode_lines:
            line = line.strip()
            if not line or line.startswith(';') or line.startswith('('):
                continue
            
            # 解析G-code行
            commands = self.parse_gcode_line(line)
            
            for command in commands:
                if 'G' in command:
                    g_code = int(command['G'])
                    
                    # 处理运动指令
                    if g_code in [0, 1, 2, 3]:  # G00, G01, G02, G03
                        # 更新目标位置
                        for axis in ['X', 'Y', 'Z']:
                            if axis in command:
                                value = command[axis]
                                
                                # 单位转换
                                if self.unit_mode == 'G20':  # 英寸转毫米
                                    value *= 25.4
                                
                                # 坐标模式处理
                                if self.coordinate_mode == 'G90':  # 绝对坐标
                                    current_pos[axis] = value
                                else:  # G91 相对坐标
                                    current_pos[axis] += value
                        
                        # 添加轨迹点
                        self.trajectory_points.append(current_pos.copy())
        
        # 更新轨迹显示
        self.update_trajectory_display()
    
    def update_trajectory_display(self):
        """更新3D轨迹显示"""
        if not self.trajectory_points:
            # 如果没有G-code轨迹，仍然要清空显示并设置基本图形
            self.trajectory_ax.clear()
            self.setup_trajectory_plot()
            self.trajectory_canvas.draw()
            return
        
        self.trajectory_ax.clear()
        self.setup_trajectory_plot()
        
        # 提取坐标
        x_coords = [point['X'] for point in self.trajectory_points]
        y_coords = [point['Y'] for point in self.trajectory_points]
        z_coords = [point['Z'] for point in self.trajectory_points]
        
        # 绘制轨迹线
        if len(x_coords) > 1:
            self.trajectory_line = self.trajectory_ax.plot(x_coords, y_coords, z_coords, 
                                                         'b-', linewidth=2, alpha=0.7, label='Planned Trajectory')[0]
        
        # 绘制起始点
        if x_coords:
            self.trajectory_ax.scatter([x_coords[0]], [y_coords[0]], [z_coords[0]], 
                                     c='green', s=100, marker='o', label='Start')
        
        # 绘制结束点
        if len(x_coords) > 1:
            self.trajectory_ax.scatter([x_coords[-1]], [y_coords[-1]], [z_coords[-1]], 
                                     c='red', s=100, marker='s', label='End')
        
        # 添加图例
        self.trajectory_ax.legend()
        
        # 自动调整坐标轴范围
        if x_coords and y_coords and z_coords:
            x_range = max(x_coords) - min(x_coords)
            y_range = max(y_coords) - min(y_coords)
            z_range = max(z_coords) - min(z_coords)
            
            if x_range > 0:
                self.trajectory_ax.set_xlim(min(x_coords) - x_range*0.1, max(x_coords) + x_range*0.1)
            if y_range > 0:
                self.trajectory_ax.set_ylim(min(y_coords) - y_range*0.1, max(y_coords) + y_range*0.1)
            if z_range > 0:
                self.trajectory_ax.set_zlim(min(z_coords) - z_range*0.1, max(z_coords) + z_range*0.1)
        
        self.trajectory_canvas.draw()
    
    def update_current_trajectory_position(self, line_index):
        """更新当前执行位置在轨迹上的显示"""
        if not self.trajectory_points or line_index >= len(self.trajectory_points):
            return
        
        # 移除之前的当前位置标记 - 修复方法
        if hasattr(self, 'current_point_marker') and self.current_point_marker:
            try:
                self.current_point_marker.remove()
            except (NotImplementedError, AttributeError):
                # 对于scatter对象，需要清除整个axes然后重新绘制
                pass
        
        # 清除axes并重新绘制所有内容
        self.trajectory_ax.clear()
        
        # 重新设置axes属性
        self.trajectory_ax.set_xlabel('X (mm)')
        self.trajectory_ax.set_ylabel('Y (mm)')
        self.trajectory_ax.set_zlabel('Z (mm)')
        self.trajectory_ax.set_title('TCP Trajectory Visualization')
        
        # 重新绘制G-code轨迹
        if self.trajectory_points:
            x_coords = [point['X'] for point in self.trajectory_points]
            y_coords = [point['Y'] for point in self.trajectory_points]
            z_coords = [point['Z'] for point in self.trajectory_points]
            self.trajectory_ax.plot(x_coords, y_coords, z_coords, 
                                  'b-', linewidth=2, alpha=0.6, label='G-code Trajectory')
        
        # 重新绘制实时轨迹
        if hasattr(self, 'realtime_trajectory_points') and self.realtime_trajectory_points:
            real_x = [point['X'] for point in self.realtime_trajectory_points]
            real_y = [point['Y'] for point in self.realtime_trajectory_points]
            real_z = [point['Z'] for point in self.realtime_trajectory_points]
            self.trajectory_ax.plot(real_x, real_y, real_z, 
                                  'r-', linewidth=3, alpha=0.9, label='Actual Trajectory')
        
        # 添加当前位置标记
        current_point = self.trajectory_points[line_index]
        self.current_point_marker = self.trajectory_ax.scatter([current_point['X']], 
                                                             [current_point['Y']], 
                                                             [current_point['Z']], 
                                                             c='yellow', s=150, marker='*', 
                                                             edgecolors='black', linewidth=2,
                                                             label='Current Position')
        
        # 添加图例
        self.trajectory_ax.legend()
        
        self.trajectory_canvas.draw()

    def calculate_tcp_position_from_feedback(self):
        """根据反馈关节角度计算实时TCP位置"""
        try:
            # 检查是否有足够的反馈数据
            if not hasattr(self, 'last_feedback_pos') or len(self.last_feedback_pos) < 7:
                return None
                
            # 获取当前反馈角度（转换为弧度）
            feedback_angles_rad = [0]  # 添加基座固定关节
            for i in range(7):
                angle_deg = self.last_feedback_pos[i]
                feedback_angles_rad.append(math.radians(angle_deg))
            
            # 使用正运动学计算TCP位置
            tcp_matrix = self.arm_chain.forward_kinematics(feedback_angles_rad)
            tcp_position = tcp_matrix[:3, 3]  # 提取位置向量
            
            # 转换为毫米单位（如果需要）
            tcp_pos = {
                'X': float(tcp_position[0] * 1000),  # 转换为毫米
                'Y': float(tcp_position[1] * 1000),
                'Z': float(tcp_position[2] * 1000)
            }
            
            # 添加调试信息
            print(f"TCP Position: X={tcp_pos['X']:.2f}, Y={tcp_pos['Y']:.2f}, Z={tcp_pos['Z']:.2f}")
            
            return tcp_pos
            
        except Exception as e:
            print(f"TCP position calculation failed: {e}")
            return None
    
    def update_realtime_tcp_trajectory(self):
        """更新实时TCP轨迹显示"""
        # 计算当前TCP位置
        tcp_pos = self.calculate_tcp_position_from_feedback()
        if tcp_pos is None:
            return
            
        # 添加到实时轨迹点列表
        if not hasattr(self, 'realtime_trajectory_points'):
            self.realtime_trajectory_points = []
            
        # 检查是否与上一个点距离足够远（避免重复点）
        if self.realtime_trajectory_points:
            last_point = self.realtime_trajectory_points[-1]
            distance = math.sqrt(
                (tcp_pos['X'] - last_point['X'])**2 + 
                (tcp_pos['Y'] - last_point['Y'])**2 + 
                (tcp_pos['Z'] - last_point['Z'])**2
            )
            # 只有当移动距离大于1mm时才添加新点
            if distance < 1.0:
                return
                
        self.realtime_trajectory_points.append(tcp_pos.copy())
        
        # 添加调试信息
        print(f"实时轨迹点数量: {len(self.realtime_trajectory_points)}")
        
        # 限制轨迹点数量（保留最近1000个点）
        if len(self.realtime_trajectory_points) > 1000:
            self.realtime_trajectory_points.pop(0)
            
        # 更新轨迹显示
        self.update_realtime_trajectory_display(tcp_pos)
    
    def update_realtime_trajectory_display(self, current_tcp_pos):
        """更新实时轨迹显示"""
        # 清除之前的显示
        self.trajectory_ax.clear()
        
        # 重新设置axes属性
        self.trajectory_ax.set_xlabel('X (mm)')
        self.trajectory_ax.set_ylabel('Y (mm)')
        self.trajectory_ax.set_zlabel('Z (mm)')
        self.trajectory_ax.set_title('TCP Trajectory Visualization')
        
        # 显示G-code预设轨迹（蓝色半透明）
        if hasattr(self, 'trajectory_points') and self.trajectory_points:
            x_coords = [point['X'] for point in self.trajectory_points]
            y_coords = [point['Y'] for point in self.trajectory_points]
            z_coords = [point['Z'] for point in self.trajectory_points]
            
            self.trajectory_ax.plot(x_coords, y_coords, z_coords, 
                                  'b-', linewidth=2, alpha=0.6, label='G-code Trajectory')
        
        # 显示实时轨迹（红色）- 需要至少2个点才能画线
        if hasattr(self, 'realtime_trajectory_points') and len(self.realtime_trajectory_points) >= 2:
            real_x = [point['X'] for point in self.realtime_trajectory_points]
            real_y = [point['Y'] for point in self.realtime_trajectory_points]
            real_z = [point['Z'] for point in self.realtime_trajectory_points]
            
            self.trajectory_ax.plot(real_x, real_y, real_z, 
                                  'r-', linewidth=4, alpha=1.0, label='Actual Trajectory')
            
            print(f"绘制实时轨迹线，点数: {len(self.realtime_trajectory_points)}")
        
        # 始终显示当前TCP位置标记（全程动态显示）
        if current_tcp_pos:
            self.current_point_marker = self.trajectory_ax.scatter([current_tcp_pos['X']], 
                                                                   [current_tcp_pos['Y']], 
                                                                   [current_tcp_pos['Z']], 
                                                                   c='yellow', s=200, marker='*', 
                                                                   edgecolors='black', linewidth=2,
                                                                   label='Current TCP Position')
        
        # 添加图例
        self.trajectory_ax.legend()
        
        # 刷新显示
        self.trajectory_canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmGUI(root)
    root.mainloop()
