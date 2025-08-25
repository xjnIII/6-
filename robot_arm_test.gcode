; 机械臂测试程序
; Robot Arm Test Program
; 适用于7关节机械臂的基础功能测试

; 初始化设置
G21 ; 设置单位为毫米
G90 ; 绝对坐标模式
G94 ; 进给速度模式

; 测试开始提示
; Test Start

; 1. 回零测试
; Home Position Test
G28 ; 回到零点
G04 P2000 ; 暂停2秒

; 2. 基本位置测试
; Basic Position Test
G01 X100 Y0 Z100 F500 ; 移动到基础位置
G04 P1000 ; 暂停1秒

G01 X150 Y0 Z100 F500 ; X轴正方向测试
G04 P1000

G01 X100 Y100 Z100 F500 ; Y轴正方向测试
G04 P1000

G01 X100 Y0 Z150 F500 ; Z轴正方向测试
G04 P1000

; 3. 工作空间边界测试
; Workspace Boundary Test
G01 X200 Y0 Z100 F300 ; 测试X轴最大范围
G04 P1000

G01 X100 Y200 Z100 F300 ; 测试Y轴最大范围
G04 P1000

G01 X100 Y0 Z200 F300 ; 测试Z轴最大范围
G04 P1000

; 4. 方形路径测试
; Square Path Test
G01 X50 Y50 Z120 F400 ; 移动到方形起点
G04 P500

G01 X150 Y50 Z120 F400 ; 方形第一边
G04 P500

G01 X150 Y150 Z120 F400 ; 方形第二边
G04 P500

G01 X50 Y150 Z120 F400 ; 方形第三边
G04 P500

G01 X50 Y50 Z120 F400 ; 方形第四边
G04 P500

; 5. 圆形路径测试（简化为八边形）
; Circular Path Test (Simplified as Octagon)
G01 X100 Y100 Z130 F300 ; 移动到圆心
G04 P500

; 八边形路径
G01 X150 Y100 Z130 F300 ; 0度
G04 P300
G01 X135 Y135 Z130 F300 ; 45度
G04 P300
G01 X100 Y150 Z130 F300 ; 90度
G04 P300
G01 X65 Y135 Z130 F300 ; 135度
G04 P300
G01 X50 Y100 Z130 F300 ; 180度
G04 P300
G01 X65 Y65 Z130 F300 ; 225度
G04 P300
G01 X100 Y50 Z130 F300 ; 270度
G04 P300
G01 X135 Y65 Z130 F300 ; 315度
G04 P300
G01 X150 Y100 Z130 F300 ; 360度回到起点
G04 P500

; 6. 垂直运动测试
; Vertical Movement Test
G01 X100 Y100 Z80 F200 ; 下降
G04 P1000
G01 X100 Y100 Z180 F200 ; 上升
G04 P1000
G01 X100 Y100 Z130 F200 ; 回到中间位置
G04 P1000

; 7. 速度变化测试
; Speed Variation Test
G01 X120 Y100 Z130 F100 ; 慢速
G04 P500
G01 X80 Y100 Z130 F800 ; 快速
G04 P500
G01 X100 Y100 Z130 F400 ; 中速
G04 P500

; 8. 对角线运动测试
; Diagonal Movement Test
G01 X50 Y50 Z100 F300
G04 P500
G01 X150 Y150 Z150 F300
G04 P500
G01 X150 Y50 Z100 F300
G04 P500
G01 X50 Y150 Z150 F300
G04 P500

; 9. 返回安全位置
; Return to Safe Position
G01 X100 Y100 Z150 F400 ; 安全位置
G04 P1000

; 10. 最终回零
; Final Home
G28 ; 回零
G04 P2000

; 测试完成
; Test Complete
M30 ; 程序结束