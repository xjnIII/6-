#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h> // NEW: Include for isnan() and isinf()

// 完整的 LovyanGFX 配置
struct LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01     _panel_instance;
  lgfx::Bus_SPI       _bus_instance;

  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 80000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 6;
      cfg.pin_mosi = 7;
      cfg.pin_miso = -1;
      cfg.pin_dc   = 2;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 10;
      cfg.pin_rst          = -1;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 240;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = true;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;

// --- NEW: Define rainbow colors for each joint's indicator ---
const uint16_t motor_colors[] = {
  TFT_RED, TFT_ORANGE, TFT_YELLOW, TFT_GREEN, TFT_CYAN, TFT_BLUE, TFT_VIOLET
};

uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// 发送到所有电机的消息结构体
typedef struct struct_message {
  double angles[7];
} struct_message;

// 从单个电机接收的反馈结构体
typedef struct struct_feedback {
  uint8_t motor_id;
  float position;
  float voltage;
} struct_feedback;

struct_message myData;
esp_now_peer_info_t peerInfo;

// 当从电机接收到反馈时的回调函数
// Callback when feedback is received from a motor
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_feedback)) {
    return; // Ignore if data length is wrong
  }
  struct_feedback feedbackData;
  memcpy(&feedbackData, incomingData, sizeof(feedbackData));
  
  // Forward RAW feedback to PC via Serial
  Serial.print("FB:");
  Serial.print(feedbackData.motor_id);
  Serial.print(",");
  Serial.print(feedbackData.position, 2);
  Serial.print(",");
  Serial.println(feedbackData.voltage, 2);

  // --- Apply gear ratio for local display ---
  const float GEAR_RATIO = 50.0;
  float display_position = feedbackData.position / GEAR_RATIO;

  // --- RAINBOW LAYOUT REVISION: Move to bottom half ---
  if (feedbackData.motor_id < 7) {
    // --- Data validation ---
    if (isnan(display_position) || isinf(display_position)) {
      display_position = 0.0;
    }
    
    // --- Arc parameters for bottom rainbow layout ---
    int center_x = tft.width() / 2;
    int center_y = tft.height() / 2;
    
    // MODIFIED: Rainbow is a semi-circle at the bottom (0 to 180 degrees)
    float rainbow_start_angle = 0;
    float rainbow_total_sweep = 180;

    // Each motor gets a parallel arc. M0 is outermost.
    int arc_radius = 112 - (feedbackData.motor_id * 2);

    // Map motor position (-180 to 180) to an angle sweep
    // USE THE SCALED VALUE for display
    float normalized_pos = (display_position + 180.0) / 360.0;
    if (normalized_pos < 0.0) normalized_pos = 0.0;
    if (normalized_pos > 1.0) normalized_pos = 1.0;
    
    float dynamic_end_angle = rainbow_start_angle + (normalized_pos * rainbow_total_sweep);

    // --- REFRESH LOGIC ---
    // 1. Erase the full potential path of the arc for this motor.
    float rainbow_end_angle = rainbow_start_angle + rainbow_total_sweep;
    tft.drawArc(center_x, center_y, arc_radius, arc_radius, rainbow_start_angle, rainbow_end_angle, TFT_BLACK);
    
    // 2. Draw the new, updated arc in its color.
    tft.drawArc(center_x, center_y, arc_radius, arc_radius, rainbow_start_angle, dynamic_end_angle, motor_colors[feedbackData.motor_id]);
  }

  // --- Display centered text (remains the same) ---
  char display_buf[50];
  // USE THE SCALED VALUE for display
  sprintf(display_buf, "M%d:%-4d  V:%.2f", feedbackData.motor_id, (int)display_position, feedbackData.voltage);

  tft.setTextDatum(lgfx::v1::textdatum_t::top_center);
  tft.setFont(&lgfx::fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);

  int y_pos = 65 + (feedbackData.motor_id * 22);

  tft.fillRect(0, y_pos, 240, 20, TFT_BLACK);
  tft.drawString(display_buf, tft.width() / 2, y_pos);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 可选: 在串口打印发送状态
}

void setup() {
  Serial.begin(115200);
  
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  
  tft.setFont(&lgfx::fonts::efontCN_24);
  tft.setTextDatum(lgfx::v1::textdatum_t::top_center);
  tft.setTextColor(TFT_WHITE);
  // Move title down to be visible on a round screen
  tft.drawString("7-DOF Arm ", tft.width()/2, 30);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Gateway Ready. Waiting for commands...");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    
    char data_buf[data.length() + 1];
    data.toCharArray(data_buf, sizeof(data_buf));

    char *token = strtok(data_buf, ",");
    int i = 0;
    while (token != NULL && i < 7) {
      myData.angles[i] = atof(token);
      token = strtok(NULL, ",");
      i++;
    }

    if (i == 7) {
      // 将7个角度发送给所有电机
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      if (result != ESP_OK) {
        Serial.println("Error sending data");
      }
    } else {
      Serial.println("Invalid data format. Expected 7 comma-separated values.");
    }
  }
}
