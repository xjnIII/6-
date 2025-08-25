/**
 * ESP32 position motion control for a 7-DOF Robotic Arm
 * This code is a template for each of the 7 motor controllers.
 * CHANGE THE MOTOR_ID FOR EACH ESP32.
 */
#include <SimpleFOC.h>
#include <esp_now.h>
#include <WiFi.h>

// --- IMPORTANT ---
// CHANGE THIS FOR EACH MOTOR CONTROLLER (0, 1, 2, 3, 4, 5, or 6)
#define MOTOR_ID 1

const long SERIAL_REFRESH_TIME = 1000;
long refresh_time;

// Message from Gateway to all motors - contains all 7 joint angles
typedef struct struct_message {
  double angles[7];
} struct_message;

// Feedback from this motor to the Gateway
typedef struct struct_feedback {
  uint8_t motor_id;
  float position; // in degrees
  float voltage;
} struct_feedback;

struct_message myData;
struct_feedback myFeedback;

esp_now_peer_info_t peerInfo_reply;
uint8_t senderMacAddress[6] = { 0, 0, 0, 0, 0, 0 };
bool senderMacAddressKnown = false;

// Magnetic Sensor and Motor Configuration (remains the same)
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);

float target_angle = 0; // Target angle in radians for SimpleFOC

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  if (!senderMacAddressKnown) {
    memcpy(senderMacAddress, mac, 6);
    memcpy(peerInfo_reply.peer_addr, senderMacAddress, 6);
    peerInfo_reply.channel = 0;
    peerInfo_reply.encrypt = false;
    if (esp_now_add_peer(&peerInfo_reply) != ESP_OK) {
      Serial.println("Failed to add reply peer");
      return;
    }
    senderMacAddressKnown = true;
  }

  if (len == sizeof(myData)) {
    memcpy(&myData, incomingData, sizeof(myData));

    // Extract the target angle for this specific motor and convert to radians
    target_angle = myData.angles[MOTOR_ID] * DEG_TO_RAD;

    if (millis() > refresh_time) {
      refresh_time = millis() + SERIAL_REFRESH_TIME;
      Serial.print("Motor ");
      Serial.print(MOTOR_ID);
      Serial.print(" | New Target: ");
      Serial.print(myData.angles[MOTOR_ID]);
      Serial.println(" deg");
    }
  }
}

void setup() {
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.5;
  motor.voltage_limit = 2;
  motor.voltage_sensor_align = 1;
  motor.LPF_velocity.Tf = 0.05;
  motor.P_angle.P = 20;
  motor.velocity_limit = 20;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  motor.init();
  motor.initFOC();

  Serial.print("Motor ");
  Serial.print(MOTOR_ID);
  Serial.println(" Ready.");
  _delay(1000);
}

long last_feedback_time = 0;
long last_debug_print_time = 0; // For printing debug info

void loop() {
  motor.loopFOC();

  // Dynamically adjust voltage limit based on position error.
  // This reduces motor heat and power consumption when holding a position.
  float error = target_angle - motor.shaftAngle();
  if (abs(error) < 0.02) { // If error is less than ~1.15 degrees, motor is considered stationary.
    motor.voltage_limit = 0.3; // Apply low voltage for holding.
  } else {
    motor.voltage_limit = 2;   // Apply full voltage for movement.
  }

  motor.move(target_angle);

  // Send feedback periodically
  if (senderMacAddressKnown && (millis() - last_feedback_time > 200)) {
    last_feedback_time = millis();
    
    myFeedback.motor_id = MOTOR_ID;
    myFeedback.position = motor.shaftAngle() * RAD_TO_DEG; // Send position in degrees
    myFeedback.voltage = motor.voltage.q;

    esp_now_send(senderMacAddress, (uint8_t*)&myFeedback, sizeof(myFeedback));
  }
  
  // --- Add this new block for debugging ---
  // Print target vs actual angle to see the drift
  if (millis() - last_debug_print_time > 100) {
    last_debug_print_time = millis();
    Serial.print("Target: ");
    Serial.print(target_angle * RAD_TO_DEG);
    Serial.print("\t Actual: ");
    Serial.print(motor.shaftAngle() * RAD_TO_DEG);
    Serial.print("\t Error: ");
    Serial.print((target_angle - motor.shaftAngle()) * RAD_TO_DEG);
    Serial.print("\t Voltage: ");
    Serial.println(motor.voltage_limit);
  }
}
