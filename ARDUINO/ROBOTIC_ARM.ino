#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ======================= CONFIGURATION =======================
#define NUM_SERVOS 6
#define SERVO_MIN 150
#define SERVO_MAX 600
#define FREQUENCY 50

#define BASE_UPDATE_INTERVAL 10     // Base loop interval in ms
#define MIN_STEP_SIZE 0.1           // Degrees per step (sub-degree precision)
#define MAX_STEP_SIZE 5.0           // Max degrees per step (high speed)
#define ACCELERATION 0.08           // Acceleration factor (higher = faster ramp-up)

struct ServoData {
  float current_pos;    // Current position in degrees
  float target_pos;     // Target position in degrees
  float velocity;       // Current speed in deg/update
  float saved_pos;
  bool is_moving;
};

ServoData servos[NUM_SERVOS];
float initial_positions[NUM_SERVOS] = {90, 90, 90, 90, 90, 0};
int speed_setting = 50;  // 1–100
bool is_running = true;
bool system_connected = false;

String input_buffer = "";
unsigned long last_update = 0;

// ======================= SETUP =======================
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1);

  Serial.println(F("=== ROBOTIC ARM CONTROLLER v3.0 ==="));
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(100);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].current_pos = initial_positions[i];
    servos[i].target_pos = initial_positions[i];
    servos[i].saved_pos = initial_positions[i];
    servos[i].velocity = 0;
    servos[i].is_moving = false;
    setPWMValue(i, initial_positions[i]);
    delay(50);
  }

  Serial.println(F("✅ PCA9685 initialized"));
  Serial.println(F("✅ All servos moved to home position"));
  Serial.println(F("🚀 Ready for commands!"));
}

// ======================= MAIN LOOP =======================
void loop() {
  handleSerialInput();
  if (millis() - last_update >= BASE_UPDATE_INTERVAL && is_running && system_connected) {
    updateServoPositions();
    last_update = millis();
  }
}

// ======================= COMMAND HANDLER =======================
void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (input_buffer.length() > 0) {
        processCommand(input_buffer);
        input_buffer = "";
      }
    } else {
      input_buffer += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.equalsIgnoreCase("Connect")) {
    system_connected = true;
    Serial.println(F("✅ Connected - System Active"));
  }
  else if (cmd.equalsIgnoreCase("Disconnect")) {
    system_connected = false;
    stopAllServos();
    Serial.println(F("❌ Disconnected - System Inactive"));
  }
  else if (cmd.startsWith("s") && cmd.length() >= 3) {
    int servo_num = cmd.substring(1, 2).toInt() - 1;
    if (servo_num >= 0 && servo_num < NUM_SERVOS) {
      float pos = cmd.substring(2).toFloat();
      pos = constrain(pos, 0, 180);
      if (system_connected) {
        setServoTarget(servo_num, pos);
      }
    }
  }
  else if (cmd.startsWith("ss")) {
    int new_speed = cmd.substring(2).toInt();
    speed_setting = constrain(new_speed, 1, 100);
    Serial.println("⚡ Speed set to " + String(speed_setting) + "%");
  }
  else if (cmd.equalsIgnoreCase("SAVE")) saveCurrentPositions();
  else if (cmd.equalsIgnoreCase("RUN")) loadSavedPositions();
  else if (cmd.equalsIgnoreCase("RESET")) resetToHome();
  else if (cmd.equalsIgnoreCase("PAUSE")) is_running = false;
  else if (cmd.equalsIgnoreCase("RESUME")) is_running = true;
  else if (cmd.equalsIgnoreCase("STATUS")) sendStatusReport();
  else if (cmd.equalsIgnoreCase("HOME")) moveAllHome();
  else Serial.println("❓ Unknown command: " + cmd);
}

// ======================= MOVEMENT ENGINE =======================
void setServoTarget(int id, float pos) {
  servos[id].target_pos = pos;
  servos[id].is_moving = true;
}

void updateServoPositions() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].is_moving) {
      float distance = servos[i].target_pos - servos[i].current_pos;
      float direction = (distance > 0) ? 1 : -1;

      // Smooth acceleration profile
      servos[i].velocity += ACCELERATION * direction;
      float maxVel = map(speed_setting, 1, 100, MIN_STEP_SIZE * 100, MAX_STEP_SIZE * 100) / 100.0;
      servos[i].velocity = constrain(servos[i].velocity, -maxVel, maxVel);

      // Decelerate when close
      if (abs(distance) < fabs(servos[i].velocity)) {
        servos[i].velocity = distance;
      }

      // Move servo
      servos[i].current_pos += servos[i].velocity;
      setPWMValue(i, servos[i].current_pos);

      // Stop condition
      if (fabs(distance) < 0.05) {
        servos[i].current_pos = servos[i].target_pos;
        servos[i].velocity = 0;
        servos[i].is_moving = false;
      }
    }
  }
}

void setPWMValue(int servo_num, float angle) {
  int pwm_value = map(angle * 10, 0, 1800, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(servo_num, 0, pwm_value);
}

void saveCurrentPositions() {
  for (int i = 0; i < NUM_SERVOS; i++) servos[i].saved_pos = servos[i].current_pos;
}

void loadSavedPositions() {
  for (int i = 0; i < NUM_SERVOS; i++) setServoTarget(i, servos[i].saved_pos);
}

void resetToHome() {
  for (int i = 0; i < NUM_SERVOS; i++) setServoTarget(i, initial_positions[i]);
}

void moveAllHome() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].current_pos = initial_positions[i];
    servos[i].target_pos = initial_positions[i];
    setPWMValue(i, initial_positions[i]);
  }
}

void stopAllServos() {
  for (int i = 0; i < NUM_SERVOS; i++) pwm.setPWM(i, 0, 0);
}

void sendStatusReport() {
  Serial.println(F("=== STATUS REPORT ==="));
  Serial.println("System: " + String(system_connected ? "Connected" : "Disconnected"));
  Serial.println("Speed: " + String(speed_setting) + "%");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print("Servo ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(servos[i].current_pos, 1);
    Serial.print("° -> ");
    Serial.println(servos[i].target_pos, 1);
  }
}
