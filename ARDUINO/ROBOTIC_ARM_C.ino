#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// Bluetooth setup
SoftwareSerial btSerial(8, 9); // RX=8, TX=9

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo constants
#define NUM_SERVOS 6
#define SERVO_MIN 150
#define SERVO_MAX 600
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define FREQUENCY 50

// Enhanced movement parameters
#define UPDATE_INTERVAL 15      // Faster update rate for smoother movement
#define MIN_STEP_SIZE 1         // Minimum step size for precision
#define MAX_STEP_SIZE 15        // Maximum step size for speed
#define ACCELERATION_STEPS 5    // Steps to reach full speed
#define DECELERATION_DISTANCE 10 // Distance to start deceleration

// Servo data structure for better organization
struct ServoData {
  int current_pos;
  int target_pos;
  int saved_pos;
  int velocity;
  int acceleration_counter;
  bool is_moving;
};

// Servo arrays and variables
ServoData servos[NUM_SERVOS];
int initial_positions[NUM_SERVOS] = {0, 90, 90, 90, 90, 0};
int speed_setting = 50;           // 0-100 speed setting
unsigned long last_update = 0;
bool is_running = true;
bool bluetooth_connected = false;

// Command parsing variables
String input_buffer = "";
bool string_complete = false;

// Smoothing and interpolation functions
float easeInOutQuad(float t) {
  return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
}

int calculateStepSize(int distance, int speed, int accel_counter) {
  // Calculate base step size from speed setting (1-100)
  int base_step = map(speed, 1, 100, MIN_STEP_SIZE, MAX_STEP_SIZE);
  
  // Apply acceleration/deceleration
  float acceleration_factor = 1.0;
  
  if (accel_counter < ACCELERATION_STEPS) {
    // Accelerating
    acceleration_factor = easeInOutQuad((float)accel_counter / ACCELERATION_STEPS);
  } else if (abs(distance) < DECELERATION_DISTANCE) {
    // Decelerating
    acceleration_factor = easeInOutQuad((float)abs(distance) / DECELERATION_DISTANCE);
  }
  
  int step_size = (int)(base_step * acceleration_factor);
  return max(MIN_STEP_SIZE, step_size);
}

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  
  Serial.println("Robotic Arm Controller v2.0");
  Serial.println("Initializing...");
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(100);
  
  // Initialize servo data
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].current_pos = initial_positions[i];
    servos[i].target_pos = initial_positions[i];
    servos[i].saved_pos = initial_positions[i];
    servos[i].velocity = 0;
    servos[i].acceleration_counter = 0;
    servos[i].is_moving = false;
    
    // Set initial servo positions
    setPWMValue(i, initial_positions[i]);
  }
  
  // Send initial status
  btSerial.println("Disconnected");
  Serial.println("Setup complete. Waiting for commands...");
  
  // Reserve string buffer for command parsing
  input_buffer.reserve(50);
}

void loop() {
  // Handle serial communication
  handleSerialInput();
  
  // Process complete commands
  if (string_complete) {
    processCommand(input_buffer);
    input_buffer = "";
    string_complete = false;
  }
  
  // Update servo positions
  if (millis() - last_update >= UPDATE_INTERVAL && is_running) {
    updateServoPositions();
    last_update = millis();
  }
  
  // Small delay to prevent overwhelming the processor
  delay(1);
}

void handleSerialInput() {
  while (btSerial.available()) {
    char incoming_char = (char)btSerial.read();
    
    if (incoming_char == '\n' || incoming_char == '\r') {
      if (input_buffer.length() > 0) {
        string_complete = true;
      }
    } else {
      input_buffer += incoming_char;
    }
  }
}

void processCommand(String command) {
  command.trim();
  Serial.println("Received: " + command);
  
  if (command == "Connect") {
    bluetooth_connected = true;
    btSerial.println("Connected");
    Serial.println("Bluetooth connected");
  }
  else if (command == "Disconnect") {
    bluetooth_connected = false;
    btSerial.println("Disconnected");
    Serial.println("Bluetooth disconnected");
    
    // Turn off all servos
    for (int i = 0; i < NUM_SERVOS; i++) {
      pwm.setPWM(i, 0, 0);
    }
  }
  else if (command.startsWith("s") && command.length() >= 3) {
    // Servo position command: s1180 (servo 1 to 180 degrees)
    int servo_num = command.substring(1, 2).toInt() - 1; // Convert to 0-based index
    
    if (servo_num >= 0 && servo_num < NUM_SERVOS && is_running) {
      int position = command.substring(2).toInt();
      position = constrain(position, 0, 180);
      
      setServoTarget(servo_num, position);
      Serial.println("Servo " + String(servo_num + 1) + " target: " + String(position));
    }
  }
  else if (command.startsWith("ss") && command.length() >= 3) {
    // Speed setting command: ss50 (speed 50%)
    int new_speed = command.substring(2).toInt();
    speed_setting = constrain(new_speed, 1, 100);
    Serial.println("Speed set to: " + String(speed_setting));
  }
  else if (command == "SAVE") {
    saveCurrentPositions();
    btSerial.println("Positions saved: 1");
    Serial.println("Positions saved");
  }
  else if (command == "RUN") {
    loadSavedPositions();
    Serial.println("Running saved positions");
  }
  else if (command == "RESET") {
    resetToHome();
    Serial.println("Reset to home position");
  }
  else if (command == "PAUSE") {
    is_running = false;
    Serial.println("Movement paused");
  }
  else if (command == "RESUME") {
    is_running = true;
    Serial.println("Movement resumed");
  }
  else if (command == "STATUS") {
    sendStatusReport();
  }
  else {
    Serial.println("Unknown command: " + command);
  }
}

void setServoTarget(int servo_num, int target_position) {
  if (servo_num >= 0 && servo_num < NUM_SERVOS) {
    servos[servo_num].target_pos = target_position;
    servos[servo_num].is_moving = (servos[servo_num].current_pos != target_position);
    servos[servo_num].acceleration_counter = 0;
  }
}

void updateServoPositions() {
  bool any_moving = false;
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].is_moving) {
      int distance = servos[i].target_pos - servos[i].current_pos;
      
      if (abs(distance) > 0) {
        // Calculate step size with acceleration/deceleration
        int step_size = calculateStepSize(distance, speed_setting, servos[i].acceleration_counter);
        
        // Move towards target
        if (distance > 0) {
          servos[i].current_pos += min(step_size, distance);
        } else {
          servos[i].current_pos += max(-step_size, distance);
        }
        
        // Update PWM
        setPWMValue(i, servos[i].current_pos);
        
        // Increment acceleration counter
        servos[i].acceleration_counter++;
        any_moving = true;
        
        // Check if reached target
        if (servos[i].current_pos == servos[i].target_pos) {
          servos[i].is_moving = false;
          servos[i].acceleration_counter = 0;
        }
      } else {
        servos[i].is_moving = false;
        servos[i].acceleration_counter = 0;
      }
    }
  }
}

void setPWMValue(int servo_num, int angle) {
  // Enhanced PWM calculation with better resolution
  if (servo_num >= 0 && servo_num < NUM_SERVOS) {
    // Map angle to pulse width (microseconds)
    int pulse_width = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    
    // Convert to PWM value (4096 steps for 12-bit resolution)
    int pwm_value = map(pulse_width, 0, 20000, 0, 4096);  // 20ms period
    pwm_value = constrain(pwm_value, SERVO_MIN, SERVO_MAX);
    
    pwm.setPWM(servo_num, 0, pwm_value);
  }
}

void saveCurrentPositions() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].saved_pos = servos[i].current_pos;
  }
}

void loadSavedPositions() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoTarget(i, servos[i].saved_pos);
  }
}

void resetToHome() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoTarget(i, initial_positions[i]);
  }
}

void sendStatusReport() {
  Serial.println("=== STATUS REPORT ===");
  Serial.println("Bluetooth: " + String(bluetooth_connected ? "Connected" : "Disconnected"));
  Serial.println("Running: " + String(is_running ? "Yes" : "No"));
  Serial.println("Speed: " + String(speed_setting));
  
  Serial.println("Servo Positions:");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.println("  Servo " + String(i + 1) + ": " + 
                  String(servos[i].current_pos) + "° -> " + 
                  String(servos[i].target_pos) + "° " +
                  (servos[i].is_moving ? "[MOVING]" : "[STOPPED]"));
  }
  Serial.println("====================");
}

// Emergency stop function
void emergencyStop() {
  is_running = false;
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].target_pos = servos[i].current_pos;
    servos[i].is_moving = false;
  }
  Serial.println("EMERGENCY STOP ACTIVATED");
}

// Calibration function for fine-tuning servo limits
void calibrateServo(int servo_num, int min_pulse, int max_pulse) {
  if (servo_num >= 0 && servo_num < NUM_SERVOS) {
    // This could be extended to store per-servo calibration values
    Serial.println("Calibrating servo " + String(servo_num + 1));
    // Implementation would store custom min/max values per servo
  }
}

// Smooth trajectory planning for multiple servos
void moveMultipleServos(int positions[], int num_positions, int duration_ms) {
  if (num_positions > NUM_SERVOS) num_positions = NUM_SERVOS;
  
  // Calculate synchronized movement
  for (int i = 0; i < num_positions; i++) {
    setServoTarget(i, positions[i]);
  }
  
  Serial.println("Multi-servo move initiated");
}

// Power management - reduce servo power when not moving
void managePower() {
  static unsigned long last_movement_time = 0;
  static bool power_reduced = false;
  
  // Check if any servo is moving
  bool any_moving = false;
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].is_moving) {
      any_moving = true;
      last_movement_time = millis();
      break;
    }
  }
  
  // Reduce power if no movement for 30 seconds
  if (!any_moving && !power_reduced && (millis() - last_movement_time > 30000)) {
    // Reduce PWM frequency to save power
    pwm.setPWMFreq(25); // Reduce from 50Hz to 25Hz
    power_reduced = true;
    Serial.println("Power saving mode activated");
  } else if (any_moving && power_reduced) {
    // Restore full power
    pwm.setPWMFreq(FREQUENCY);
    power_reduced = false;
    Serial.println("Full power restored");
  }
}

// Advanced command parsing for complex movements
void parseComplexCommand(String command) {
  // Example: "MOVE:1,90;2,45;3,135" - move multiple servos simultaneously
  if (command.startsWith("MOVE:")) {
    String move_data = command.substring(5);
    
    // Parse servo positions
    int start_pos = 0;
    while (start_pos < move_data.length()) {
      int comma_pos = move_data.indexOf(',', start_pos);
      int semicolon_pos = move_data.indexOf(';', start_pos);
      
      if (comma_pos == -1) break;
      
      int servo_num = move_data.substring(start_pos, comma_pos).toInt() - 1;
      int end_pos = (semicolon_pos == -1) ? move_data.length() : semicolon_pos;
      int position = move_data.substring(comma_pos + 1, end_pos).toInt();
      
      if (servo_num >= 0 && servo_num < NUM_SERVOS) {
        setServoTarget(servo_num, constrain(position, 0, 180));
      }
      
      start_pos = (semicolon_pos == -1) ? move_data.length() : semicolon_pos + 1;
    }
  }
}

// Watchdog timer simulation - reset if no communication
void checkCommunicationTimeout() {
  static unsigned long last_command_time = 0;
  static bool timeout_warning_sent = false;
  
  if (bluetooth_connected) {
    // Reset timeout on any command
    if (string_complete) {
      last_command_time = millis();
      timeout_warning_sent = false;
    }
    
    // Check for timeout (5 minutes of no communication)
    if (millis() - last_command_time > 300000) {
      if (!timeout_warning_sent) {
        Serial.println("WARNING: Communication timeout");
        btSerial.println("Communication timeout - entering safe mode");
        timeout_warning_sent = true;
      }
      
      // Enter safe mode - slowly move to home position
      if (millis() - last_command_time > 360000) { // 6 minutes total
        Serial.println("Entering safe mode - moving to home");
        resetToHome();
        last_command_time = millis(); // Reset to prevent continuous reset
      }
    }
  }
}
