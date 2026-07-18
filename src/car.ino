/*
ESP32 Robotic Vehicle + Servo Arm Controller

Features:
- Bluepad32 Bluetooth controller
- 4-wheel motor drive
- 5-servo robotic arm control
- Non-blocking servo updates

Author: Kush Suthar
*/

#include <Bluepad32.h>
#include <ESP32Servo.h>

// motor driver pins
const int FM1A = 27; const int FM1B = 26; // Front Left
const int FM2A = 25; const int FM2B = 33; // Front Right
const int BM1A = 14; const int BM1B = 2;  // Back Left (Pin 14 to avoid strapping issues)
const int BM2A = 23; const int BM2B = 19; // Back Right

// motor PWM channels
#define CH_FM1A 8
#define CH_FM1B 9
#define CH_FM2A 10
#define CH_FM2B 11
#define CH_BM1A 12
#define CH_BM1B 13
#define CH_BM2A 14
#define CH_BM2B 15

// servo arm pins
#define BASE_L_PIN    13  
#define BASE_R_PIN    32   
#define MIDDLE_PIN    15  
#define WRIST_PIN     21  
#define GRIP_PIN      22  

Servo baseLServo; Servo baseRServo; Servo middleServo; Servo wristServo; Servo gripServo;

// arm state tracking
int angleBase = 90;
int angleMiddle = 90;
int angleWrist = 90;
int angleGrip = 90;

const int mediumSpeed = 130; // max speed for the car (fixed)
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// non-blocking timer to prevent servo spamming
unsigned long lastServoTime = 0;
const unsigned long servoDelay = 30; // nnly update servos every 30ms

void setupPWM() {
  ledcSetup(CH_FM1A, 1000, 8); ledcAttachPin(FM1A, CH_FM1A);
  ledcSetup(CH_FM1B, 1000, 8); ledcAttachPin(FM1B, CH_FM1B);
  ledcSetup(CH_FM2A, 1000, 8); ledcAttachPin(FM2A, CH_FM2A);
  ledcSetup(CH_FM2B, 1000, 8); ledcAttachPin(FM2B, CH_FM2B);
  ledcSetup(CH_BM1A, 1000, 8); ledcAttachPin(BM1A, CH_BM1A);
  ledcSetup(CH_BM1B, 1000, 8); ledcAttachPin(BM1B, CH_BM1B);
  ledcSetup(CH_BM2A, 1000, 8); ledcAttachPin(BM2A, CH_BM2A);
  ledcSetup(CH_BM2B, 1000, 8); ledcAttachPin(BM2B, CH_BM2B);
}

void stopMotors() {
  ledcWrite(CH_FM1A, 0); 
  ledcWrite(CH_FM1B, 0);
  ledcWrite(CH_FM2A, 0); 
  ledcWrite(CH_FM2B, 0);
  ledcWrite(CH_BM1A, 0); 
  ledcWrite(CH_BM1B, 0);
  ledcWrite(CH_BM2A, 0); 
  ledcWrite(CH_BM2B, 0);
}

void forward() {
  ledcWrite(CH_FM1A, mediumSpeed); 
  ledcWrite(CH_FM1B, 0);
  ledcWrite(CH_FM2A, mediumSpeed); 
  ledcWrite(CH_FM2B, 0);
  ledcWrite(CH_BM1A, mediumSpeed); 
  ledcWrite(CH_BM1B, 0);
  ledcWrite(CH_BM2A, mediumSpeed); 
  ledcWrite(CH_BM2B, 0);
}

void back() {
  ledcWrite(CH_FM1A, 0); 
  ledcWrite(CH_FM1B, mediumSpeed);
  ledcWrite(CH_FM2A, 0); 
  ledcWrite(CH_FM2B, mediumSpeed);
  ledcWrite(CH_BM1A, 0); 
  ledcWrite(CH_BM1B, mediumSpeed);
  ledcWrite(CH_BM2A, 0); 
  ledcWrite(CH_BM2B, mediumSpeed);
}

void right() {
  ledcWrite(CH_FM1A, mediumSpeed); 
  ledcWrite(CH_FM1B, 0);
  ledcWrite(CH_BM1A, mediumSpeed); 
  ledcWrite(CH_BM1B, 0);
  ledcWrite(CH_FM2A, 0);            
  ledcWrite(CH_FM2B, mediumSpeed);
  ledcWrite(CH_BM2A, 0);            
  ledcWrite(CH_BM2B, mediumSpeed);
}

void left() {
  ledcWrite(CH_FM1A, 0);            
  ledcWrite(CH_FM1B, mediumSpeed);
  ledcWrite(CH_BM1A, 0);            
  ledcWrite(CH_BM1B, mediumSpeed);
  ledcWrite(CH_FM2A, mediumSpeed); 
  ledcWrite(CH_FM2B, 0);
  ledcWrite(CH_BM2A, mediumSpeed); 
  ledcWrite(CH_BM2B, 0);
}

void syncServos() {
  baseLServo.write(180 - angleBase);
  baseRServo.write(angleBase);
  middleServo.write(angleMiddle);
  wristServo.write(angleWrist);
  gripServo.write(angleGrip);
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.println("--- GAMEPAD CONNECTED ---");
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      stopMotors();
      break;
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  if (!ctl->isConnected() || !ctl->hasData()) return;
  if (!ctl->isGamepad()) return;
  
  // car movement
  uint8_t dpadState = ctl->dpad();
  if (dpadState == 0x01)       forward();  
  else if (dpadState == 0x02) back();     
  else if (dpadState == 0x04) right();    
  else if (dpadState == 0x08) left();     
  else                        stopMotors(); 

  // TIMED SERVO UPDATE
  if (millis() - lastServoTime >= servoDelay) {
    lastServoTime = millis();
    bool stateChanged = false;

    // Presets (Only syncs when pressed)
    if (ctl->a()) {  
      angleBase = 90;     
      angleMiddle = 45;   
      stateChanged = true;
    }
    if (ctl->b()) {  
      angleBase = 90;     
      angleMiddle = 130;  
      stateChanged = true;
    }

    // Manual Wrist override (Bumpers)
    uint16_t buttons = ctl->buttons();
    if (buttons & 0x0010) { 
      angleWrist = constrain(angleWrist - 4, 0, 180);
      stateChanged = true;
    }
    if (buttons & 0x0020) { 
      angleWrist = constrain(angleWrist + 4, 0, 180);
      stateChanged = true;
    }

    // Manual Gripper override (Triggers)
    int leftTrigger = ctl->throttle(); 
    int rightTrigger = ctl->brake();   
    if (leftTrigger > 100) {  
      angleGrip = constrain(angleGrip - 4, 0, 180); 
      stateChanged = true;
    }
    else if (rightTrigger > 100) { 
      angleGrip = constrain(angleGrip + 4, 0, 180); 
      stateChanged = true;
    }

    // Manual Base rotation override (Right Stick)
    int rightX = ctl->axisRX(); 
    if (abs(rightX) > 40) { 
      if (rightX > 0) angleBase += 3; 
      else           angleBase -= 3; 
      angleBase = constrain(angleBase, 0, 180); 
      stateChanged = true;      
    }

    // only talk to the servos if an actual arm button/stick was moved!
    // if you are just driving with the dpad, stateChanged stays false, and the arm stays locked
    if (stateChanged) {
      syncServos();
    }
  }
}

void setup() {
  Serial.begin(115200);

  // fully allocate all hardware timers before attaching anything
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // setup car hardware
  setupPWM();
  stopMotors(); 

  // explicitly assign servo refresh rate to separate them from the 1000Hz motor lines
  baseLServo.setPeriodHertz(50);  baseRServo.setPeriodHertz(50);
  middleServo.setPeriodHertz(50); wristServo.setPeriodHertz(50); gripServo.setPeriodHertz(50);

  // servo pins attach
  baseLServo.attach(BASE_L_PIN, 500, 2400);  
  baseRServo.attach(BASE_R_PIN, 500, 2400);  
  middleServo.attach(MIDDLE_PIN, 500, 2400); 
  wristServo.attach(WRIST_PIN, 500, 2400);   
  gripServo.attach(GRIP_PIN, 500, 2400);    

  // default position of the servo arm
  syncServos();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); 
  Serial.println("--- SINGLE ESP32 COEXISTENCE ENGINE ONLINE ---");
}

void loop() {
  if (BP32.update()){ 
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if (myControllers[i] != nullptr) {
        processGamepad(myControllers[i]);
      }
    }
  }
  delayMicroseconds(400); 
}
