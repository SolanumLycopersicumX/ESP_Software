// #include "BLE.cpp"
// #include "C12832.h" // Include for LCD screen
// #include "QEI.h"
// #include "mbed.h"

// C12832 lcd(D11, D13, D12, D7, D10);
// Serial hm10(PA_11, PA_12); // UART6 TX,RX

// Serial pc(USBTX, NC);

// BLEHandler ble(9600, &hm10);

// // Base Motor class controlling a digital output (active low)
// class Motor {
// protected:
//   DigitalOut outputSignal;
//   bool status;

// public:
//   Motor(PinName pin) : outputSignal(pin), status(false) {
//     off(); // Start with motor off
//   }
//   void on(void) {
//     outputSignal = 0;
//     status = true;
//   }
//   void off(void) {
//     outputSignal = 1;
//     status = false;
//   }
//   void toggle(void) {
//     if (status)
//       off();
//     else
//       on();
//   }
//   bool getStatus(void) { return status; }
// };

// // Derived class using PwmOut for motor speed control
// class MotorPWM : public Motor {
// private:
//   PwmOut pwmSignal;

// public:
//   MotorPWM(PinName pin) : Motor(pin), pwmSignal(pin) {}
//   void setPWM(float period, float dutyCycle) {
//     pwmSignal.period(period);
//     pwmSignal.write(dutyCycle);
//   }
// };

// // Initialize QEI objects for both encoders
// QEI leftEncoder(PB_1, PB_15, NC, 256, QEI::X4_ENCODING);
// QEI rightEncoder(PB_14, PB_13, NC, 256, QEI::X4_ENCODING);

// // Global variables for RPM and direction
// float leftRPM;
// float rightRPM;
// const char *leftDirection;
// const char *rightDirection;

// // Calculate RPM based on pulses since last measurement (sampling period =
// // sampleTime)
// void calculateRPM(float sampleTime) {
//   float leftPulses = leftEncoder.getPulses();
//   float rightPulses = rightEncoder.getPulses();

//   // Assuming 1024 counts per revolution and sampleTime sampling interval
//   leftRPM = (leftPulses * 60.0f) / (1024.0f * sampleTime);
//   rightRPM = (rightPulses * 60.0f) / (1024.0f * sampleTime);

//   leftDirection = (leftRPM >= 0) ? "Forward" : "Reverse";
//   rightDirection = (rightRPM >= 0) ? "Forward" : "Reverse";

//   leftEncoder.reset();
//   rightEncoder.reset();
// }

// // Forward declaration for the fire button handler
// void handleFireButtonPress();

// // Application state
// typedef enum { Velocity, Sensortest, BLE } currentstate;
// currentstate state = Sensortest;

// // Simple clamp function
// float clamp(float value, float minVal, float maxVal) {
//   if (value > maxVal)
//     return maxVal;
//   if (value < minVal)
//     return minVal;
//   return value;
// }

// InterruptIn fire(D4);

// int main(void) {
//   // Define motors using PC8 and PC6
//   MotorPWM leftmotor(PC_8);
//   MotorPWM rightmotor(PC_6);

//   // Setup fire button interrupt
//   fire.rise(&handleFireButtonPress);

//   // Setup serial communication
//   hm10.baud(9600);

//   DigitalOut led(D9, 1);
//   DigitalOut Enable(PC_4, 1);
//   DigitalOut Bipolar1(PC_9, 1);
//   DigitalOut Bipolar2(PB_8, 1);

//   // Define analog sensor inputs
//   AnalogIn sensor1(PC_3);
//   AnalogIn sensor2(PC_2);
//   AnalogIn sensor3(PC_1);
//   AnalogIn sensor4(PB_0);
//   AnalogIn sensor5(PA_4);

//   while (1) {
//     switch (state) {
//     case Velocity: {
//       Enable = 1;

//       // Target and timing settings
//       float targetS = 0.8;      // Sensor reading target
//       float sampleTime = 0.02f; // Reduced sampling interval for faster response
//       float baseRPM = 200.0f;   // Base RPM when sensor error is zero
//       float feedForward = 0.05f; // Nominal offset to overcome static load

//       float Kps = 200.0f; // Sensor error scaling factor

//       // Smoothing factor (0 < smoothingFactor <= 1): lower values yield
//       // smoother changes
//       float smoothingFactor = 0.1f;

//       // Initialize smoothed target RPM values to baseRPM
//       float leftTargetRPM = baseRPM;
//       float rightTargetRPM = baseRPM;

//       // PID Gains (tune as needed)
//       float Kp = 0.00055f;
//       float Ki = 0.0005f;
//       float Kd = 0.0002f;

//       float Lintegral = 0.0f;
//       float Rintegral = 0.0f;
//       float maxIntegral = 0.3f; // Anti-windup limit

//       // Variables for derivative calculation
//       float leftPrevError = 0.0f;
//       float rightPrevError = 0.0f;

//       // Initialize motors with the feed-forward value
//       leftmotor.setPWM(0.00005f, 0.5f + feedForward);
//       rightmotor.setPWM(0.00005f, 0.5f + feedForward);
//       wait(sampleTime);         // Initial wait
//       calculateRPM(sampleTime); // Update RPM from encoder pulses
//       while (state == Velocity) {

//         wait(sampleTime); // Wait for the sampling interval

//         float s1 = sensor1.read(); // 0~1 values
//         float s2 = sensor2.read();
//         float s3 = sensor3.read();
//         float s4 = sensor4.read();
//         float s5 = sensor5.read();

//         // Weighted error calculation using all 4 sensors
//         float weight_s1 = -2.0f;
//         float weight_s2 = -1.0f;
//         float weight_s4 = 1.0f;
//         float weight_s5 = 2.0f;
//         float weightedError =
//             (s1 * weight_s1 + s2 * weight_s2 + s4 * weight_s4 +
//              s5 * weight_s5) /
//             (s1 + s2 + s4 + s5 + 1e-6); // Avoid division by zero

//         // Calculate raw target RPMs based on sensor errors
//         float rawRightTargetRPM = baseRPM + weightedError * Kps;
//         float rawLeftTargetRPM = baseRPM - weightedError * Kps;

//         // Smooth the target RPM transitions for smooth control
//         rightTargetRPM +=
//             smoothingFactor * (rawRightTargetRPM - rightTargetRPM);
//         leftTargetRPM += smoothingFactor * (rawLeftTargetRPM - leftTargetRPM);

//         // ----- Left Motor PID Control with derivative -----
//         float leftError = leftTargetRPM - leftRPM;
//         Lintegral += leftError * sampleTime;
//         Lintegral = clamp(Lintegral, -maxIntegral, maxIntegral);
//         float leftDerivative = (leftError - leftPrevError) / sampleTime;
//         float Lcontrol =
//             feedForward + Kp * leftError + Ki * Lintegral + Kd * leftDerivative;
//         float LdutyCycle = 0.5f + Lcontrol;
//         LdutyCycle = clamp(LdutyCycle, 0.0f, 1.0f);
//         leftmotor.setPWM(0.00005f, LdutyCycle);

//         pc.printf("LdutyCycle: %f\n", LdutyCycle);

//         leftPrevError = leftError; // Update previous error

//         // ----- Right Motor PID Control with derivative -----
//         float rightError = rightTargetRPM - rightRPM;
//         Rintegral += rightError * sampleTime;
//         Rintegral = clamp(Rintegral, -maxIntegral, maxIntegral);
//         float rightDerivative = (rightError - rightPrevError) / sampleTime;
//         float Rcontrol = feedForward + Kp * rightError + Ki * Rintegral +
//                          Kd * rightDerivative;
//         float RdutyCycle = 0.5f + Rcontrol;
//         RdutyCycle = clamp(RdutyCycle, 0.0f, 1.0f);

//         pc.printf("RdutyCycle: %f\n", RdutyCycle);

//         rightmotor.setPWM(0.00005f, RdutyCycle);
//         rightPrevError = rightError; // Update previous error


//       }

//       break;
//     }
//     case Sensortest:
//       lcd.cls();
//       Enable = 0;
//       while (state == Sensortest) {
//         lcd.locate(0, 0);
//         lcd.printf("Sensor1: %.3f, Sensor2: %.3f,Sensor3: %.3f, Sensor4: "
//                    "%.3f,Sensor5: %.3f",
//                    sensor1.read(), sensor2.read(), sensor3.read(),
//                    sensor4.read(), sensor5.read());
//       }
//       break;
//     case BLE:
//       lcd.cls();
//       Enable = 0;
//       lcd.locate(0, 0);
//       lcd.printf("BLE");
//       while (state == BLE) {
//         char c = ble.read_last(); // Read a single character
//         if (c == 'A') {
//           led = 1;
//         } else if (c == 'B') {
//           led = 0;
//         }
//       }
//       break;
//     default:
//       state = Sensortest;
//       break;
//     }
//   }
// }

// void handleFireButtonPress() {
//   // Cycle through states on button press
//   switch (state) {
//   case Velocity:
//     state = BLE;
//     break;
//   case BLE:
//     state = Sensortest;
//     break;
//   case Sensortest:
//     state = Velocity;
//     break;
//   default:
//     break;
//   }
// }


















// #include "BLE.cpp"
// #include "C12832.h" // Include for LCD screen
// #include "PIDController.h" // Include for PID controller
// #include "QEI.h"
// #include "mbed.h"

// C12832 lcd(D11, D13, D12, D7, D10);
// Serial hm10(PA_11, PA_12); // UART6 TX,RX
// Serial pc(USBTX, NC);

// BLEHandler ble(9600, &hm10);

// // Base Motor class controlling a digital output (active low)
// class Motor {
// protected:
//   DigitalOut outputSignal;
//   bool status;

// public:
//   Motor(PinName pin) : outputSignal(pin), status(false) {
//     off(); // Start with motor off
//   }
//   void on(void) {
//     outputSignal = 0;
//     status = true;
//   }
//   void off(void) {
//     outputSignal = 1;
//     status = false;
//   }
//   void toggle(void) {
//     if (status)
//       off();
//     else
//       on();
//   }
//   bool getStatus(void) { return status; }
// };

// // Derived class using PwmOut for motor speed control
// class MotorPWM : public Motor {
// private:
//   PwmOut pwmSignal;

// public:
//   MotorPWM(PinName pin) : Motor(pin), pwmSignal(pin) {}
//   void setPWM(float period, float dutyCycle) {
//     pwmSignal.period(period);
//     pwmSignal.write(dutyCycle);
//   }
// };

// // Initialize QEI objects for both encoders
// QEI leftEncoder(PB_1, PB_15, NC, 256, QEI::X4_ENCODING);
// QEI rightEncoder(PB_14, PB_13, NC, 256, QEI::X4_ENCODING);

// // Global variables for RPM and direction
// float leftRPM;
// float rightRPM;
// const char *leftDirection;
// const char *rightDirection;

// // Calculate RPM based on pulses since last measurement (sampling period =
// // sampleTime)
// void calculateRPM(float sampleTime) {
//   float leftPulses = leftEncoder.getPulses();
//   float rightPulses = rightEncoder.getPulses();

//   // Assuming 1024 counts per revolution and sampleTime sampling interval
//   leftRPM = (leftPulses * 60.0f) / (1024.0f * sampleTime);
//   rightRPM = (rightPulses * 60.0f) / (1024.0f * sampleTime);

//   leftDirection = (leftRPM >= 0) ? "Forward" : "Reverse";
//   rightDirection = (rightRPM >= 0) ? "Forward" : "Reverse";

//   leftEncoder.reset();
//   rightEncoder.reset();
// }

// // Forward declaration for the fire button handler
// void handleFireButtonPress();

// // Application state
// typedef enum { Velocity, Sensortest, BLE } currentstate;
// currentstate state = Sensortest;

// InterruptIn fire(D4);

// int main(void) {
//   // Define motors using PC8 and PC6
//   MotorPWM leftmotor(PC_8);
//   MotorPWM rightmotor(PC_6);

//   // Setup fire button interrupt
//   fire.rise(&handleFireButtonPress);

//   // Setup serial communication
//   hm10.baud(9600);

//   DigitalOut led(D9, 1);
//   DigitalOut Enable(PC_4, 1);
//   DigitalOut Bipolar1(PC_9, 1);
//   DigitalOut Bipolar2(PB_8, 1);

//   // Define analog sensor inputs
//   AnalogIn sensor1(PC_3);
//   AnalogIn sensor2(PC_2);
//   AnalogIn sensor3(PC_1);
//   AnalogIn sensor4(PB_0);
//   AnalogIn sensor5(PA_4);

//   while (1) {
//     switch (state) {
//     case Velocity: {
//       Enable = 1;

//       // Target and timing settings
//       float targetS = 0.8;      // Sensor reading target
//       float sampleTime = 0.02f; // Reduced sampling interval for faster response
//       float baseRPM = 200.0f;   // Base RPM when sensor error is zero
//       float feedForward = 0.05f; // Nominal offset to overcome static load

//       float Kps = 200.0f; // Sensor error scaling factor

//       // Smoothing factor (0 < smoothingFactor <= 1): lower values yield
//       // smoother changes
//       float smoothingFactor = 0.1f;

//       // Initialize smoothed target RPM values to baseRPM
//       float leftTargetRPM = baseRPM;
//       float rightTargetRPM = baseRPM;

//       // Create PID controllers for left and right motors
//       PIDController leftPID(0.00055f, 0.0005f, 0.0002f, 0.3f, feedForward, sampleTime);
//       PIDController rightPID(0.00055f, 0.0005f, 0.0002f, 0.3f, feedForward, sampleTime);

//       // Initialize motors with the feed-forward value
//       leftmotor.setPWM(0.00005f, 0.5f + feedForward);
//       rightmotor.setPWM(0.00005f, 0.5f + feedForward);
//       wait(sampleTime);         // Initial wait
//       calculateRPM(sampleTime); // Update RPM from encoder pulses
      
//       while (state == Velocity) {
//         wait(sampleTime); // Wait for the sampling interval

//         float s1 = sensor1.read(); // 0~1 values
//         float s2 = sensor2.read();
//         float s3 = sensor3.read();
//         float s4 = sensor4.read();
//         float s5 = sensor5.read();

//         // Weighted error calculation using all 4 sensors
//         float weight_s1 = -2.0f;
//         float weight_s2 = -1.0f;
//         float weight_s4 = 1.0f;
//         float weight_s5 = 2.0f;
//         float weightedError = 
//             (s1 * weight_s1 + s2 * weight_s2 + s4 * weight_s4 + s5 * weight_s5) / 
//             (s1 + s2 + s4 + s5 + 1e-6); // Avoid division by zero

//         // Calculate raw target RPMs based on sensor errors
//         float rawRightTargetRPM = baseRPM + weightedError * Kps;
//         float rawLeftTargetRPM = baseRPM - weightedError * Kps;

//         // Smooth the target RPM transitions for smooth control
//         rightTargetRPM += smoothingFactor * (rawRightTargetRPM - rightTargetRPM);
//         leftTargetRPM += smoothingFactor * (rawLeftTargetRPM - leftTargetRPM);

//         // Calculate RPM from encoder pulses
//         calculateRPM(sampleTime);

//         // ----- Left Motor PID Control -----
//         float Lcontrol = leftPID.calculate(leftTargetRPM, leftRPM);
//         float LdutyCycle = 0.5f + Lcontrol;
//         LdutyCycle = leftPID.clamp(LdutyCycle, 0.0f, 1.0f);
//         pc.printf("LdutyCycle: %f \n", LdutyCycle);

//         leftmotor.setPWM(0.00005f, LdutyCycle);

//         // ----- Right Motor PID Control -----
//         float Rcontrol = rightPID.calculate(rightTargetRPM, rightRPM);
//         float RdutyCycle = 0.5f + Rcontrol;
//         RdutyCycle = rightPID.clamp(RdutyCycle, 0.0f, 1.0f);
//         pc.printf("RdutyCycle: %f \n", RdutyCycle);
        
//         rightmotor.setPWM(0.00005f, RdutyCycle);
//       }

//       break;
//     }
//     case Sensortest:
//       lcd.cls();
//       Enable = 0;
//       while (state == Sensortest) {
//         lcd.locate(0, 0);
//         lcd.printf("Sensor1: %.3f, Sensor2: %.3f,Sensor3: %.3f, Sensor4: "
//                    "%.3f,Sensor5: %.3f",
//                    sensor1.read(), sensor2.read(), sensor3.read(),
//                    sensor4.read(), sensor5.read());
//       }
//       break;
//     case BLE:
//       lcd.cls();
//       Enable = 0;
//       lcd.locate(0, 0);
//       lcd.printf("BLE");
//       while (state == BLE) {
//         char c = ble.read_last(); // Read a single character
//         if (c == 'A') {
//           led = 1;
//         } else if (c == 'B') {
//           led = 0;
//         }
//       }
//       break;
//     default:
//       state = Sensortest;
//       break;
//     }
//   }
// }

// void handleFireButtonPress() {
//   // Cycle through states on button press
//   switch (state) {
//   case Velocity:
//     state = BLE;
//     break;
//   case BLE:
//     state = Sensortest;
//     break;
//   case Sensortest:
//     state = Velocity;
//     break;
//   default:
//     break;
//   }
// }






















#include "BLE.cpp"
#include "C12832.h" // Include for LCD screen
#include "PIDController.h" // Include for PID controller
#include "QEI.h"
#include "mbed.h"

C12832 lcd(D11, D13, D12, D7, D10);
Serial hm10(PA_11, PA_12); // UART6 TX,RX
Serial pc(USBTX, NC);

BLEHandler ble(9600, &hm10);

// Base Motor class controlling a digital output (active low)
class Motor {
protected:
  DigitalOut outputSignal;
  bool status;

public:
  Motor(PinName pin) : outputSignal(pin), status(false) {
    off(); // Start with motor off
  }
  void on(void) {
    outputSignal = 0;
    status = true;
  }
  void off(void) {
    outputSignal = 1;
    status = false;
  }
  void toggle(void) {
    if (status)
      off();
    else
      on();
  }
  bool getStatus(void) { return status; }
};

// Derived class using PwmOut for motor speed control
class MotorPWM : public Motor {
private:
  PwmOut pwmSignal;

public:
  MotorPWM(PinName pin) : Motor(pin), pwmSignal(pin) {}
  void setPWM(float period, float dutyCycle) {
    pwmSignal.period(period);
    pwmSignal.write(dutyCycle);
  }
};

// Initialize QEI objects for both encoders
QEI leftEncoder(PB_1, PB_15, NC, 256, QEI::X4_ENCODING);
QEI rightEncoder(PB_14, PB_13, NC, 256, QEI::X4_ENCODING);

// Global variables for RPM and direction
float leftRPM;
float rightRPM;
const char *leftDirection;
const char *rightDirection;

// Calculate RPM based on pulses since last measurement (sampling period =
// sampleTime)
void calculateRPM(float sampleTime) {
  float leftPulses = leftEncoder.getPulses();
  float rightPulses = rightEncoder.getPulses();

  // Assuming 1024 counts per revolution and sampleTime sampling interval
  leftRPM = (leftPulses * 60.0f) / (1024.0f * sampleTime);
  rightRPM = (rightPulses * 60.0f) / (1024.0f * sampleTime);

  leftDirection = (leftRPM >= 0) ? "Forward" : "Reverse";
  rightDirection = (rightRPM >= 0) ? "Forward" : "Reverse";

  leftEncoder.reset();
  rightEncoder.reset();
}

// Forward declaration for the fire button handler
void handleFireButtonPress();

// Application state
typedef enum { Velocity, Sensortest, BLE } currentstate;
currentstate state = Sensortest;

InterruptIn fire(D4);

int main(void) {
  // Define motors using PC8 and PC6
  MotorPWM leftmotor(PC_8);
  MotorPWM rightmotor(PC_6);

  // Setup fire button interrupt
  fire.rise(&handleFireButtonPress);

  // Setup serial communication
  hm10.baud(9600);

  DigitalOut led(D9, 1);
  DigitalOut Enable(PC_4, 1);
  DigitalOut Bipolar1(PC_9, 1);
  DigitalOut Bipolar2(PB_8, 1);

  // Define analog sensor inputs
  AnalogIn sensor1(PC_3);
  AnalogIn sensor2(PC_2);
  AnalogIn sensor3(PC_1);
  AnalogIn sensor4(PB_0);
  AnalogIn sensor5(PA_4);

  while (1) {
    switch (state) {
    case Velocity: {
      Enable = 1;

      // Target and timing settings
      float targetVelocity = 200.0f; // Target velocity in RPM
      float sampleTime = 0.02f;      // Reduced sampling interval for faster response
      float baseRPM = 0.0f;        // Base RPM when sensor error is zero
      float feedForward = 0.05f;     // Nominal offset to overcome static load
      float speedFactor = 1.0f;      // Speed factor (1.0 = 100% of base speed)

      float Kps = 200.0f; // Sensor error scaling factor

      // Smoothing factor (0 < smoothingFactor <= 1): lower values yield
      // smoother changes
      float smoothingFactor = 0.1f;

      // Initialize smoothed target RPM values to baseRPM
      float leftTargetRPM = baseRPM;
      float rightTargetRPM = baseRPM;

      // Create PID controllers for left and right motors (steering control)
      PIDController leftPID(0.00055f, 0.0005f, 0.0002f, 0.3f, feedForward, sampleTime);
      PIDController rightPID(0.00055f, 0.0005f, 0.0002f, 0.3f, feedForward, sampleTime);
      
      // Create velocity PID controller for speed control
      PIDController velocityPID(0.0008f, 0.0008f, 0.0001f, 0.3f, 0.0f, sampleTime);

      // Initialize motors with the feed-forward value
      leftmotor.setPWM(0.00005f, 0.5f + feedForward);
      rightmotor.setPWM(0.00005f, 0.5f + feedForward);
      wait(sampleTime);         // Initial wait
      calculateRPM(sampleTime); // Update RPM from encoder pulses
      
      while (state == Velocity) {
        wait(sampleTime); // Wait for the sampling interval

        float s1 = sensor1.read(); // 0~1 values
        float s2 = sensor2.read();
        float s3 = sensor3.read();
        float s4 = sensor4.read();
        float s5 = sensor5.read();

        // Weighted error calculation using all 4 sensors for steering
        float weight_s1 = -2.0f;
        float weight_s2 = -1.0f;
        float weight_s4 = 1.0f;
        float weight_s5 = 2.0f;
        float weightedError = 
            (s1 * weight_s1 + s2 * weight_s2 + s4 * weight_s4 + s5 * weight_s5) / 
            (s1 + s2 + s4 + s5 + 1e-6); // Avoid division by zero

        // Calculate RPM from encoder pulses
        calculateRPM(sampleTime);
        
        // Calculate current average velocity
        float currentAverageRPM = (leftRPM + rightRPM) / 2.0f;
        
        // Velocity control loop - adjust speed factor
        float speedAdjustment = velocityPID.calculate(targetVelocity, currentAverageRPM);
        speedFactor += speedAdjustment * 0.01f; // Small incremental adjustment
        speedFactor = velocityPID.clamp(speedFactor, 0.5f, 1.5f); // Limit to 50%-150% range
        
        // Apply speed factor to base RPM
        float effectiveBaseRPM = baseRPM * speedFactor;
        
        // Calculate raw target RPMs based on sensor errors and effective base RPM
        float rawRightTargetRPM = effectiveBaseRPM + weightedError * Kps;
        float rawLeftTargetRPM = effectiveBaseRPM - weightedError * Kps;

        // Smooth the target RPM transitions for smooth control
        rightTargetRPM += smoothingFactor * (rawRightTargetRPM - rightTargetRPM);
        leftTargetRPM += smoothingFactor * (rawLeftTargetRPM - leftTargetRPM);

        // ----- Left Motor PID Control -----
        float Lcontrol = leftPID.calculate(leftTargetRPM, leftRPM);
        float LdutyCycle = 0.5f + Lcontrol;
        LdutyCycle = leftPID.clamp(LdutyCycle, 0.0f, 1.0f);
        pc.printf("LdutyCycle: %f \n", LdutyCycle);
        leftmotor.setPWM(0.00005f, LdutyCycle);

        // ----- Right Motor PID Control -----
        float Rcontrol = rightPID.calculate(rightTargetRPM, rightRPM);
        float RdutyCycle = 0.5f + Rcontrol;
        RdutyCycle = rightPID.clamp(RdutyCycle, 0.0f, 1.0f);
        pc.printf("RdutyCycle: %f \n", RdutyCycle);
        rightmotor.setPWM(0.00005f, RdutyCycle);
        
        // Debug output for velocity control
        pc.printf("Target: %.2f, Current: %.2f, SpeedFactor: %.2f\n\n", 
                 targetVelocity, currentAverageRPM, speedFactor);
      }

      break;
    }
    case Sensortest:
      lcd.cls();
      Enable = 0;
      while (state == Sensortest) {
        lcd.locate(0, 0);
        lcd.printf("Sensor1: %.3f, Sensor2: %.3f,Sensor3: %.3f, Sensor4: "
                   "%.3f,Sensor5: %.3f",
                   sensor1.read(), sensor2.read(), sensor3.read(),
                   sensor4.read(), sensor5.read());
      }
      break;
    case BLE:
      lcd.cls();
      Enable = 0;
      lcd.locate(0, 0);
      lcd.printf("BLE");
      while (state == BLE) {
        char c = ble.read_last(); // Read a single character
        if (c == 'A') {
          led = 1;
        } else if (c == 'B') {
          led = 0;
        }
      }
      break;
    default:
      state = Sensortest;
      break;
    }
  }
}

void handleFireButtonPress() {
  // Cycle through states on button press
  switch (state) {
  case Velocity:
    state = BLE;
    break;
  case BLE:
    state = Sensortest;
    break;
  case Sensortest:
    state = Velocity;
    break;
  default:
    break;
  }
}


