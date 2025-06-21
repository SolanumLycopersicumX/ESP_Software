#include "BLE.cpp"
////////////////////////////////////////////////////////////////////
//                          _ooOoo_                               //
//                         o8888888o                              //
//                         88" . "88                              //
//                         (| ^_^ |)                              //
//                         O\  =  /O                              //
//                      ____/`---'\____                           //
//                    .'  \\|     |//  `.                         //
//                   /  \\|||  :  |||//  \                        //
//                  /  _||||| -:- |||||-  \                       //
//                  |   | \\\  -  /// |   |                       //
//                  | \_|  ''\---/''  |   |                       //
//                  \  .-\__  `-`  ___/-. /                       //
//                ___`. .'  /--.--\  `. . ___                     //
//              ."" '<  `.___\_<|>_/___.'  >'"".                  //
//            | | :  `- \`.;`\ _ /`;.`/ - ` : | |                 //
//            \  \ `-.   \_ __\ /__ _/   .-` /  /                 //
//      ========`-.____`-.___\_____/___.-`____.-'========         //
//                           `=---='                              //
//      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^        //
//         佛祖保佑         永无BUG        永不修改                 //
////////////////////////////////////////////////////////////////////

#include "C12832.h" // Include for LCD screen
#include "QEI.h"
#include "mbed.h"

C12832 lcd(D11, D13, D12, D7, D10);
Serial hm10(PA_11, PA_12); // UART6 TX,RX

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
  MotorPWM(PinName pin) : pwmSignal(pin), Motor(pin) {}
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
currentstate state = Velocity;

// Simple clamp function
float clamp(float value, float minVal, float maxVal) {
  if (value > maxVal)
    return maxVal;
  if (value < minVal)
    return minVal;
  return value;
}

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

      float sampleTime = 0.02f; // Reduced sampling interval for faster response
      float feedForward = 0.05f; // Nominal offset to overcome static load\


      //   float baseRPM = 1150.0f;
      //   float Kp = 0.00023f;
      //   float Ki = 0.00010f;
      //   float Kd = 0.000005f;
      //   float Kps = 475.0f;
      //   float Kis = 10.0f;
      //   float Kds = 0.0f;

      //   float baseRPM = 1150.0f;
      //   float Kp = 0.00023f;
      //   float Ki = 0.00010f;
      //   float Kd = 0.000005f;
      //   float Kps = 500.0f;
      //   float Kis = 10.0f;
      //   float Kds = 1.0f;

      //   float baseRPM = 1300.0f;
      //   float Kp = 0.00023f;
      //   float Ki = 0.00012f;
      //   float Kd = 0.000004f;
      //   float Kps = 565.0f;
      //   float Kis = 11.75f;
      //   float Kds = 3.042f;

      float baseRPM = 950.0f;
      float Kp = 0.00023f;
      float Ki = 0.00012f;
      float Kd = 0.000004f;
      float Kps = 570.0f;
      float Kis = 11.75f;
      float Kds = 3.042f;

      float Lintegral = 0.0f;
      float Rintegral = 0.0f;
      float maxIntegral = 0.3f; // Anti-windup limit
      float sensor_integral = 0.0f;
      float sensor_prev_error = 0.0f;
      float maxSensorIntegral = 30.0f;
      // Variables for derivative calculation
      float leftPrevError = 0.0f;
      float rightPrevError = 0.0f;

      // Initialize motors with the feed-forward value
      leftmotor.setPWM(0.00005f, 0.5f + feedForward);
      rightmotor.setPWM(0.00005f, 0.5f + feedForward);
      wait((double)sampleTime); // Initial wait
      calculateRPM(sampleTime); // Update RPM from encoder pulses
      while (state == Velocity) {

        wait((double)sampleTime);  // Wait for the sampling interval
        calculateRPM(sampleTime);  // Update RPM from encoder pulses
        float s1 = sensor1.read(); // 0~1 values
        float s2 = sensor2.read();
        float s3 = sensor3.read();
        float s4 = sensor4.read();
        float s5 = sensor5.read();

        // Weighted error calculation using all 4 sensors
        float weight_s1 = -3.2f;
        float weight_s2 = -2.0f;
        float weight_s4 = 2.0f;
        float weight_s5 = 3.5f;
        float weightedError =
            (s1 * weight_s1 + s2 * weight_s2 + s4 * weight_s4 +
             s5 * weight_s5) /
            (s1 + s2 + s4 + s5 + 1e-6); // Avoid division by zero
        float sensor_error = weightedError;
        sensor_integral += sensor_error * sampleTime;
        sensor_integral =
            clamp(sensor_integral, -maxSensorIntegral, maxSensorIntegral);
        float sensor_derivative =
            (sensor_error - sensor_prev_error) / sampleTime;
        sensor_prev_error = sensor_error;
        float sensorControl = Kps * sensor_error + Kis * sensor_integral +
                              Kds * sensor_derivative;
        // Calculate raw target RPMs based on sensor errors
        float rightTargetRPM = baseRPM + sensorControl;
        float leftTargetRPM = baseRPM - sensorControl;

        // ----- Left Motor PID Control with derivative -----
        float leftError = leftTargetRPM - leftRPM;
        Lintegral += leftError * sampleTime;
        Lintegral = clamp(Lintegral, -maxIntegral, maxIntegral);
        float leftDerivative = (leftError - leftPrevError) / sampleTime;
        float Lcontrol =
            feedForward + Kp * leftError + Ki * Lintegral + Kd * leftDerivative;
        float LdutyCycle = 0.5f + Lcontrol;
        LdutyCycle = clamp(LdutyCycle, 0.0f, 1.0f);
        leftmotor.setPWM(0.00005f, LdutyCycle);
        leftPrevError = leftError; // Update previous error

        // ----- Right Motor PID Control with derivative -----
        float rightError = rightTargetRPM - rightRPM;
        Rintegral += rightError * sampleTime;
        Rintegral = clamp(Rintegral, -maxIntegral, maxIntegral);
        float rightDerivative = (rightError - rightPrevError) / sampleTime;
        float Rcontrol = feedForward + Kp * rightError + Ki * Rintegral +
                         Kd * rightDerivative;
        float RdutyCycle = 0.5f + Rcontrol;
        RdutyCycle = clamp(RdutyCycle, 0.0f, 1.0f);
        rightmotor.setPWM(0.00005f, RdutyCycle);
        rightPrevError = rightError; // Update previous error

        float val[2] = {leftRPM, rightRPM};
        ble.write_float_array(val, 2, 5);
        char ASDA = ble.read_char();
        if (ASDA == 'C') {
          ble.read_char_set();
          state = Sensortest;
          led = 1;
          break;
        }

       
        if (ASDA == 'A') {
          ble.read_char_set();
          baseRPM = 600.0;
        }
        if (ASDA == 'B') {
          ble.read_char_set();
          baseRPM = 950.0;
        }
        if (ASDA == 'D') {
          ble.read_char_set();
          baseRPM = 1300.0;
        }
        if (s1 <= 0.4 && s2 <= 0.4 && s3 <= 0.4 && s4 <= 0.4 && s5 <= 0.4) {
          leftmotor.setPWM(0.00005f, 0.5);
          rightmotor.setPWM(0.00005f, 0.5);
        }
      }

      break;
    }
    case Sensortest:
      leftmotor.setPWM(0.00005f, 0.65);
      rightmotor.setPWM(0.00005f, 0.35);
      wait(0.35);

      while (state == Sensortest) {

        float s2 = sensor2.read();
        if (s2 >= 0.4) {
          leftmotor.setPWM(0.00005f, 0.5);
          rightmotor.setPWM(0.00005f, 0.5);
          wait(0.2);
          state = Velocity;
          break;
        }
      }
      break;
    case BLE:
      lcd.cls();
      Enable = 0;
      lcd.locate(0, 0);
      lcd.printf("BLE");
      while (state == BLE) {
        char c = ble.read_char(); // Read a single character
        if (c == 'A') {
          led = 1;
        } else if (c == 'B') {
          led = 0;
        }
      }
      break;
    default:
      state = Velocity;
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
