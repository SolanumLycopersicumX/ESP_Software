 #include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, 
                           float maxInt, float ff, float sTime)
    : Kp(kp), Ki(ki), Kd(kd), 
      integral(0.0f), prevError(0.0f), 
      maxIntegral(maxInt), feedForward(ff), sampleTime(sTime) {
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}

void PIDController::setGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::setMaxIntegral(float max) {
    maxIntegral = max;
}

void PIDController::setFeedForward(float ff) {
    feedForward = ff;
}

void PIDController::setSampleTime(float sTime) {
    sampleTime = sTime;
}

float PIDController::calculate(float setpoint, float measurement) {
    // Calculate error
    float error = setpoint - measurement;
    
    // Calculate integral term with anti-windup
    integral += error * sampleTime;
    integral = clamp(integral, -maxIntegral, maxIntegral);
    
    // Calculate derivative term
    float derivative = (error - prevError) / sampleTime;
    
    // Calculate control output
    float output = feedForward + Kp * error + Ki * integral + Kd * derivative;
    
    // Store error for next iteration
    prevError = error;
    
    return output;
}

float PIDController::clamp(float value, float minVal, float maxVal) {
    if (value > maxVal)
        return maxVal;
    if (value < minVal)
        return minVal;
    return value;
}

float PIDController::getIntegral() const {
    return integral;
}

float PIDController::getPrevError() const {
    return prevError;
}
