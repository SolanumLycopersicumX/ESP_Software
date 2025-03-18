#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    // PID gains
    float Kp;
    float Ki;
    float Kd;
    
    // State variables
    float integral;
    float prevError;
    float maxIntegral;  // For anti-windup
    float feedForward;  // Feed-forward term
    float sampleTime;   // Sampling interval
    
public:
    // Constructor with default values
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, 
                 float maxInt = 0.3f, float ff = 0.05f, float sTime = 0.02f);
    
    // Reset the controller state
    void reset();
    
    // Set PID gains
    void setGains(float kp, float ki, float kd);
    
    // Set anti-windup limit
    void setMaxIntegral(float max);
    
    // Set feed-forward term
    void setFeedForward(float ff);
    
    // Set sample time
    void setSampleTime(float sTime);
    
    // Calculate control output based on error
    float calculate(float setpoint, float measurement);
    
    // Utility function to clamp values
    float clamp(float value, float minVal, float maxVal);
    
    // Getters for internal state (useful for debugging)
    float getIntegral() const;
    float getPrevError() const;
};

#endif // PID_CONTROLLER_H
