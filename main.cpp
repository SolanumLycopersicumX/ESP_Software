Enable = 1;

      // Target and timing settings
      float targetS = 0.8;       // Sensor reading target
      float sampleTime = 0.05f;  // Sampling interval
      float feedForward = 0.05f; // Nominal offset to overcome static load

      float Kps = 200.0f; // Sensor error scaling factor

      // Smoothing factor (0 < smoothingFactor <= 1): lower values yield
      // smoother changes
      float smoothingFactor = 0.1f;

      // PID Gains (tune as needed)
      float Kp = 0.0006f;
      float Ki = 0.0001f;
      float Kd = 0.0003f;

      float Lintegral = 0.0f;
      float Rintegral = 0.0f;
      float maxIntegral = 0.3f; // Anti-windup limit

      // Variables for derivative calculation
      float leftPrevError = 0.0f;
      float rightPrevError = 0.0f;

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

        // Compute weighted sensor error for line following
        float weightedError = (s1 * -2.0f) + (s2 * -1.0f) + (s3 * 0.0f) + (s4 * 1.0f) + (s5 * 2.0f);

        // Calculate target RPM adjustments based on sensor errors
        float leftTargetRPM = -weightedError * Kps;
        float rightTargetRPM = weightedError * Kps;

        // Smooth the target RPM transitions for smooth control
        rightTargetRPM += smoothingFactor * (rightTargetRPM - rightRPM);
        leftTargetRPM += smoothingFactor * (leftTargetRPM - leftRPM);

        // ----- Left Motor PID Control with derivative -----
        float leftError = leftTargetRPM - leftRPM;
        Lintegral += leftError * sampleTime;
        Lintegral = clamp(Lintegral, -maxIntegral, maxIntegral);
        float leftDerivative = (leftError - leftPrevError) / sampleTime;
        float Lcontrol = feedForward + Kp * leftError + Ki * Lintegral + Kd * leftDerivative;
        float LdutyCycle = 0.5f + Lcontrol;
        LdutyCycle = clamp(LdutyCycle, 0.5f, 1.0f);
        leftmotor.setPWM(0.00005f, LdutyCycle);
        leftPrevError = leftError; // Update previous error

        // ----- Right Motor PID Control with derivative -----
        float rightError = rightTargetRPM - rightRPM;
        Rintegral += rightError * sampleTime;
        Rintegral = clamp(Rintegral, -maxIntegral, maxIntegral);
        float rightDerivative = (rightError - rightPrevError) / sampleTime;
        float Rcontrol = feedForward + Kp * rightError + Ki * Rintegral + Kd * rightDerivative;
        float RdutyCycle = 0.5f + Rcontrol;
        RdutyCycle = clamp(RdutyCycle, 0.5f, 1.0f);
        rightmotor.setPWM(0.00005f, RdutyCycle);
        rightPrevError = rightError; // Update previous error
      }