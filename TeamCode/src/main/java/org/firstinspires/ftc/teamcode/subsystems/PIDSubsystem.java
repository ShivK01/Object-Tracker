package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

//PID class
public class PIDSubsystem {
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    public double KIntegral, KDerivative , KProportional;

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        return (error * KProportional) + (derivative * KDerivative) + (integralSum * KIntegral);
    }
}
