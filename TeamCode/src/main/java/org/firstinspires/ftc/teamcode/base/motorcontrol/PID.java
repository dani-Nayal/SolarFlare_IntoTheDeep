package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

public class PID {
    HardwareConfig hw;
    RobotState state;
    public double error;
    public int encoderPosition;
    public double derivative;
    public double proportionalPower;
    public double integralPower;
    public double derivativePower;
    public double outPower;
    double lastError = 0;
    double lastReference = 0;
    double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();
    public PID(){
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
    }
    public double getPIDOutput(MotorEnum motorEnum, double reference) {
        MotorConfig motorConfig = hw.getMotorConfig(motorEnum);
        encoderPosition = motorConfig.motor.getCurrentPosition();

        error = reference - encoderPosition;

        derivative = motorConfig.motor.getVelocity();

        integralSum = integralSum + (error * timer.seconds());

        if (reference != lastReference){
            integralSum = 0;
        }
        
        proportionalPower = error * motorConfig.kP;
        integralPower = integralSum * motorConfig.kI;
        derivativePower = derivative * motorConfig.kD;
        outPower = Math.max(-1, Math.min(1, proportionalPower + integralPower + derivativePower));

        lastError = error;
        lastReference = reference;
        timer.reset();

        return outPower;
    }
}
