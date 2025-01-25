package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
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
    boolean isFirstIteration = true;
    public PID(){
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
    }
    public double getPIDOutput(MotorEnum motorEnum, double reference) {
        if (isFirstIteration){
            timer.reset();
            isFirstIteration = false;
        }

        encoderPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();

        error = state.getMotorTarget(motorEnum) - encoderPosition;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        if (reference != lastReference){
            integralSum = 0;
        }

        proportionalPower = error * 0.015;

        integralPower = integralSum * hw.getMotorConfig(motorEnum).kI;

        derivativePower = derivative * 0.0002;

        outPower = proportionalPower + integralPower + derivativePower;

        lastReference = reference;
        lastError = error;
        timer.reset();

        return outPower;
    }
}
