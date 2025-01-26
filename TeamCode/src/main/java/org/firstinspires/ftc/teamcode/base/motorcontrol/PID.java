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
    public double lastError = 0;
    public double lastReference = 0;
    public double integralSum = 0;
    public double loopTime;
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

        proportionalPower = error * hw.getMotorConfig(motorEnum).kP;

        integralPower = integralSum * hw.getMotorConfig(motorEnum).kI;

        derivativePower = derivative * hw.getMotorConfig(motorEnum).kD;

        outPower = proportionalPower + integralPower + derivativePower;

        lastReference = reference;
        lastError = error;
        loopTime = timer.seconds();
        timer.reset();
        return outPower;
    }
}
