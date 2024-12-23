package org.firstinspires.ftc.teamcode.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.RobotState;

public class PID {
    HardwareConfig hw;
    RobotState state;
    int lastError = 0;
    int lastReference = 0;
    double integralSum = 0;
    double integralSumLimit = 0.25;
    ElapsedTime timer = new ElapsedTime();
    public PID(){
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
    }
    public double getPIDOutput(MotorEnum motorEnum, int reference){

        int encoderPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();

        int error = reference - encoderPosition;

        double derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        if (reference != lastReference){
            integralSum = 0;
        }
        if (integralSum > integralSumLimit){
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit){
            integralSum = -integralSumLimit;
        }
        double proportionalPower = error * hw.getMotorConfig(motorEnum).kP;
        double integralPower = integralSum * hw.getMotorConfig(motorEnum).kI;
        double derivativePower = derivative * hw.getMotorConfig(motorEnum).kD;
        double outPower = proportionalPower + integralPower + derivativePower;

        lastError = error;
        lastReference = reference;
        timer.reset();

        return outPower;
    }
}
