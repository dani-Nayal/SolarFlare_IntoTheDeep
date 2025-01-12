package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

public class PID {
    HardwareConfig hw;
    RobotState state;
    double lastError = 0;
    double lastReference = 0;
    double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();
    public PID(){
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
    }
    public double getPIDOutput(MotorEnum motorEnum, double reference){

        int encoderPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();

        double error = reference - encoderPosition;

        double derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        if (reference != lastReference){
            integralSum = 0;
        }
        
        double proportionalPower = error * hw.getMotorConfig(motorEnum).kP;
        double integralPower = integralSum * hw.getMotorConfig(motorEnum).kI;
        double derivativePower = derivative * hw.getMotorConfig(motorEnum).kD;
        double outPower = Math.max(-1,Math.min(1,proportionalPower + integralPower + derivativePower));

        lastError = error;
        lastReference = reference;
        timer.reset();

        return outPower;
    }
}
