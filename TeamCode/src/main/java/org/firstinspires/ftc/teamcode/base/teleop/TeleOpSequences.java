package org.firstinspires.ftc.teamcode.base.teleop;
/*
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.config.ServoEnum;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;

import java.util.List;

public class TeleOpSequences {
    HardwareConfig hw;
    RobotState state;
    MotorEnum motorEnum;
    ServoEnum servoEnum;
    MotorControl motorControl;
    int currentPositionIndex;
    double targetDifference;
    double leastTargetDifference = Double.POSITIVE_INFINITY;
    int closestPositionIndex;

    public TeleOpSequences(MotorEnum motorEnum){
        this.motorEnum = motorEnum;
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
        motorControl = new MotorControl(motorEnum);
    }
    public TeleOpSequences(ServoEnum servoEnum){
        this.servoEnum = servoEnum;
    }

    public void moveToPositionMotor(int position){
        state.setMotorTarget(motorEnum, position);
    }

    public void moveToPositionServo(double position){
        state.setServoPosition(servoEnum, position);
    }

    public void finiteStateMotor(boolean gamePadInput, int[] positions){
        for (int i = 0; i < positions.length; i++ ){
            if (state.getMotorTarget(motorEnum) == positions[i]) {
                currentPositionIndex = positions[i];
            }

            else {

                targetDifference = state.getMotorTarget(motorEnum) - positions[i];
                if (targetDifference < leastTargetDifference){
                    leastTargetDifference = targetDifference;
                    closestPositionIndex = i;
                }
            }
        }

        if (gamePadInput){
            state.setMotorTarget(motorEnum, positions[currentPositionIndex + 1]);
            currentPositionIndex += 1;
        }
    }
}
*/