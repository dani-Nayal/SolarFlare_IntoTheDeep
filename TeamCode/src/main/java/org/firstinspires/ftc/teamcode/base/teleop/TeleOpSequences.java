package org.firstinspires.ftc.teamcode.base.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.MotorEnum;
import org.firstinspires.ftc.teamcode.base.RobotState;
import org.firstinspires.ftc.teamcode.base.ServoEnum;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;

import java.util.List;

public class TeleOpSequences {
    HardwareConfig hw;
    RobotState state;
    MotorEnum motorEnum;
    ServoEnum servoEnum;
    MotorControl motorControl;
    int currentPositionIndex;
    double shortestTargetDifference;
    double leastShortestTargetDifference;
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

            shortestTargetDifference = state.getMotorTarget(motorEnum) - positions[i];
            if (shortestTargetDifference < leastShortestTargetDifference){
                leastShortestTargetDifference = shortestTargetDifference;
                closestPositionIndex = i;
            }

            else{
                if (gamePadInput){
                    state.setMotorTarget(motorEnum, positions[closestPositionIndex]);
                }
            }
        }
        if (gamePadInput){
            state.setMotorTarget(motorEnum, positions[currentPositionIndex + 1]);
            currentPositionIndex += 1;
        }
    }

    public void finiteStateMotor(boolean gamePadInput1, boolean gamePadInput2, int[] positions){
            for (int i = 0; i < positions.length; i++ ){
                if (state.getMotorTarget(motorEnum) == positions[i]) {
                    currentPositionIndex = positions[i];
                }

                shortestTargetDifference = state.getMotorTarget(motorEnum) - positions[i];
                if (shortestTargetDifference < leastShortestTargetDifference){
                    leastShortestTargetDifference = shortestTargetDifference;
                    closestPositionIndex = i;
                }

                if (state.getMotorTarget(motorEnum) != positions[i]){
                    if (gamePadInput1 && ){
                        state.setMotorTarget(motorEnum, positions[closestPositionIndex]);
                    }
                }
            }
            if (gamePadInput2){
                state.setMotorTarget(motorEnum, positions[currentPositionIndex + 1]);
                currentPositionIndex += 1;
            }
    }
}
