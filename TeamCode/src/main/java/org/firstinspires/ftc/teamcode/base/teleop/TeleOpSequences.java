package org.firstinspires.ftc.teamcode.base.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public void pickUpSample(boolean sequenceRunning){
        ElapsedTime timer = new ElapsedTime();

    }

    public void moveToPositionMotor(boolean gamePadInput, int position){
        if (gamePadInput) {
            state.setMotorTarget(motorEnum, position);
        }
    }

    public void moveToPositionServo(boolean gamePadInput, double position){
        if (gamePadInput) {
            state.setServoPosition(servoEnum, position);
        }
    }

    public void resetMotorEncoder(boolean gamePadInput){
        DcMotor.RunMode currentRunmode = hw.getMotorConfig(motorEnum).runMode;

        if (gamePadInput){
            hw.getMotorConfig(motorEnum).motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hw.getMotorConfig(motorEnum).motor.setMode(currentRunmode);
        }
    }

    public void dynamicMovement(int tickIncrease , boolean gamePadInput, boolean gamePadInput2){
        if (gamePadInput && state.getMotorTarget(motorEnum) + tickIncrease <= hw.getMotorConfig(motorEnum).maxTarget){
            state.setMotorTarget(motorEnum, state.getMotorTarget(motorEnum) + tickIncrease);
        }
        else if (gamePadInput2 && state.getMotorTarget(motorEnum) - tickIncrease >= hw.getMotorConfig(motorEnum).minTarget){
            state.setMotorTarget(motorEnum, state.getMotorTarget(motorEnum) - tickIncrease);
        }
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
        if (state.getMotorTarget(motorEnum) == positions[currentPositionIndex]){
            if (gamePadInput){
                // If index is highest index, set lowest index
                if (currentPositionIndex == (positions.length - 1)){
                    state.setMotorTarget(motorEnum, positions[0]);
                }
                else{
                    state.setMotorTarget(motorEnum, positions[currentPositionIndex + 1]);
                }
            }
        }
        else{
            if (gamePadInput){
                state.setMotorTarget(motorEnum, positions[closestPositionIndex]);
            }
        }
    }

    public void finiteStateMotor(boolean gamePadInput, boolean gamePadInput2, int[] positions){
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
        if (state.getMotorTarget(motorEnum) == positions[currentPositionIndex]){
            if (gamePadInput){
                // If highest index, go to lowest index
                if (currentPositionIndex == (positions.length - 1)){
                    state.setMotorTarget(motorEnum, positions[0]);
                }
                else{
                    state.setMotorTarget(motorEnum, positions[currentPositionIndex + 1]);
                }
            }
            else if (gamePadInput2){
                // If lowest index, go highest index
                if (currentPositionIndex == (0)){
                    state.setMotorTarget(motorEnum, positions[positions.length - 1]);
                }
                else{
                    state.setMotorTarget(motorEnum, positions[currentPositionIndex - 1]);
                }
            }
        }
        else{
            if (gamePadInput){
                state.setMotorTarget(motorEnum, positions[closestPositionIndex]);
            }
        }
    }
}