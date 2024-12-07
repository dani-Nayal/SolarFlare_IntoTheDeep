package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.ServoEnum;
import org.firstinspires.ftc.teamcode.RobotState;

public class TeleOpSequences {
    HardwareConfig hw;
    RobotState state;
    Gamepad gamepad1;
    Gamepad gamepad2;
    boolean isSubmersibleSampleIntakeActive;
    ElapsedTime submersibleSampleIntakeTimer;
    boolean isBucketRetractionActive;
    ElapsedTime bucketRetractionTimer;
    boolean isTransferSampleActive;
    ElapsedTime transferSampleTimer;
    public TeleOpSequences(Gamepad gamepad1, Gamepad gamepad2){
        hw = HardwareConfig.getHardwareConfig();
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        submersibleSampleIntakeTimer = new ElapsedTime();
        bucketRetractionTimer = new ElapsedTime();
    }
    public void SubmersibleSampleIntake(){
        if (gamepad1.b){
            isSubmersibleSampleIntakeActive=true;
            submersibleSampleIntakeTimer.reset();
        }
        if (isSubmersibleSampleIntakeActive){
            state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 1350);
            state.setServoPosition(ServoEnum.CLAW_WRIST, 76.5);
            if (submersibleSampleIntakeTimer.seconds() > 0.7) {
                state.setMotorTarget(MotorEnum.EXTENDO, 503);
                state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,104);
                state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT,104);
                isSubmersibleSampleIntakeActive = false;
            }
        }
    }
    public void bucketRetraction(){
        if (gamepad1.x){
            isBucketRetractionActive = true;
            bucketRetractionTimer.reset();
        }
        if (isBucketRetractionActive) {
            state.setServoPosition(ServoEnum.BUCKET, 85);

            if (bucketRetractionTimer.seconds()>0.3){
                state.setMotorTarget(MotorEnum.BUCKET_SLIDES, 0);
                isBucketRetractionActive = false;
            }
        }
    }
    public void transferSample(){
        if (gamepad1.a && !isSubmersibleSampleIntakeActive){
            isTransferSampleActive=true;
            transferSampleTimer.reset();
        }
        if (isTransferSampleActive) {
            if (state.getServoPosition(ServoEnum.CLAW_WRIST) != 76.5){
                state.setServoPosition(ServoEnum.CLAW_WRIST,76.5);
                state.setServoPosition(ServoEnum.BUCKET,85);
                if (transferSampleTimer.seconds() > 0.3){
                    state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,104);
                    state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT,104);
                }
                if (transferSampleTimer.seconds() > 0.5) {
                    state.setMotorTarget(MotorEnum.EXTENDO,0);
                }
                if (transferSampleTimer.seconds() > 0.8) {
                    if (!(hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition()>150)) {
                        state.setMotorTarget(MotorEnum.EXTENDO_PITCH,0);
                    }
                }

                if (hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.getCurrentPosition()<100) {
                    state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,200);
                    state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT,200);
                    isTransferSampleActive=false;
                }
            }
            else{
                state.setServoPosition(ServoEnum.BUCKET,85 );
                state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,104);
                state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT,104);

                if (transferSampleTimer.seconds() > 0.2){
                    state.setMotorTarget(MotorEnum.EXTENDO, 0);
                }

                if (transferSampleTimer.seconds() > 0.5) {
                    if (!(hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition()>150)) {
                        state.setMotorTarget(MotorEnum.EXTENDO_PITCH,0);
                    }

                }
                if (hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.getCurrentPosition()<100) {
                    state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,200);
                    state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT,200);
                    isTransferSampleActive=false;
                }
            }

        }
    }
}
