package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;

import java.util.Arrays;

public class AutoMoveToSamplePickUpPosition implements Action {
    HardwareConfig hw = HardwareConfig.getInstance();
    LLResultTypes.DetectorResult targetDetection;
    double tX;
    double tY;
    String className;
    // Tune these values
    double targetTX = 0;
    double targetTY = 0;
    double tXError;
    double tYError;
    // Tune these values
    double tXTolerance = 0.5;
    double tYTolerance = 0.5;
    double targetDeviance;
    // Tune this value, measure the furthest possible sample that can be picked up
    double minimumTA = 0.01;
    String[] imageClassNames;
    double motorPower;
    // Tune this
    double kP = 0.015;

    public AutoMoveToSamplePickUpPosition(String... imageClasses){
        this.imageClassNames = imageClasses;
        hw.getLimelightConfig().limelight.pipelineSwitch(4);
        hw.getLimelightConfig().limelight.start();
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket){
        LLResult result = hw.getLimelightConfig().limelight.getLatestResult();

        // Reset Value
        double smallestTargetDeviance = Double.POSITIVE_INFINITY;

        if (result != null && result.isValid()) {

            for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {

                className = detection.getClassName();

                if (Arrays.asList(imageClassNames).contains(className) && detection.getTargetArea() > minimumTA) {

                    tX = detection.getTargetXPixels();
                    tY = detection.getTargetYDegrees();

                    targetDeviance = Math.sqrt(Math.pow(targetTX - tX, 2) + Math.pow(targetTY - tY, 2));

                    if (targetDeviance < smallestTargetDeviance) {
                        smallestTargetDeviance = targetDeviance;
                        targetDetection = detection;
                        tXError = targetTX - targetDetection.getTargetXDegrees();
                        tYError = targetTY - targetDetection.getTargetYDegrees();
                    }
                }
            }
        }
        if (tXError < 0){
            // Move drivetrain left
            motorPower = Math.abs(kP * tXError);
            hw.getMotorConfig(MotorEnum.LEFT_BACK).motor.setPower(motorPower);
            hw.getMotorConfig(MotorEnum.RIGHT_BACK).motor.setPower(-motorPower);
            hw.getMotorConfig(MotorEnum.LEFT_FRONT).motor.setPower(-motorPower);
            hw.getMotorConfig(MotorEnum.RIGHT_FRONT).motor.setPower(motorPower);
        }
        else if (tXError > 0){
            // Move drivetrain right
            motorPower = Math.abs(kP * tXError);
            hw.getMotorConfig(MotorEnum.LEFT_BACK).motor.setPower(-motorPower);
            hw.getMotorConfig(MotorEnum.RIGHT_BACK).motor.setPower(motorPower);
            hw.getMotorConfig(MotorEnum.LEFT_FRONT).motor.setPower(motorPower);
            hw.getMotorConfig(MotorEnum.RIGHT_FRONT).motor.setPower(-motorPower);
        }
        if (Math.abs(tXError) < tXTolerance) {
            if (tYError < 0) {
                // Move drivetrain down
                motorPower = Math.abs(kP * tYError);
                hw.getMotorConfig(MotorEnum.LEFT_BACK).motor.setPower(-motorPower);
                hw.getMotorConfig(MotorEnum.RIGHT_BACK).motor.setPower(-motorPower);
                hw.getMotorConfig(MotorEnum.LEFT_FRONT).motor.setPower(-motorPower);
                hw.getMotorConfig(MotorEnum.RIGHT_FRONT).motor.setPower(-motorPower);
            } else if (tYError > 0) {
                // Move drivetrain up
                motorPower = Math.abs(kP * tYError);
                hw.getMotorConfig(MotorEnum.LEFT_BACK).motor.setPower(motorPower);
                hw.getMotorConfig(MotorEnum.RIGHT_BACK).motor.setPower(motorPower);
                hw.getMotorConfig(MotorEnum.LEFT_FRONT).motor.setPower(motorPower);
                hw.getMotorConfig(MotorEnum.RIGHT_FRONT).motor.setPower(motorPower);
            }
        }
        boolean driveTrainIsPositioned = tXError < tXTolerance || tYError < tYTolerance;

        return !driveTrainIsPositioned;
    }
}