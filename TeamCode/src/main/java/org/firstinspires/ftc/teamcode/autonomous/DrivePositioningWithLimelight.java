package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous
public class DrivePositioningWithLimelight extends LinearOpMode {

    // Proportional control output is relative, not absolute
    double targetTX = 0;
    double targetTY = 0;
    double tXTolerance = 0.5;
    double tYTolerance = 0.5;
    double kP = 0;
    HardwareConfig hw;
    PinpointDrive drive;
    @Override
    public void runOpMode() {
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        HardwareConfig.makeInstance(hardwareMap);
        hw = HardwareConfig.getInstance();

        hw.getLimelightConfig().limelight.pipelineSwitch(4);

        waitForStart();

        hw.getLimelightConfig().limelight.start();

        LLResultTypes.DetectorResult targetDetection = FindTargetSample.findTargetSample("red", "blue", "yellow");

        while (opModeIsActive()) {

            if (targetDetection != null) {
                double tX = targetDetection.getTargetXDegrees();
                double tY = targetDetection.getTargetYDegrees();
                LLResultTypes.DetectorResult closestDetection = null;
                double minDistance = Double.POSITIVE_INFINITY;
                double dist;

                for (LLResultTypes.DetectorResult detection: hw.getLimelightConfig().limelight.getLatestResult().getDetectorResults()){
                    dist = Math.sqrt(Math.pow(detection.getTargetXDegrees()-tX,2)+Math.pow(detection.getTargetXDegrees()-tX,2));
                    if (dist < minDistance){
                        minDistance = dist;
                        closestDetection = detection;
                    }
                }

                targetDetection = closestDetection;

                double tXError = Math.abs(targetTX - tX);
                double tYError = Math.abs(targetTY - tY);

                if (tXError < 0 && tXError > tXTolerance) {
                    // move drivetrain left
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToX(drive.pose.position.x - kP * tXError)
                                    .build()
                    );
                }
                if (tXError > 0 && tXError > tXTolerance) {
                    // move drivetrain right
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToX(drive.pose.position.x + kP * tXError)
                                    .build()
                    );
                }
                if (tYError < 0 && tYError > tYTolerance) {
                    // move drivetrain down
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToY(drive.pose.position.y - kP * tYError)
                                    .build()
                    );
                }
                if (tYError > 0 && tYError > tYTolerance) {
                    // move drivetrain up
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToY(drive.pose.position.y + kP * tYError)
                                    .build()
                    );
                }
            }
        }
    }
}


