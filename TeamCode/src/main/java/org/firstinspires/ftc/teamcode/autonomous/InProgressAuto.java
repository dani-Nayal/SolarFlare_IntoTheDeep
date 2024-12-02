package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous
public class InProgressAuto extends LinearOpMode {
    HardwareConfig hw;
    CustomActions actions;
    RobotState state;
    @Override
    public void runOpMode() {
        hw = new HardwareConfig(hardwareMap);
        state = new RobotState();
        actions = new CustomActions(state, hw);

        Pose2d initialPose = new Pose2d(-42, -62.5, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-42, -62.5, Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-7, -46.8), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket2 = drive.actionBuilder(new Pose2d(-7, -46.8, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket3 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-63, -52), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-63, -52, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
                // sample zone 3
                .strafeToLinearHeading(new Vector2d(-67, -52), Math.toRadians(285))
                .build();
        Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-67, -52, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-44, -6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-23.4, -6), Math.toRadians(0))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalPID(),
                        actions.updateTelemetry(telemetry),
                        new SequentialAction(
                                // Close claw
                                actions.setClawFingerPosition(39),
                                actions.setBucketPosition(205),
                                // Drive and prepare extendo pitch
                                new SleepAction(0.5),
                                new ParallelAction(
                                        onePlusThreeBucket1,
                                        actions.setExtendoPitchTarget(400),
                                        actions.setClawPitchPosition(104)
                                ),
                                new SleepAction(0.5),
                                // Raise extendo, lower extendoPitch slightly, lower claw pitch slightly
                                new ParallelAction(
                                        actions.setExtendoTarget(500),
                                        actions.setExtendoPitchTarget(400),
                                        actions.setClawPitchPosition(100)
                                ),
                                new SleepAction(0.5),
                                // Lower extendo
                                actions.setExtendoPitchTarget(750),
                                new SleepAction(1),
                                // Open claw
                                actions.setClawFingerPosition(90),
                                new SleepAction(0.5),
                                // Drive to sample zone 1, lower extendo, retract extendo pitch when driving
                                // Retract extendo to default position
                                actions.setExtendoTarget(0),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        actions.setBucketPosition(85),
                                        // Drive to sample zone 1
                                        onePlusThreeBucket2,
                                        // Extendo pitch pickup position
                                        actions.setExtendoPitchTarget(1421)
                                ),
                                // Extendo to sample zone 1
                                actions.setExtendoTarget(400),
                                new SleepAction(0.5),
                                // Claw pitch picking up pos
                                actions.setClawPitchPosition(30.5),
                                new SleepAction(0.5),
                                // Close Claw
                                actions.setClawFingerPosition(39),
                                new SleepAction(0.5),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket3,
                                        new SequentialAction(
                                                // Retract extendo
                                                actions.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                actions.setClawPitchPosition(200),
                                                new SleepAction(0.3),
                                                // Extendo pitch transfer position
                                                actions.setExtendoPitchTarget(0),
                                                new SleepAction(0.8),
                                                // Open claw fully bc bucketSlides coming down later
                                                actions.setClawFingerPosition(120)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                actions.setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                actions.setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                actions.setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                actions.setBucketSlidesTarget(0),
                                // Drive to sample zone 2, while driving lower extendoPitch
                                new ParallelAction(
                                        // Drive to sample zone 2
                                        onePlusThreeBucket4,
                                        // Lower extendo pitch to pickup pos
                                        actions.setExtendoPitchTarget(1421)
                                ),
                                // Extendo to sample zone 2
                                actions.setExtendoTarget(400),
                                new SleepAction(0.5),
                                // Claw pitch picking up position
                                actions.setClawPitchPosition(30.5),
                                new SleepAction(0.5),
                                // Close claw
                                actions.setClawFingerPosition(30),
                                new SleepAction(0.5),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket5,
                                        new SequentialAction(
                                                // Retract extendo
                                                actions.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                actions.setClawPitchPosition(200),
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3),
                                                actions.setExtendoPitchTarget(0),
                                                new SleepAction(0.8),
                                                // Open claw fully bc bucketSlides coming down later
                                                actions.setClawFingerPosition(120)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                actions.setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                actions.setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                actions.setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                actions.setBucketSlidesTarget(0),
                                // Drive to sample zone 3, while driving lower extendoPitch
                                new ParallelAction(
                                        // Drive to sample zone 3
                                        onePlusThreeBucket6,
                                        // Lower extendo pitch to pickup pos
                                        actions.setExtendoPitchTarget(1421)
                                ),
                                // Extendo to sample zone 3
                                actions.setExtendoTarget(400),
                                new SleepAction(0.5),
                                // Claw pitch picking up position
                                actions.setClawPitchPosition(200),
                                new SleepAction(0.5),
                                // Close claw
                                actions.setClawFingerPosition(39),
                                new SleepAction(0.5),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket7,
                                        new SequentialAction(
                                                // Retract extendo

                                                actions.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                actions.setClawPitchPosition(200),
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3),
                                                actions.setExtendoPitchTarget(0),
                                                new SleepAction(0.8),
                                                // Open claw fully bc bucketSlides coming down later
                                                actions.setClawFingerPosition(120)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                actions.setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                actions.setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                actions.setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to low rung touch position
                                actions.setBucketSlidesTarget(100),
                                // Park and low rung
                                onePlusThreeBucket8
                        )
                )
        );
    }
}