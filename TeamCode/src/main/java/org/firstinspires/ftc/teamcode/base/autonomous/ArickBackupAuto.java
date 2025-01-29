package org.firstinspires.ftc.teamcode.base.autonomous;

import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.CRServos;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucket;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucketSlides;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawWrist;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.drive;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendoPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.motors;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.servos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.BotMotor;

import java.util.Objects;

@Autonomous(name = "MotionProfileFourSample", group = "Autonomous")
public class ArickBackupAuto extends LinearOpMode {
    public static class GlobalPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (BotMotor motor: TeleOpComponents.motors){
                if (Objects.equals(motor.MOVEMENT_MODE, "MOTION_PROFILE")) {
                    motor.createPendingMotionProfiles();
                    motor.runMotionProfileOnce();
                }
            }
            return true;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry,new Pose2d(0,0,Math.toRadians(90)));
        waitForStart();
        clawFingers.setPosition(clawFingers.getPos("openPosition"));
        clawWrist.setPosition(clawWrist.getPos("normalPosition"));
        clawPitch.setPosition(clawPitch.getPos("transferPosition"));
        innerClawPitch.setPosition(innerClawPitch.getPos("transferPosition"));
        bucket.setPosition(bucket.getPos("transferPosition"));

        Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-39,-62.5, Math.toRadians(90)))
                // Score preload bucket
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket2 = drive.actionBuilder(new Pose2d(-59,-59, Math.toRadians(45)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-47,-56), Math.toRadians(90))
                .build();
        Action onePlusThreeBucket3 = drive.actionBuilder(new Pose2d(-47,-56, Math.toRadians(90)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-59,-59, Math.toRadians(45)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-57.5,-56), Math.toRadians(96))
                .build();
        Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-57.5,-56, Math.toRadians(93)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-59,-59, Math.toRadians(45)))
                // sample zone 3
                .strafeToLinearHeading(new Vector2d(-65,-55), Math.toRadians(108))
                .build();
        Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-65,-55, Math.toRadians(108)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();
        Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-59,-59, Math.toRadians(45)))
                // park
                .strafeToLinearHeading(new Vector2d(-44,-6), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-24.2,-6), Math.toRadians(180))

                .build();

        for (int i=0;i<TeleOpComponents.motors.size();i++){
            if (Objects.equals(motors.get(i).MOVEMENT_MODE, "MOTION_PROFILE")) {
                TeleOpComponents.motors.get(i).LOOP_TIMER.reset();
            }
        }
        Actions.runBlocking(
            new ParallelAction(
                new GlobalPID(),
                new SequentialAction(
                        new ParallelAction(
                                // score preload bucket
                                onePlusThreeBucket1,
                                //close claw
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        bucketSlides.setTargetAction(1070)
                                )

                        ),
                        // Move bucketSlides up to scoring position

                        new SleepAction(0.8),
                        // Rotate bucket to score
                        bucket.setPositionAction(158),
                        new SleepAction(0.7),
                        // Move bucket back to default position
                        bucket.setPositionAction(36),
                        // Avoid level 4 hang
                        new SleepAction(0.4),

                        // Drive to sample zone 1, lower extendo, retract extendo pitch when driving
                        new ParallelAction(
                                // Drive to sample zone 1
                                onePlusThreeBucket2,
                                bucketSlides.setTargetAction(0),
                                innerClawPitch.setPositionAction(5),
                                clawPitch.setPositionAction(68),
                                // Extendo pitch pickup position
                                extendoPitch.setTargetAction(-960)
                        ),

                        new SleepAction(0.3),
                        // Extendo to sample zone 1
                        extendo.setTargetAction(800),

                        // Claw pitch picking up pos
                        new SleepAction(0.5),

                        new ParallelAction(
                                innerClawPitch.setPositionAction(78),
                                clawPitch.setPositionAction(13)
                        ),
                        // Close Claw
                        new SleepAction(0.3),

                        clawFingers.setPositionAction(20),
                        new SleepAction(0.3),

                        new ParallelAction(
                                clawWrist.setPositionAction(95),
                                innerClawPitch.setPositionAction(5),
                                clawPitch.setPositionAction(68)
                        ),

                        // Retract extendo, transfer and move to scoring pos
                        new ParallelAction(
                                // Move to scoring position
                                onePlusThreeBucket3,
                                new SequentialAction(
                                        // Retract extendo
                                        extendo.setTargetAction(0),
                                        innerClawPitch.setPositionAction(200),
                                        // Claw pitch transfer position
                                        new SleepAction(0.6),
                                        // Extendo pitch transfer position
                                        extendoPitch.setTargetAction(0),
                                        new SleepAction(0.8),
                                        clawPitch.setPositionAction(110),
                                        new SleepAction(0.3),
                                        clawFingers.setPositionAction(92),
                                        new SleepAction(0.4),
                                        clawPitch.setPositionAction(72.4),
                                        innerClawPitch.setPositionAction(160)
                                )
                        ),
                        // Wait for sample to settle in bucket
                        new SleepAction(0.4),
                        // Move bucketSlides up to scoring position
                        bucketSlides.setTargetAction(1070),
                        new SleepAction(0.8),
                        // Rotate bucket to score
                        bucket.setPositionAction(158),
                        new SleepAction(0.7),
                        // Move bucket back to default position
                        bucket.setPositionAction(36),
                        new SleepAction(0.4),
                        // Avoid level 4 hang


                        new ParallelAction(
                                // Drive to sample zone 2
                                onePlusThreeBucket4,
                                bucketSlides.setTargetAction(0),
                                innerClawPitch.setPositionAction(5),
                                clawPitch.setPositionAction(68),
                                // Extendo pitch pickup position
                                extendoPitch.setTargetAction(-960)
                        ),

                        new SleepAction(0.3),
                        // Extendo to sample zone 1
                        extendo.setTargetAction(800),

                        // Claw pitch picking up pos
                        new SleepAction(0.5),

                        new ParallelAction(
                                innerClawPitch.setPositionAction(78),
                                clawPitch.setPositionAction(13)
                        ),
                        // Close Claw
                        new SleepAction(0.3),

                        clawFingers.setPositionAction(20),
                        new SleepAction(0.3),

                        new ParallelAction(
                                clawWrist.setPositionAction(95),
                                innerClawPitch.setPositionAction(5),
                                clawPitch.setPositionAction(68)
                        ),

                        // Retract extendo, transfer and move to scoring pos
                        new ParallelAction(
                                // Move to scoring position
                                onePlusThreeBucket5,
                                new SequentialAction(
                                        // Retract extendo
                                        extendo.setTargetAction(0),
                                        innerClawPitch.setPositionAction(200),
                                        // Claw pitch transfer position
                                        new SleepAction(0.6),
                                        // Extendo pitch transfer position
                                        extendoPitch.setTargetAction(0),
                                        new SleepAction(0.8),
                                        clawPitch.setPositionAction(110),
                                        new SleepAction(0.3),
                                        clawFingers.setPositionAction(92),
                                        new SleepAction(0.4),
                                        clawPitch.setPositionAction(72.4),
                                        innerClawPitch.setPositionAction(160)
                                )
                        ),
                        // Wait for sample to settle in bucket
                        new SleepAction(0.4),
                        // Move bucketSlides up to scoring position
                        bucketSlides.setTargetAction(1070),
                        new SleepAction(0.8),
                        // Rotate bucket to score
                        bucket.setPositionAction(158),
                        new SleepAction(0.7),
                        // Move bucket back to default position
                        bucket.setPositionAction(36),
                        // Avoid level 4 hang
                        new SleepAction(0.4),
                        new ParallelAction(
                                // Drive to sample zone 1
                                onePlusThreeBucket6,
                                bucketSlides.setTargetAction(0),
                                innerClawPitch.setPositionAction(5),
                                clawPitch.setPositionAction(68),
                                // Extendo pitch pickup position
                                extendoPitch.setTargetAction(-960)
                        ),

                        new SleepAction(0.3),
                        // Extendo to sample zone 1
                        extendo.setTargetAction(800),

                        // Claw pitch picking up pos
                        new SleepAction(0.5),

                        new ParallelAction(
                                innerClawPitch.setPositionAction(78),
                                clawPitch.setPositionAction(13)
                        ),
                        // Close Claw
                        new SleepAction(0.3),

                        clawFingers.setPositionAction(20),
                        new SleepAction(0.3),

                        new ParallelAction(
                                clawWrist.setPositionAction(95),
                                innerClawPitch.setPositionAction(5),
                                clawPitch.setPositionAction(68)
                        ),

                        // Retract extendo, transfer and move to scoring pos
                        new ParallelAction(
                                // Move to scoring position
                                onePlusThreeBucket7,
                                new SequentialAction(
                                        // Retract extendo
                                        extendo.setTargetAction(0),
                                        innerClawPitch.setPositionAction(200),
                                        // Claw pitch transfer position
                                        new SleepAction(0.6),
                                        // Extendo pitch transfer position
                                        extendoPitch.setTargetAction(0),
                                        new SleepAction(0.8),
                                        clawPitch.setPositionAction(110),
                                        new SleepAction(0.3),
                                        clawFingers.setPositionAction(92),
                                        new SleepAction(0.4),
                                        clawPitch.setPositionAction(72.4),
                                        innerClawPitch.setPositionAction(160)
                                )
                        ),
                        // Wait for sample to settle in bucket
                        new SleepAction(0.4),
                        // Move bucketSlides up to scoring position
                        bucketSlides.setTargetAction(1070),
                        new SleepAction(0.8),
                        // Rotate bucket to score
                        bucket.setPositionAction(158),
                        new SleepAction(0.7),
                        // Move bucket back to default position
                        bucket.setPositionAction(36),
                        // Avoid level 4 hang
                        new SleepAction(0.4),
                        // Move bucketSlides back to hang position
                        bucketSlides.setTargetAction(300),
                        // Park and low rung

                        onePlusThreeBucket8


                )
            )
        );
        motors.clear();
        servos.clear();
        CRServos.clear();
    }
}
