package org.firstinspires.ftc.teamcode.autonomous;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.InitializeMechanisms;
import org.firstinspires.ftc.teamcode.PinpointDrive;
@Autonomous
public class FiveSampleHighBucket extends LinearOpMode {
    double kP = 0.015;
    DcMotor extendo;
    DcMotor extendoPitch;
    DcMotor hang;
    DcMotor bucketSlides;
    Servo clawPitchLeft;
    Servo clawPitchRight;
    Servo clawFingers;
    Servo clawWrist;
    Servo bucket;
    public double extendoTarget = 0;
    public double extendoPitchTarget = 0;
    public double clawPitchPosition = 200;
    public double clawFingerPosition = 0;
    public double clawWristPosition = 76.5;
    public double bucketSlidesTarget = 0;
    public double bucketPosition = 85;
    public double hangTarget = 0;

    public class GlobalPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * 0.005);
            hang.setPower((hangTarget - hang.getCurrentPosition()) * kP);
            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * kP);
            clawPitchLeft.setPosition(clawPitchPosition / 270);
            clawPitchRight.setPosition(clawPitchPosition / 270);
            bucket.setPosition(bucketPosition / 270);
            clawFingers.setPosition(clawFingerPosition / 180);
            clawWrist.setPosition(clawWristPosition / 180);
            return true;
        }
    }

    public Action globalPID() {
        return new GlobalPID();
    }

    public class SetExtendoTarget implements Action {
        private double target;

        public SetExtendoTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendoTarget = target;
            return false;
        }
    }

    public Action setExtendoTarget(double target) {
        return new SetExtendoTarget(target);
    }

    public class SetExtendoPitchTarget implements Action {
        private double target;

        public SetExtendoPitchTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendoPitchTarget = target;
            return false;
        }
    }

    public Action setExtendoPitchTarget(double target) {
        return new SetExtendoPitchTarget(target);
    }

    public class SetBucketSlidesTarget implements Action {
        private double target;

        public SetBucketSlidesTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bucketSlidesTarget = target;
            return false;
        }
    }

    public Action setBucketSlidesTarget(double target) {
        return new SetBucketSlidesTarget(target);
    }

    public class SetHangTarget implements Action {
        private double target;

        public SetHangTarget(double target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hangTarget = target;
            return false;
        }
    }

    public Action setHangTarget(double target) {
        return new SetHangTarget(target);
    }

    public class SetClawPitchPosition implements Action {
        private double position;

        public SetClawPitchPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawPitchPosition = position;
            return false;
        }
    }

    public Action setClawPitchPosition(double position) {
        return new SetClawPitchPosition(position);
    }

    public class SetClawFingerPosition implements Action {
        private double position;

        public SetClawFingerPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawFingerPosition = position;
            return false;
        }
    }

    public Action setClawFingerPosition(double position) {
        return new SetClawFingerPosition(position);
    }

    public class SetClawWristPosition implements Action {
        private double position;

        public SetClawWristPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawWristPosition = position;
            return false;
        }
    }

    public Action setClawWristPosition(double position) {
        return new SetClawWristPosition(position);
    }

    public class SetBucketPosition implements Action {
        private double position;

        public SetBucketPosition(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bucketPosition = position;
            return false;
        }
    }

    public Action setBucketPosition(double position) {
        return new SetBucketPosition(position);
    }
    @Override
    public void runOpMode(){

        InitializeMechanisms initializeMechanisms = new InitializeMechanisms(hardwareMap);
        Pose2d initialPose = new Pose2d(-42, -62.5, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action fiveSampleAuto1= drive.actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score sample preload
                .strafeToLinearHeading(new Vector2d(-58,-53), Math.toRadians(225))
                .build();
        Action fiveSampleAuto2= drive.actionBuilder(new Pose2d(-58,-53, Math.toRadians(225)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-53,-51), Math.toRadians(270))
                .build();
        Action fiveSampleAuto3 = drive.actionBuilder(new Pose2d(-53,-51, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                .build();
        Action fiveSampleAuto4 = drive.actionBuilder(new Pose2d(-55,-55, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-63,-51), Math.toRadians(273))
                .build();
        Action fiveSampleAuto5 = drive.actionBuilder(new Pose2d(-63,-51, Math.toRadians(273)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                .build();
        Action fiveSampleAuto6 = drive.actionBuilder(new Pose2d(-55,-55, Math.toRadians(225)))
                // sample zone 3
                .strafeToLinearHeading(new Vector2d(-67,-51), Math.toRadians(285))
                .build();
        Action fiveSampleAuto7 = drive.actionBuilder(new Pose2d(-67,-51, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(225))
                .build();
        Action fiveSampleAuto8 = drive.actionBuilder(new Pose2d(-55,-55, Math.toRadians(225)))
                // go to scarsdale sample for the wr
                .strafeToLinearHeading(new Vector2d(-39,-48), Math.toRadians(180))
                .build();
        Action fiveSampleAuto9 = drive.actionBuilder(new Pose2d(-39,-48, Math.toRadians(180)))
                // score sample lets go
                .strafeToLinearHeading(new Vector2d(-53.5,-53), Math.toRadians(225))
                .build();
        Action fiveSampleAuto10 = drive.actionBuilder(new Pose2d(-53.5,-53, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-44,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24.2,-6), Math.toRadians(0))
                .build();

        extendo = initializeMechanisms.extendo;
        extendoPitch = initializeMechanisms.extendoPitch;
        hang = initializeMechanisms.hang;
        bucketSlides = initializeMechanisms.bucketSlides;
        clawPitchLeft = initializeMechanisms.clawPitchLeft;
        clawPitchRight = initializeMechanisms.clawPitchRight;
        clawFingers = initializeMechanisms.clawFingers;
        clawWrist = initializeMechanisms.clawWrist;
        bucket = initializeMechanisms.bucket;

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        globalPID(),
                        new SequentialAction(
                                setClawPitchPosition(200),
                                fiveSampleAuto1,
                                // Open claw fully bc bucketSlides coming down later
                                setClawFingerPosition(120),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.5),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                setBucketSlidesTarget(0),
                                // Drive to sample zone 1, lower extendo, retract extendo pitch when driving
                                // Retract extendo to default position
                                setExtendoTarget(0),
                                new SleepAction(0.3),
                                new ParallelAction(
                                        setBucketPosition(85),

                                        // Drive to sample zone 1
                                        fiveSampleAuto2,
                                        // Extendo pitch pickup position
                                        setExtendoPitchTarget(1350)
                                ),
                                // Extendo to sample zone 1
                                setExtendoTarget(425),
                                new SleepAction(0.3),

                                // Claw pitch picking up pos
                                setClawPitchPosition(30.5),
                                new SleepAction(0.3),
                                // Close Claw
                                setClawFingerPosition(39),
                                new SleepAction(0.3),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        fiveSampleAuto3,
                                        new SequentialAction(
                                                // Retract extendo
                                                setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                new SleepAction(0.2),
                                                // Extendo pitch transfer position
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.8),
                                                setClawPitchPosition(200),
                                                new SleepAction(0.3),
                                                // Open claw fully bc bucketSlides coming down later
                                                setClawFingerPosition(120)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.5),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                setBucketSlidesTarget(0),
                                // Drive to sample zone 2, while driving lower extendoPitch
                                new ParallelAction(
                                        // Drive to sample zone 2
                                        fiveSampleAuto4,
                                        // Lower extendo pitch to pickup pos
                                        setExtendoPitchTarget(1350)
                                ),
                                // Extendo to sample zone 2
                                setExtendoTarget(425),

                                new SleepAction(0.3),
                                // Claw pitch picking up position
                                setClawPitchPosition(30.5),
                                new SleepAction(0.3),
                                // Close claw
                                setClawFingerPosition(39),

                                new SleepAction(0.3),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        fiveSampleAuto5,
                                        new SequentialAction(
                                                // Retract extendo
                                                setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3),
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.8),
                                                setClawPitchPosition(200),
                                                new SleepAction(0.5),
                                                // Open claw fully bc bucketSlides coming down later
                                                setClawFingerPosition(120)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.5),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                setBucketSlidesTarget(0),
                                // Drive to sample zone 3, while driving lower extendoPitch
                                new ParallelAction(
                                        // Drive to sample zone 3
                                        fiveSampleAuto6,
                                        // Lower extendo pitch to pickup pos
                                        setExtendoPitchTarget(1350)
                                ),
                                // Extendo to sample zone 3
                                setExtendoTarget(425),
                                new SleepAction(0.3),

                                // Claw pitch picking up position
                                setClawPitchPosition(30.5),
                                new SleepAction(0.3),
                                // Close claw
                                setClawFingerPosition(39),

                                new SleepAction(0.3),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        fiveSampleAuto7,
                                        new SequentialAction(
                                                // Retract extendo
                                                setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3),
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.8),
                                                setClawPitchPosition(200),
                                                new SleepAction(0.5),
                                                // Open claw fully bc bucketSlides coming down later
                                                setClawFingerPosition(120)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.5),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides to default position
                                setBucketSlidesTarget(0),
                                /** SCHEDULED FOR CHANGE **/
                                new ParallelAction(
                                        fiveSampleAuto8,
                                        // Extendo pitch to pickup pos
                                        setExtendoPitchTarget(1350),
                                        // Claw pitch picking up position
                                        setClawPitchPosition(30.5)
                                ),
                                // Extend extendo
                                setExtendoTarget(380),
                                new SleepAction(0.6),
                                // Close Claw
                                setClawFingerPosition(39),
                                new SleepAction(0.4),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        fiveSampleAuto9,
                                        new SequentialAction(
                                                // Retract extendo
                                                setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3),
                                                setExtendoPitchTarget(0),
                                                new SleepAction(0.8),
                                                setClawPitchPosition(200),
                                                new SleepAction(0.5),
                                                // Open claw fully bc bucketSlides coming down later
                                                setClawFingerPosition(120)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.5),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to hang position
                                new ParallelAction(
                                        setBucketSlidesTarget(300),
                                        fiveSampleAuto10
                                )
                        )
                )
        );
    }
}
