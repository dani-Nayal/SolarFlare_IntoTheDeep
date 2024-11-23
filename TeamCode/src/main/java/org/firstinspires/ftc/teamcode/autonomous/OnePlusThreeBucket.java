package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.InitializeMechanisms;
import org.firstinspires.ftc.teamcode.PIDCoefficients;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "OnePlusThreeBucket", group = "Autonomous")
public class OnePlusThreeBucket extends LinearOpMode {
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
    public double clawPitchPosition = 104;
    public double clawFingerPosition = 50;
    public double clawWristPosition = 76.5;
    public double bucketSlidesTarget = 0;
    public double bucketPosition = 85;
    public double hangTarget = 0;

    public class GlobalPID implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * kP);
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

    public Action GlobalPID() {
        return new GlobalPID();
    }

    public class SetExtendoTarget implements Action {
        private double target;

        public SetExtendoTarget(double target) {
            this.target = target;
        }

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

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bucketPosition = position;
            return false;
        }
    }

    public Action setBucketPosition(double position) {
        return new SetBucketTarget(position);
    }

    @Override
    public void runOpMode() {


        InitializeMechanisms initializeMechanisms = new InitializeMechanisms(hardwareMap);
        Pose2d initialPose = new Pose2d(-42, -62.5, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9,-58), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket2 = drive.actionBuilder(new Pose2d(-9,-58, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-53), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket3 = drive.actionBuilder(new Pose2d(-48,-53, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-57,-50), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-57,-50, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61,-50), Math.toRadians(285))
                .build();
        Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-61,-50, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-35,-6), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-23.4,-6), Math.toRadians(180))
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
                        GlobalPID(),
                        new SequentialAction(
                                // Close claw
                                setClawFingerPosition(0),
                                // Drive and prepare extendo
                                new ParallelAction(
                                        // Drive to scoring pos
                                        onePlusThreeBucket1,
                                        // Raise extendo pitch and extendo
                                        new SequentialAction(
                                                setExtendoPitchTarget(450),
                                                new SleepAction(0.5),
                                                setExtendoTarget(500)
                                        )
                                ),
                                // Lower extendo pitch
                                setExtendoPitchTarget(760),
                                new SleepAction(1.5),
                                // Open claw
                                setClawFingerPosition(50),
                                // Retract extendo to default position
                                setExtendoTarget(0),
                                new SleepAction(0.5),
                                // Drive to sample zone 1 and lower extendo pitch when driving
                                new ParallelAction(
                                        // Drive to sample zone 1
                                        onePlusThreeBucket2,
                                        // Extendo pitch pickup position
                                        setExtendoPitchTarget(1421)
                                ),
                                /** CHANGE LATER **/
                                setExtendoTarget(250),
                                // Claw pitch picking up pos
                                setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close Claw
                                setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket3,
                                        new SequentialAction(
                                                // Retract extendo
                                                setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                setClawPitchPosition(195),
                                                new SleepAction(0.3),
                                                // Extendo pitch transfer position
                                                setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketSlides coming down later
                                                setClawFingerPosition(80)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                setBucketSlidesTarget(0),
                                // Drive to sample zone 2, while driving lower extendoPitch
                                new ParallelAction(
                                        // Drive to sample zone 2
                                        onePlusThreeBucket4,
                                        // Lower extendo pitch to pickup pos
                                        setExtendoPitchTarget(1421)
                                ),
                                /** CHANGE LATER **/
                                setExtendoTarget(250),
                                // Claw pitch picking up position
                                setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close claw
                                setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket5,
                                        new SequentialAction(
                                                // Retract extendo
                                                setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                setClawPitchPosition(195),
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3),
                                                setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketSlides coming down later
                                                setClawFingerPosition(80)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                setBucketSlidesTarget(0),
                                // Drive to sample zone 3, while driving lower extendoPitch
                                new ParallelAction(
                                        // Drive to sample zone 3
                                        onePlusThreeBucket6,
                                        // Lower extendo pitch to pickup pos
                                        setExtendoPitchTarget(1421)
                                ),
                                /** CHANGE LATER **/
                                setExtendoTarget(250),
                                // Claw pitch picking up position
                                setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close claw
                                setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket7,
                                        new SequentialAction(
                                                // Retract extendo
                                                setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                setClawPitchPosition(195),
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3),
                                                setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketSlides coming down later
                                                setClawFingerPosition(80)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                setBucketSlidesTarget(1100),
                                new SleepAction(0.6),
                                // Rotate bucket to score
                                setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                setBucketPosition(85),
                                // Avoid level 4 hang
                                new SleepAction(0.4),
                                // Move bucketSlides back to down position
                                setBucketSlidesTarget(0),
                                // Park
                                onePlusThreeBucket8
                                /** ADD ACTUAL LEVEL 1 ASCENT**/

                        )
                )
        );
    }
}
