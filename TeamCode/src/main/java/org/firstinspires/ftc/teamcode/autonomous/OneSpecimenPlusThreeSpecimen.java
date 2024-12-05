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
//import org.firstinspires.ftc.teamcode.PIDCoefficients;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@Autonomous
public class OneSpecimenPlusThreeSpecimen extends LinearOpMode {
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
    public double clawPitchPosition = 15;
    public double clawFingerPosition = 0;
    public double clawWristPosition = 76.5;
    public double bucketSlidesTarget = 0;
    public double bucketPosition = 85;
    public double hangTarget = 0;

    public class GlobalPID implements Action {
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
        return new SetBucketPosition(position);
    }


    @Override
    public void runOpMode() {

        InitializeMechanisms initializeMechanisms = new InitializeMechanisms(hardwareMap);
        Pose2d initialPose = new Pose2d(24, -62, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action onePlusThreeSpecimen1 = drive.actionBuilder(new Pose2d(24, -62, Math.toRadians(270)))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(8,-46), Math.toRadians(270))
                .build();
        Action onePlusThreeSpecimen2 = drive.actionBuilder(new Pose2d(8, -46, Math.toRadians(270)))
                // Go to sample 1 sample zone
                .strafeToLinearHeading(new Vector2d(30.7,-45.1), Math.toRadians(230))
                .build();
        Action onePlusThreeSpecimen3 = drive.actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(230)))
                // Rotate towards observation zone
                .turnTo(Math.toRadians(135))
                .build();
        Action onePlusThreeSpecimen4 = drive.actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(135)))
                // Rotate to sample 2 in sample zone
                .strafeToLinearHeading(new Vector2d(40,-41), Math.toRadians(220))
                .build();
        Action onePlusThreeSpecimen5 = drive.actionBuilder(new Pose2d(40, -41, Math.toRadians(220)))
                // Rotate towards observation zone 2nd time
                .turnTo(Math.toRadians(110))
                .build();
        Action onePlusThreeSpecimen6 = drive.actionBuilder(new Pose2d(40, -41, Math.toRadians(110)))
                // Rotate towards sample 3 in sample zone
                .strafeToLinearHeading(new Vector2d(51,-41), Math.toRadians(220))
                .build();
        Action onePlusThreeSpecimen7 = drive.actionBuilder(new Pose2d(51, -41, Math.toRadians(220)))
                // Rotate towards observation zone
                .turnTo(Math.toRadians(80))
                .build();
        Action onePlusThreeSpecimen8 = drive.actionBuilder(new Pose2d(51, -41, Math.toRadians(80)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(135))
                .build();
        Action onePlusThreeSpecimen9 = drive.actionBuilder(new Pose2d(29, -52, Math.toRadians(135)))
                // Score second specimen
                .strafeToLinearHeading(new Vector2d(4,-46), Math.toRadians(270))
                .build();
        Action onePlusThreeSpecimen10 = drive.actionBuilder(new Pose2d(4, -46, Math.toRadians(270)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(135))
                .build();
        Action onePlusThreeSpecimen11 = drive.actionBuilder(new Pose2d(29, -52, Math.toRadians(135)))
                // Score third specimen
                .strafeToLinearHeading(new Vector2d(0,-46), Math.toRadians(270))
                .build();
        Action onePlusThreeSpecimen12 = drive.actionBuilder(new Pose2d(0, -46, Math.toRadians(270)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(135))
                .build();
        Action onePlusThreeSpecimen13 = drive.actionBuilder(new Pose2d(29, -52, Math.toRadians(135)))
                // Score fourth specimen
                .strafeToLinearHeading(new Vector2d(-4,-46), Math.toRadians(270))
                .build();
        Action onePlusThreeSpecimen14 = drive.actionBuilder(new Pose2d(-4, -46, Math.toRadians(270)))
                // Park
                .strafeToLinearHeading(new Vector2d(34,-62), Math.toRadians(270))
                .build();
        Action setUpSpecimen = new SequentialAction(
                // Drive and prepare extendo pitch
                new SleepAction(0.5),
                new ParallelAction(
                        setBucketPosition(205),
                        setExtendoPitchTarget(400),
                        setClawPitchPosition(104)
                ),
                new SleepAction(0.5),
                setExtendoTarget(500),

                new SleepAction(0.5));
        Action scoreSpecimen = new SequentialAction(
                // Lower extendo
                setExtendoPitchTarget(850),
                new SleepAction(0.7),
                // Open claw
                setClawFingerPosition(90),
                new SleepAction(0.5)
        );
        Action sampleZone1 = new SequentialAction(
                // Drive to sample zone 1, lower extendo, retract extendo pitch when driving
                // Retract extendo to default position
                new ParallelAction(
                        onePlusThreeSpecimen2,
                        new SequentialAction(
                            setExtendoTarget(0),
                            new SleepAction(0.5),
                            // Extendo pitch pickup position
                            setExtendoPitchTarget(1350),
                            // Extendo to sample zone 1
                            /** CHANGE THIS **/
                            new SleepAction(0.5),
                            setExtendoTarget(425),
                            new SleepAction(0.5)
                        )
                ),
                // Claw pitch picking up pos
                setClawPitchPosition(30.5),
                new SleepAction(0.5),
                // Close Claw
                setClawFingerPosition(39),
                new SleepAction(0.5)
        );
        Action sampleZone2 = new SequentialAction(
                // Drive to sample zone 1, lower extendo, retract extendo pitch when driving
                // Retract extendo to default position
                new ParallelAction(
                        onePlusThreeSpecimen4,
                        new SequentialAction(
                                setExtendoTarget(0),
                                new SleepAction(0.5),
                                // Extendo pitch pickup position
                                setExtendoPitchTarget(1350),
                                // Extendo to sample zone 1
                                /** CHANGE THIS **/
                                new SleepAction(0.5),
                                setExtendoTarget(425),
                                new SleepAction(0.5)
                        )
                ),
                // Claw pitch picking up pos
                setClawPitchPosition(30.5),
                new SleepAction(0.5),
                // Close Claw
                setClawFingerPosition(39),
                new SleepAction(0.5)
        );
        Action sampleZone3 = new SequentialAction(
                // Drive to sample zone 1, lower extendo, retract extendo pitch when driving
                // Retract extendo to default position
                new ParallelAction(
                        onePlusThreeSpecimen6,
                        new SequentialAction(
                                setExtendoTarget(0),
                                new SleepAction(0.5),
                                // Extendo pitch pickup position
                                setExtendoPitchTarget(1350),
                                // Extendo to sample zone 1
                                /** CHANGE THIS **/
                                new SleepAction(0.5),
                                setExtendoTarget(425),
                                new SleepAction(0.5)
                        )
                ),
                // Claw pitch picking up pos
                setClawPitchPosition(30.5),
                new SleepAction(0.5),
                // Close Claw
                setClawFingerPosition(39),
                new SleepAction(0.5)
        );
        Action pickUpPosition = new SequentialAction(
                setExtendoPitchTarget(1350),
                /** CHANGE THIS **/
                setExtendoTarget(200),
                // Claw pitch picking up pos
                setClawPitchPosition(30.5),
                new SleepAction(0.5),
                // Close Claw
                setClawFingerPosition(39),
                new SleepAction(0.5)

        );


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
                                setUpSpecimen,
                                onePlusThreeSpecimen1,
                                scoreSpecimen,
                                sampleZone1,
                                onePlusThreeSpecimen3,
                                setClawFingerPosition(90),
                                new SleepAction(0.5),
                                sampleZone2,
                                onePlusThreeSpecimen5,
                                setClawFingerPosition(90),
                                new SleepAction(0.5),
                                sampleZone3,
                                onePlusThreeSpecimen7,
                                setClawFingerPosition(90),
                                new SleepAction(0.5),
                                pickUpPosition,
                                scoreSpecimen,
                                pickUpPosition,
                                scoreSpecimen,
                                pickUpPosition,
                                scoreSpecimen,
                                onePlusThreeSpecimen14
                        )
                )
        );
    }
}
