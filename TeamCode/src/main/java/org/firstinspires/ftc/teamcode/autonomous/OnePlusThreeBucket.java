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
    InitializeMechanisms initializeMechanisms = new InitializeMechanisms(hardwareMap);
    DcMotor extendo = initializeMechanisms.extendo;
    DcMotor extendoPitch = initializeMechanisms.extendoPitch;
    DcMotor hang = initializeMechanisms.hang;
    DcMotor bucketSlides = initializeMechanisms.bucketSlides;
    Servo clawPitchLeft = initializeMechanisms.clawPitchLeft;
    Servo clawPitchRight = initializeMechanisms.clawPitchRight;
    Servo clawFingers = initializeMechanisms.clawFingers;
    Servo clawWrist = initializeMechanisms.clawWrist;
    Servo bucket = initializeMechanisms.bucket;
    public double extendoTarget;
    public double extendoPitchTarget;
    public double clawPitchPosition;
    public double clawFingerPosition;
    public double clawWristPosition;
    public double bucketSlidesTarget;
    public double bucketPosition;
    public double hangTarget;

<<<<<<< Updated upstream
    public class CustomActions{
        DcMotor extendo = hardwareMap.dcMotor.get("extendo");
        DcMotor extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        DcMotor hang = hardwareMap.dcMotor.get("hang");
        DcMotor bucketSlides = hardwareMap.dcMotor.get("bucketSlides");

        Servo clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        Servo clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        Servo clawFingers = hardwareMap.servo.get("clawFingers");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo bucket = hardwareMap.servo.get("bucket");
        public class Initialize implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                extendo.setDirection(DcMotorSimple.Direction.REVERSE);

                extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                bucketSlides.setDirection(DcMotor.Direction.REVERSE);
                bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                clawPitchLeft.setDirection(Servo.Direction.REVERSE);

                hangTarget = 0;
                extendoPitchTarget = 0;
                extendoTarget = 0;
                bucketSlidesTarget = 0;
                clawWristPosition = 76.5;
                clawFingerPosition = 50;
                clawPitchPosition = 104; //[change] best start pos for pitch is 104. This is what is used when scoring specimen
                bucketPosition = 85;
                return false;
            }
        }
        public Action initialize(){return new Initialize();}
        public class SetPositions implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){

                extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * PIDCoefficients.kP);
                extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * PIDCoefficients.kP);
                hang.setPower((hangTarget - hang.getCurrentPosition()) * PIDCoefficients.kP);
                bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * PIDCoefficients.kP);
                clawPitchLeft.setPosition(clawPitchPosition/270);
                clawPitchRight.setPosition(clawPitchPosition/270);
                bucket.setPosition(bucketPosition/270);
                clawFingers.setPosition(clawFingerPosition/180);
                clawWrist.setPosition(clawWristPosition/180);
                return true;
            }
        }
        public Action setPositions(){return new SetPositions();}
        public class SetExtendoTarget implements Action{
            private double target;
            public SetExtendoTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                extendoTarget = target;
                return false;
            }
        }
        public Action setExtendoTarget(double target){return new SetExtendoTarget(target);}
        public class SetExtendoPitchTarget implements Action{
            private double target;
            public SetExtendoPitchTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                extendoPitchTarget = target;
                return false;
            }
        }
        public Action setExtendoPitchTarget(double target){return new SetExtendoPitchTarget(target);}
        public class SetBucketSlidesTarget implements Action{
            private double target;
            public SetBucketSlidesTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                bucketSlidesTarget = target;
                return false;
            }
        }
        public Action setBucketSlidesTarget(double target){return new SetBucketSlidesTarget(target);}
        public class SetHangTarget implements Action{
            private double target;
            public SetHangTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                hangTarget = target;
                return false;
            }
        }
        public Action setHangTarget(double target){return new SetHangTarget(target);}
        public class SetClawPitchPosition implements Action{
            private double position;
            public SetClawPitchPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawPitchPosition = position;
                return false;
            }
        }
        public Action setClawPitchPosition(double position){return new SetHangTarget(position);}
        public class SetClawFingerPosition implements Action{
            private double position;
            public SetClawFingerPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawFingerPosition = position;
                return false;
            }
        }
        public Action setClawFingerPosition(double position){return new SetHangTarget(position);}
        public class SetClawWristPosition implements Action{
            private double position;
            public SetClawWristPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawWristPosition = position;
                return false;
            }
        }
        public Action setClawWristPosition(double position){return new SetHangTarget(position);}
        public class SetBucketPosition implements Action{
            private double position;
            public SetBucketPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                bucketPosition = position;
                return false;
            }
        }
        public Action setBucketPosition(double position){return new SetHangTarget(position);}
=======
    public class GlobalPID implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * PIDCoefficients.kP);
            extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * PIDCoefficients.kP);
            hang.setPower((hangTarget - hang.getCurrentPosition()) * PIDCoefficients.kP);
            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * PIDCoefficients.kP);
            clawPitchLeft.setPosition(clawPitchPosition / 270);
            clawPitchRight.setPosition(clawPitchPosition / 270);
            bucket.setPosition(bucketPosition / 270);
            clawFingers.setPosition(clawFingerPosition / 270);
            clawWrist.setPosition(clawWristPosition / 180);
            return true;
        }
>>>>>>> Stashed changes
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
        return new SetHangTarget(position);
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
        return new SetHangTarget(position);
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
        return new SetHangTarget(position);
    }

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-42, -62.5, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-42, -62.5, Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9, -58), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket2 = drive.actionBuilder(new Pose2d(-9, -58, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48, -53), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket3 = drive.actionBuilder(new Pose2d(-48, -53, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-57, -50), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-57, -50, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61, -50), Math.toRadians(285))
                .build();
        Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-61, -50, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-35, -6), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-23.4, -6), Math.toRadians(180))
                .build();


        OnePlusThreeBucket onePlusThreeBucket = new OnePlusThreeBucket();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        onePlusThreeBucket.GlobalPID(),
                        new SequentialAction(
                                // Close claw
                                onePlusThreeBucket.setClawFingerPosition(0),
                                // Drive and prepare extendo
                                new ParallelAction(
                                        // Drive to scoring pos
                                        onePlusThreeBucket1,
                                        // Raise extendo pitch and extendo
                                        new SequentialAction(
<<<<<<< Updated upstream
                                                customActions.setBucketPosition(205), //[change] move bucket to avoid linkage crashing into bucket
                                                customActions.setExtendoPitchTarget(450),
=======
                                                onePlusThreeBucket.setExtendoPitchTarget(450),
>>>>>>> Stashed changes
                                                new SleepAction(0.5),
                                                onePlusThreeBucket.setExtendoTarget(500)
                                        )
                                ),
                                // Lower extendo pitch
                                onePlusThreeBucket.setExtendoPitchTarget(760),
                                new SleepAction(1.5),
                                // Open claw
                                onePlusThreeBucket.setClawFingerPosition(50),
                                // Retract extendo to default position
                                onePlusThreeBucket.setExtendoTarget(0),
                                new SleepAction(0.5),
                                // Extendo pitch default position
<<<<<<< Updated upstream
                                customActions.setExtendoPitchTarget(0), // why not remove this line and go directly to 1421 pickup position while driving to sample zone?
=======
                                onePlusThreeBucket.setExtendoPitchTarget(0),
>>>>>>> Stashed changes
                                // Drive to sample zone 1 and lower extendo pitch when driving
                                new ParallelAction(
                                        // Drive to sample zone 1
                                        onePlusThreeBucket2,
                                        // Extendo pitch pickup position
                                        onePlusThreeBucket.setExtendoPitchTarget(1421)
                                ),
                                customActions.setBucketPosition(85), //[change] reset bucket after it was moved to avoid linkage crashing into bucket
                                /** CHANGE LATER **/
                                onePlusThreeBucket.setExtendoTarget(250),
                                // Claw pitch picking up pos
                                onePlusThreeBucket.setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close Claw
                                onePlusThreeBucket.setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket3,
                                        new SequentialAction(
                                                // Retract extendo
<<<<<<< Updated upstream
                                                customActions.setClawPitchPosition(195), //[change]195 is new pitch position for transfer
                                                customActions.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                new SleepAction(0.3), //[change] give extendo time to retract before moving extendo pitch
=======
                                                onePlusThreeBucket.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                onePlusThreeBucket.setClawPitchPosition(200),
>>>>>>> Stashed changes
                                                // Extendo pitch transfer position
                                                onePlusThreeBucket.setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
<<<<<<< Updated upstream
                                                // Open claw fully bc bucketslides coming down later
                                                customActions.setClawFingerPosition(80) //[change]for some reason Tristan wants 80 instead of 100
=======
                                                // Open claw fully bc bucketSlides coming down later
                                                onePlusThreeBucket.setClawFingerPosition(100)
>>>>>>> Stashed changes
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketSlides up to scoring position
                                onePlusThreeBucket.setBucketSlidesTarget(1100),
                                // Rotate bucket to score
<<<<<<< Updated upstream
                                new SleepAction(0.4), //[change] give bucket slides time to get to target before rotating to avoid dropping sample
                                customActions.setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                customActions.setBucketPosition(85),
                                new SleepAction(0.4), //[change] give servo time to rotate to avoid lvl 4 hang
                                // Move bucketslides back to down position
                                customActions.setBucketSlidesTarget(0),
                                // Drive to sample zone 2, while driving lower extendopitch
=======
                                onePlusThreeBucket.setBucketPosition(190),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                onePlusThreeBucket.setBucketPosition(81.51),
                                // Move bucketSlides back to down position
                                onePlusThreeBucket.setBucketSlidesTarget(0),
                                // Drive to sample zone 2, while driving lower extendoPitch
>>>>>>> Stashed changes
                                new ParallelAction(
                                        // Drive to sample zone 2
                                        onePlusThreeBucket4,
                                        // Lower extendo pitch to pickup pos
                                        onePlusThreeBucket.setExtendoPitchTarget(1421)
                                ),
                                /** CHANGE LATER **/
                                onePlusThreeBucket.setExtendoTarget(250),
                                // Claw pitch picking up position
                                onePlusThreeBucket.setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close claw
                                onePlusThreeBucket.setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket5,
                                        new SequentialAction(
                                                // Retract extendo
                                                onePlusThreeBucket.setExtendoTarget(0),
                                                // Claw pitch transfer position
<<<<<<< Updated upstream
                                                customActions.setClawPitchPosition(195),
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3), //[change] give extendo time to retract before moving extendo pitch
                                                customActions.setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketslides coming down later
                                                customActions.setClawFingerPosition(80)
=======
                                                onePlusThreeBucket.setClawPitchPosition(200),
                                                // Extendo pitch transfer position
                                                onePlusThreeBucket.setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketSlides coming down later
                                                onePlusThreeBucket.setClawFingerPosition(100)
>>>>>>> Stashed changes
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
<<<<<<< Updated upstream
                                // Move bucketslides up to scoring position
                                customActions.setBucketSlidesTarget(1100),
                                new SleepAction(0.4), //[change] give bucket slides time to get to target before rotating to avoid dropping sample
                                // Rotate bucket to score
                                customActions.setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                customActions.setBucketPosition(85),
                                new SleepAction(0.4), //[change] give servo time to rotate to avoid lvl 4 hang
                                // Move bucketslides back to down position
                                customActions.setBucketSlidesTarget(0),
                                // Drive to sample zone 3, while driving lower extendopitch
=======
                                // Move bucketSlides up to scoring position
                                onePlusThreeBucket.setBucketSlidesTarget(1100),
                                // Rotate bucket to score
                                onePlusThreeBucket.setBucketPosition(190),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                onePlusThreeBucket.setBucketPosition(81.51),
                                // Move bucketSlides back to down position
                                onePlusThreeBucket.setBucketSlidesTarget(0),
                                // Drive to sample zone 3, while driving lower extendoPitch
>>>>>>> Stashed changes
                                new ParallelAction(
                                        // Drive to sample zone 3
                                        onePlusThreeBucket6,
                                        // Lower extendo pitch to pickup pos
                                        onePlusThreeBucket.setExtendoPitchTarget(1421)
                                ),
                                /** CHANGE LATER **/
                                onePlusThreeBucket.setExtendoTarget(250),
                                // Claw pitch picking up position
                                onePlusThreeBucket.setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close claw
                                onePlusThreeBucket.setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket7,
                                        new SequentialAction(
                                                // Retract extendo
                                                onePlusThreeBucket.setExtendoTarget(0),
                                                // Claw pitch transfer position
<<<<<<< Updated upstream
                                                customActions.setClawPitchPosition(195),
                                                new SleepAction(0.3), //[change] give extendo time to retract before moving extendo pitch
=======
                                                onePlusThreeBucket.setClawPitchPosition(200),
>>>>>>> Stashed changes
                                                // Extendo pitch transfer position
                                                onePlusThreeBucket.setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
<<<<<<< Updated upstream
                                                // Open claw fully bc bucketslides coming down later
                                                customActions.setClawFingerPosition(80)
=======
                                                // Open claw fully bc bucketSlides coming down later
                                                onePlusThreeBucket.setClawFingerPosition(100)
>>>>>>> Stashed changes
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
<<<<<<< Updated upstream
                                // Move bucketslides up to scoring position
                                customActions.setBucketSlidesTarget(1100),
                                new SleepAction(0.4), //[change] give bucket slides time to get to target before rotating to avoid dropping sample
                                // Rotate bucket to score
                                customActions.setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                customActions.setBucketPosition(85),
                                new SleepAction(0.4), //[change] give servo time to rotate to avoid lvl 4 hang
                                // Move bucketslides back to down position
                                customActions.setBucketSlidesTarget(0),
=======
                                // Move bucketSlides up to scoring position
                                onePlusThreeBucket.setBucketSlidesTarget(1100),
                                // Rotate bucket to score
                                onePlusThreeBucket.setBucketPosition(190),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                onePlusThreeBucket.setBucketPosition(81.51),
                                // Move bucketSlides back to down position
                                onePlusThreeBucket.setBucketSlidesTarget(0),
>>>>>>> Stashed changes
                                // Park
                                onePlusThreeBucket8
                                /** ADD ACTUAL LEVEL 1 ASCENT**/
                        )
                )
        );
    }
}
