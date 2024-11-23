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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PIDCoefficients;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@Autonomous(name = "onePlusThreeBucket", group = "Autonomous")
public class onePlusThreeBucket extends LinearOpMode{
    public double extendoTarget;
    public double extendoPitchTarget;
    public double clawPitchPosition;
    public double clawFingerPosition;
    public double clawWristPosition;
    public double bucketSlidesTarget;
    public double bucketPosition;
    public double hangTarget;

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
    }
    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-42,-62.5,Math.toRadians(270));
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


        CustomActions customActions = new CustomActions();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        customActions.initialize(),
                        new ParallelAction(
                        customActions.setPositions(),
                        new SequentialAction(
                                // Close claw
                                customActions.setClawFingerPosition(0),
                                // Drive and prepare extendo
                                new ParallelAction(
                                        // Drive to scoring pos
                                        onePlusThreeBucket1,
                                        // Raise extendo pitch and extendo
                                        new SequentialAction(
                                                customActions.setBucketPosition(205), //[change] move bucket to avoid linkage crashing into bucket
                                                customActions.setExtendoPitchTarget(450),
                                                new SleepAction(0.5),
                                                customActions.setExtendoTarget(500)
                                        )
                                ),
                                // Lower extendo pitch
                                customActions.setExtendoPitchTarget(760),
                                new SleepAction(1.5),
                                // Open claw
                                customActions.setClawFingerPosition(50),
                                // Retract extendo to default position
                                customActions.setExtendoTarget(0),
                                new SleepAction(0.5),
                                // Extendo pitch default position
                                customActions.setExtendoPitchTarget(0), // why not remove this line and go directly to 1421 pickup position while driving to sample zone?
                                // Drive to sample zone 1 and lower extendo pitch when driving
                                new ParallelAction(
                                        // Drive to sample zone 1
                                        onePlusThreeBucket2,
                                        // Extendo pitch pickup position
                                        customActions.setExtendoPitchTarget(1421)
                                ),
                                customActions.setBucketPosition(85), //[change] reset bucket after it was moved to avoid linkage crashing into bucket
                                /** CHANGE LATER **/
                                customActions.setExtendoTarget(250),
                                // Claw pitch picking up pos
                                customActions.setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close Claw
                                customActions.setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket3,
                                        new SequentialAction(
                                                // Retract extendo
                                                customActions.setClawPitchPosition(195), //[change]195 is new pitch position for transfer
                                                customActions.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                new SleepAction(0.3), //[change] give extendo time to retract before moving extendo pitch
                                                // Extendo pitch transfer position
                                                customActions.setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketslides coming down later
                                                customActions.setClawFingerPosition(80) //[change]for some reason Tristan wants 80 instead of 100
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
                                // Move bucketslides up to scoring position
                                customActions.setBucketSlidesTarget(1100),
                                // Rotate bucket to score
                                new SleepAction(0.4), //[change] give bucket slides time to get to target before rotating to avoid dropping sample
                                customActions.setBucketPosition(205),
                                new SleepAction(0.7),
                                // Move bucket back to default position
                                customActions.setBucketPosition(85),
                                new SleepAction(0.4), //[change] give servo time to rotate to avoid lvl 4 hang
                                // Move bucketslides back to down position
                                customActions.setBucketSlidesTarget(0),
                                // Drive to sample zone 2, while driving lower extendopitch
                                new ParallelAction(
                                        // Drive to sample zone 2
                                        onePlusThreeBucket4,
                                        // Lower extendo pitch to pickup pos
                                        customActions.setExtendoPitchTarget(1421)
                                ),
                                /** CHANGE LATER **/
                                customActions.setExtendoTarget(250),
                                // Claw pitch picking up position
                                customActions.setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close claw
                                customActions.setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket5,
                                        new SequentialAction(
                                                // Retract extendo
                                                customActions.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                customActions.setClawPitchPosition(195),
                                                // Extendo pitch transfer position
                                                new SleepAction(0.3), //[change] give extendo time to retract before moving extendo pitch
                                                customActions.setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketslides coming down later
                                                customActions.setClawFingerPosition(80)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
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
                                new ParallelAction(
                                        // Drive to sample zone 3
                                        onePlusThreeBucket6,
                                        // Lower extendo pitch to pickup pos
                                        customActions.setExtendoPitchTarget(1421)
                                ),
                                /** CHANGE LATER **/
                                customActions.setExtendoTarget(250),
                                // Claw pitch picking up position
                                customActions.setClawPitchPosition(30.5),
                                new SleepAction(0.7),
                                // Close claw
                                customActions.setClawFingerPosition(0),
                                // Retract extendo, transfer and move to scoring pos
                                new ParallelAction(
                                        // Move to scoring position
                                        onePlusThreeBucket7,
                                        new SequentialAction(
                                                // Retract extendo
                                                customActions.setExtendoTarget(0),
                                                // Claw pitch transfer position
                                                customActions.setClawPitchPosition(195),
                                                new SleepAction(0.3), //[change] give extendo time to retract before moving extendo pitch
                                                // Extendo pitch transfer position
                                                customActions.setExtendoPitchTarget(0),
                                                new SleepAction(1.5),
                                                // Open claw fully bc bucketslides coming down later
                                                customActions.setClawFingerPosition(80)
                                        )
                                ),
                                // Wait for sample to settle in bucket
                                new SleepAction(0.5),
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
                                // Park
                                onePlusThreeBucket8
                        )
                )
                )

        );
    }
}
