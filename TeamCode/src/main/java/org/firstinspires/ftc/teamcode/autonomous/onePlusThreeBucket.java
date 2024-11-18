package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "onePlusThreeBucket", group = "Autonomous")
public class onePlusThreeBucket extends LinearOpMode{
    public class Extendo{
        private DcMotor extendo;

        public Extendo(HardwareMap hardwareMap){
            extendo = hardwareMap.dcMotor.get("extendo");
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    public class ExtendoPitch{
        private DcMotor extendoPitch;

        public ExtendoPitch(HardwareMap hardwareMap){
            extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
            extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public class Hang{
        private DcMotor hang;

        public Hang(HardwareMap hardwareMap){
            hang = hardwareMap.dcMotor.get("hang");
            hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public class BucketSlides{
        private DcMotor bucketSlides;

        public BucketSlides(HardwareMap hardwareMap){
            bucketSlides = hardwareMap.dcMotor.get("bucketSlides");
            bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bucketSlides.setDirection(DcMotor.Direction.REVERSE);
            bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public class ClawPitch{
        private Servo clawPitchLeft;
        private Servo clawPitchRight;

        public ClawPitch(HardwareMap hardwareMap){
            clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
            clawPitchRight = hardwareMap.servo.get("clawPitchRight");
            clawPitchLeft.setDirection(Servo.Direction.REVERSE);
        }
    }
    public class ClawFingers{
        private Servo clawFingers;

        public ClawFingers(HardwareMap hardwareMap){
            clawFingers = hardwareMap.servo.get("clawFingers");
        }
    }
    public class ClawWrist{
        private Servo clawWrist;

        public ClawWrist(HardwareMap hardwareMap){
            clawWrist = hardwareMap.servo.get("clawWrist");
        }
    }
    public class Bucket{
        private Servo bucket;

        public Bucket(HardwareMap hardwareMap){
            bucket = hardwareMap.servo.get("bucket");
        }
    }

    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-42,-62.5,Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        // Motors
        Extendo extendo = new Extendo(hardwareMap);
        ExtendoPitch extendoPitch = new ExtendoPitch(hardwareMap);
        Hang hang = new Hang(hardwareMap);
        BucketSlides bucketSlides = new BucketSlides(hardwareMap);
        ClawPitch clawPitch = new ClawPitch(hardwareMap);

        // Servos
        ClawFingers clawFingers = new ClawFingers(hardwareMap);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);

        Action onePlusThreeBucket = drive.actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(90)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9,-58), Math.toRadians(90))
                .waitSeconds(1.5)
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-53), Math.toRadians(90))
                .waitSeconds(1.5)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45))
                .waitSeconds(2)
                // Go to sample zone 2
                .strafeToLinearHeading(new Vector2d(-57,-50), Math.toRadians(90))
                .waitSeconds(1.5)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45))
                .waitSeconds(2)
                // Turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61,-50), Math.toRadians(105))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45))
                .waitSeconds(2)
                // Park
                .strafeToLinearHeading(new Vector2d(-30,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-23.4,-6), Math.toRadians(0))
                .waitSeconds(1)
                .build();
        
        waitForStart();

        Actions.runBlocking(
                onePlusThreeBucket
        );

    }
}
