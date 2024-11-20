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
    public double clawFingersPosition;
    public double clawWristPosition;
    public double bucketSlidesTarget;
    public double bucketTarget;

    public class CustomActions{
        DcMotor extendo = hardwareMap.dcMotor.get("extendo");
        DcMotor extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
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

                frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                bucketSlides.setDirection(DcMotor.Direction.REVERSE);
                bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                clawPitchLeft.setDirection(Servo.Direction.REVERSE);
                return false;
            }
        }
        public class SetPositions implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){

                return false;
            }
        }






    }




    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-42,-62.5,Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

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
