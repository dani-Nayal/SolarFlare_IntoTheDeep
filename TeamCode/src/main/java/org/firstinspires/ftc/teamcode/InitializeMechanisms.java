package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class InitializeMechanisms {
    public DcMotor extendo;
    public DcMotor extendoPitch;
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor hang;
    public DcMotor bucketSlides;
    public Servo clawPitchLeft;
    public Servo clawPitchRight;
    public Servo clawFingers;
    public Servo clawWrist;
    public Servo bucket;
    public InitializeMechanisms(HardwareMap hardwareMap){
        this.extendo = hardwareMap.dcMotor.get("extendo");
        this.extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        this.frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        this.backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        this.frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        this.backRightMotor = hardwareMap.dcMotor.get("rightBack");
        this.bucketSlides = hardwareMap.dcMotor.get("bucketSlides");
        this.hang = hardwareMap.dcMotor.get("hang");


        this.clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        this.clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        this.clawFingers = hardwareMap.servo.get("clawFingers");
        this.clawWrist = hardwareMap.servo.get("clawWrist");
        this.bucket = hardwareMap.servo.get("bucket");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketSlides.setDirection(DcMotor.Direction.REVERSE);
        bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawPitchLeft.setDirection(Servo.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        pinpoint.resetPosAndIMU();
    }
}
