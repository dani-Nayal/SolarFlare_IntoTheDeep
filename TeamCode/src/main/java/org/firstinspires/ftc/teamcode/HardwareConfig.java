package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.PinpointEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

public class HardwareConfig {
    public MotorConfig extendo;
    public MotorConfig extendoPitch;
    public MotorConfig frontLeftMotor;
    public MotorConfig backLeftMotor;
    public MotorConfig frontRightMotor;
    public MotorConfig backRightMotor;
    public MotorConfig hang;
    public MotorConfig bucketSlides;
    public ServoConfig clawPitchLeft;
    public ServoConfig clawPitchRight;
    public ServoConfig clawFingers;
    public ServoConfig clawWrist;
    public ServoConfig bucket;
    public IMUConfig imu;
    public PinpointConfig pinpoint;

    public HardwareConfig(HardwareMap hardwareMap){
        this.extendo = new MotorConfig(
                hardwareMap,
                "extendo",
                0.015,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE");
        this.extendoPitch = new MotorConfig(
                hardwareMap,
                "extendoPitch",
                0.015,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE");
        this.frontLeftMotor = new MotorConfig(
                hardwareMap,
                "leftFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE");
        this.backLeftMotor = new MotorConfig(
                hardwareMap,
                "leftFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE");
        this.frontRightMotor = new MotorConfig(
                hardwareMap,
                "leftFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE");
        this.backRightMotor = new MotorConfig(
                hardwareMap,
                "leftFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE");
        this.hang = new MotorConfig(
                hardwareMap,
                "leftFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE");
        this.bucketSlides = new MotorConfig(
                hardwareMap,
                "leftFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE");
        this.clawPitchLeft = new ServoConfig(
                hardwareMap,
                "clawPitchLeft",
                "REVERSE");
        this.clawPitchRight = new ServoConfig(
                hardwareMap,
                "clawPitchLeft",
                "FORWARD");
        this.clawFingers = new ServoConfig(
                hardwareMap,
                "clawPitchLeft",
                "FORWARD");
        this.clawWrist = new ServoConfig(
                hardwareMap,
                "clawPitchLeft",
                "FORWARD");
        this.bucket = new ServoConfig(
                hardwareMap,
                "clawPitchLeft",
                "FORWARD");
        this.imu = new IMUConfig(
                hardwareMap,
                "imu",
                "LEFT",
                "UP");
        this.pinpoint = new PinpointConfig(
                hardwareMap,
                "pinpoint");
    }
    public class MotorConfig{
        public DcMotor motor;
        public String deviceName;
        public double kP;
        public double kI;
        public double kD;
        public DcMotor.RunMode runMode;
        public DcMotorSimple.Direction direction;
        public DcMotor.ZeroPowerBehavior zeroPowerBehaviour;
        public MotorConfig(HardwareMap hardwareMap, String deviceName, double kP, double kI, double kD, String runMode, String direction, String zeroPowerBehaviour){
            this.motor = hardwareMap.dcMotor.get(deviceName);
            this.deviceName = deviceName;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            this.runMode = DcMotor.RunMode.valueOf(runMode);
            this.direction = DcMotorSimple.Direction.valueOf(direction);
            this.zeroPowerBehaviour = DcMotor.ZeroPowerBehavior.valueOf(zeroPowerBehaviour);
            motor.setMode(this.runMode);
            motor.setDirection(this.direction);
            motor.setZeroPowerBehavior(this.zeroPowerBehaviour);

        }

    }
    public class ServoConfig{
        public Servo servo;
        public String deviceName;
        public Servo.Direction direction;
        public ServoConfig(HardwareMap hardwareMap, String deviceName, String direction){
            this.servo = hardwareMap.servo.get(deviceName);
            this.deviceName = deviceName;
            this.direction = Servo.Direction.valueOf(direction);
            servo.setDirection(this.direction);
        }
    }
    public class IMUConfig{
        public IMU imu;
        public String deviceName;
        public RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
        public RevHubOrientationOnRobot.UsbFacingDirection usbDirection;
        public IMUConfig(HardwareMap hardwareMap, String deviceName, String logoDirection, String usbDirection){
            this.imu = hardwareMap.get(IMU.class, deviceName);
            this.logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.valueOf(logoDirection);
            this.usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.valueOf(usbDirection);

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    this.logoDirection,
                    this.usbDirection));
            imu.initialize(parameters);
        }
    }
    public class PinpointConfig{
        public GoBildaPinpointDriverRR pinpoint;
        public String deviceName;
        public PinpointConfig(HardwareMap hardwareMap, String deviceName){
            pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, deviceName);
        }

    }

}
