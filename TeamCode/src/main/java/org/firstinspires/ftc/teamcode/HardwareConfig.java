package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import java.security.InvalidParameterException;
import java.util.HashMap;

// Initializes hardwareMap of all mechanisms, as well as other related values
public class HardwareConfig {

    private static HardwareConfig hardwareConfig;

    private final HashMap<MotorEnum,MotorConfig> motorConfigs;
    private final HashMap<ServoEnum,ServoConfig> servoConfigs;

    public IMUConfig imu;
    public PinpointConfig pinpoint;

    public static HardwareConfig getHardwareConfig() {
        if(hardwareConfig == null)
            throw new InvalidParameterException("HardwareConfig not initialized");
        return hardwareConfig;
    }
    // Gets MotorConfig from motorConfigs HashMap
    public MotorConfig getMotorConfig(MotorEnum motorEnum) throws IllegalArgumentException{
        MotorConfig motorConfig = motorConfigs.get(motorEnum);
        if (motorConfig == null){
            throw new IllegalArgumentException("Config is not present for " + motorEnum.name());
        }
        return motorConfig;
    }

    // Gets ServoConfig from servoConfigs HashMap
    public ServoConfig getServoConfig(ServoEnum servoEnum) throws IllegalArgumentException{
        ServoConfig servoConfig = servoConfigs.get(servoEnum);
        if (servoConfig == null){
            throw new IllegalArgumentException("Config is not present for " + servoEnum.name());
        }
        return servoConfig;
    }

    public static void makeHardwareConfig(HardwareMap hardwareMap){
        hardwareConfig = new HardwareConfig(hardwareMap);
    }

    private HardwareConfig(HardwareMap hardwareMap){
        motorConfigs = new HashMap<>(10);
        servoConfigs = new HashMap<>(10);

        // Initialize MotorConfigs
        motorConfigs.put(MotorEnum.EXTENDO, new MotorConfig(
                hardwareMap,
                "extendo",
                0.015,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE"));
        motorConfigs.put(MotorEnum.EXTENDO_PITCH, new MotorConfig(
                hardwareMap,
                "extendoPitch",
                0.005,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE"));
        motorConfigs.put(MotorEnum.LEFT_FRONT, new MotorConfig(
                hardwareMap,
                "leftFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE"));
        motorConfigs.put(MotorEnum.LEFT_BACK, new MotorConfig(
                hardwareMap,
                "leftBack",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE"));
        motorConfigs.put(MotorEnum.RIGHT_FRONT, new MotorConfig(
                hardwareMap,
                "rightFront",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE"));
        motorConfigs.put(MotorEnum.RIGHT_BACK, new MotorConfig(
                hardwareMap,
                "rightBack",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE"));
        motorConfigs.put(MotorEnum.HANG, new MotorConfig(
                hardwareMap,
                "hang",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE"));
        motorConfigs.put(MotorEnum.BUCKET_SLIDES, new MotorConfig(
                hardwareMap,
                "bucketSlides",
                0,0,0,
                "RUN_WITHOUT_ENCODER",
                "REVERSE",
                "BRAKE"));
        // Initialize ServoConfigs
        servoConfigs.put(ServoEnum.CLAW_PITCH_LEFT, new ServoConfig(
                hardwareMap,
                "clawPitchLeft",
                "REVERSE",
                0,
                270));
        servoConfigs.put(ServoEnum.CLAW_PITCH_RIGHT, new ServoConfig(
                hardwareMap,
                "clawPitchRight",
                "FORWARD",
                0,
                270));
        servoConfigs.put(ServoEnum.CLAW_FINGERS, new ServoConfig(
                hardwareMap,
                "clawFingers",
                "FORWARD",
                0,
                180));
        servoConfigs.put(ServoEnum.CLAW_WRIST, new ServoConfig(
                hardwareMap,
                "clawWrist",
                "FORWARD",
                0,
                180));
        servoConfigs.put(ServoEnum.BUCKET, new ServoConfig(
                hardwareMap,
                "bucket",
                "FORWARD",
                0,
                270));
        this.imu = new IMUConfig(
                hardwareMap,
                "imu",
                "LEFT",
                "UP");
        this.pinpoint = new PinpointConfig(
                hardwareMap,
                "pinpoint");
    }
    public static class MotorConfig{
        public DcMotor motor;
        public double kP;
        public double kI;
        public double kD;
        public DcMotor.RunMode runMode;
        public DcMotorSimple.Direction direction;
        public ZeroPowerBehavior zeroPowerBehaviour;
        public MotorConfig(HardwareMap hardwareMap, String deviceName, double kP, double kI, double kD, String runMode, String direction, String zeroPowerBehaviour){
            this.motor = hardwareMap.dcMotor.get(deviceName);
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
    public static class ServoConfig{
        public Servo servo;
        public Servo.Direction direction;
        public double minServoPosition;
        public double maxServoPosition;
        public ServoConfig(HardwareMap hardwareMap, String deviceName, String direction, double minServoPosition, double maxServoPosition){
            this.servo = hardwareMap.servo.get(deviceName);
            this.direction = Servo.Direction.valueOf(direction);
            servo.setDirection(this.direction);
            this.minServoPosition = minServoPosition;
            this.maxServoPosition = maxServoPosition;
        }
    }
    public static class IMUConfig{
        public IMU imu;
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
    public static class PinpointConfig{
        public GoBildaPinpointDriverRR pinpoint;
        public PinpointConfig(HardwareMap hardwareMap, String deviceName){
            pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, deviceName);

        }

    }

}
