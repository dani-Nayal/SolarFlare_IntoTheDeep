package org.firstinspires.ftc.teamcode.base.config;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class HardwareConfig {
    // TODO: Measure robot length and width
    public final double ROBOT_LENGTH = 15.0625;
    public final double ROBOT_WIDTH = 12.4375;
    private static HardwareConfig hardwareConfig;
    private final HashMap<MotorEnum,MotorConfig> motorConfigs;
    private final HashMap<ServoEnum,ServoConfig> servoConfigs;
    public IMUConfig imuConfig;
    public PinpointConfig pinpointConfig;
    public Limelight3aConfig limelight3aConfig;

    private HardwareConfig(HardwareMap hardwareMap){
        motorConfigs = new HashMap<>(8);
        servoConfigs = new HashMap<>(12);

        // TODO: Assign config variables here and determine their parameters

        motorConfigs.put(MotorEnum.TESTING_MOTOR, new MotorConfig(
                hardwareMap,
                "motor-1",
                0.0427,0,0.00043,
                "RUN_WITHOUT_ENCODER",
                "FORWARD",
                "BRAKE",
                0,
                10000,
                4000,
                2960));

        pinpointConfig    = new PinpointConfig(hardwareMap, "pinpoint");
        limelight3aConfig = new Limelight3aConfig(hardwareMap, "limelight", 11);
    }

    public static HardwareConfig createInstance(HardwareMap hardwareMap) {
        hardwareConfig = new HardwareConfig(hardwareMap);
        return hardwareConfig;
    }

    public static HardwareConfig getInstance() {
        if(hardwareConfig == null)
            throw new IllegalStateException("HardwareConfig has not been initialized");
        return hardwareConfig;
    }

    public MotorConfig getMotorConfig(MotorEnum motorEnum) throws IllegalArgumentException{
        MotorConfig motorConfig = motorConfigs.get(motorEnum);
        if (motorConfig == null){
            throw new IllegalArgumentException("Config is not present for " + motorEnum.name());
        }
        return motorConfig;
    }

    public ServoConfig getServoConfig(ServoEnum servoEnum) throws IllegalArgumentException{
        ServoConfig servoConfig = servoConfigs.get(servoEnum);
        if (servoConfig == null){
            throw new IllegalArgumentException("Config is not present for " + servoEnum.name());
        }
        return servoConfig;
    }

    public PinpointConfig getPinpointConfig() {
        if(pinpointConfig == null)
            throw new IllegalStateException("PinpointConfig not initialized");
        return pinpointConfig;
    }

    public IMUConfig getImuConfig() {
        if(imuConfig == null)
            throw new IllegalStateException("PinpointConfig not initialized");
        return imuConfig;
    }

    public Limelight3aConfig getLimelightConfig(){
        if(limelight3aConfig == null)
            throw new IllegalStateException("Limelight3aConfig not initialized");
    return limelight3aConfig;}

    public static class MotorConfig{
        public DcMotorEx motor;
        public double kP;
        public double kI;
        public double kD;
        public RunMode runMode;
        public DcMotorSimple.Direction direction;
        public ZeroPowerBehavior zeroPowerBehaviour;
        public int minTarget;
        public int maxTarget;
        public double maxAcceleration;
        public double maxVelocity;
        public MotorConfig(HardwareMap hardwareMap, String deviceName, double kP, double kI, double kD, String runMode, String direction, String zeroPowerBehaviour, int minTarget, int maxTarget, double maxVelocity, double maxAcceleration){
            this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            this.runMode = RunMode.valueOf(runMode);
            this.direction = DcMotorSimple.Direction.valueOf(direction);
            this.zeroPowerBehaviour = ZeroPowerBehavior.valueOf(zeroPowerBehaviour);
            motor.setMode(this.runMode);
            motor.setDirection(this.direction);
            motor.setZeroPowerBehavior(this.zeroPowerBehaviour);
            this.minTarget = minTarget;
            this.maxTarget = maxTarget;
            this.maxAcceleration = maxAcceleration;
            this.maxVelocity = maxVelocity;
        }
    }
    public static class ServoConfig{
        public Servo servo;
        public Servo.Direction direction;
        public double minServoPosition;
        public double maxServoPosition;
        public double degreesPerSecond;
        public ServoConfig(HardwareMap hardwareMap, String deviceName, String direction, double minServoPosition, double maxServoPosition, double degreesPerSecond){
            this.servo = hardwareMap.servo.get(deviceName);
            this.direction = Servo.Direction.valueOf(direction);
            servo.setDirection(this.direction);
            this.minServoPosition = minServoPosition;
            this.maxServoPosition = maxServoPosition;
            this.degreesPerSecond = degreesPerSecond;
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
    public static class Limelight3aConfig{
        public Limelight3A limelight;
        public String deviceName;
        public int pollingRate;
        public Limelight3aConfig(HardwareMap hardwareMap, String deviceName, int pollingRate){
            this.limelight = hardwareMap.get(Limelight3A.class, deviceName);
            this.deviceName = deviceName;
            this.pollingRate = pollingRate;
            limelight.setPollRateHz(pollingRate);
        }
    }
}
