package org.firstinspires.ftc.teamcode.base.config;

import static java.util.logging.Level.INFO;
import java.util.EnumMap;
import java.util.logging.Logger;

import org.json.JSONException;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareConfig {
    private static HardwareConfig                 hardwareConfig;

    private final  Logger                         logger;
    public  final  RobotDimensions                robotDimensions;
    private final  EnumMap<MotorEnum,MotorConfig> motorConfigs;
    private final  EnumMap<ServoEnum,ServoConfig> servoConfigs;
    public  final  IMUConfig                      imuConfig;
    public  final  PinpointConfig                 pinpointConfig;
    public  final  LimelightConfig                limelightConfig;

    private HardwareConfig(HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        logger            = RobotLogger.getInstance().getConfigLogger();
        robotDimensions   = robotConfig.getRobotDimensions();
        motorConfigs      = new EnumMap<>(MotorEnum.class);
        servoConfigs      = new EnumMap<>(ServoEnum.class);
        for(MotorEnum motorEnum: robotConfig.getMotorEnums()) {
            logger.logp(INFO,
                    "HardwareConfig",
                    "MotorConfig",
                    "Initializing MotorConfig: " + motorEnum);
            motorConfigs.put(motorEnum, new MotorConfig(motorEnum, hardwareMap, robotConfig));
        }
        for(ServoEnum servoEnum: robotConfig.getServoEnums()) {
            logger.logp(INFO,
                    "HardwareConfig",
                    "MotorConfig",
                    "Initializing ServoConfig: " + servoEnum);
            servoConfigs.put(servoEnum, new ServoConfig(servoEnum, hardwareMap, robotConfig));
        }

        logger.logp(INFO,
                "HardwareConfig",
                "MotorConfig",
                "Initializing IMUConfig");
        imuConfig         = new IMUConfig(hardwareMap,       robotConfig);
        logger.logp(INFO,
                "HardwareConfig",
                "MotorConfig",
                "Initializing IMUConfig");
        pinpointConfig    = new PinpointConfig(hardwareMap,  robotConfig);
        logger.logp(INFO,
                "HardwareConfig",
                "MotorConfig",
                "Initializing LimelightConfig");
        limelightConfig   = new LimelightConfig(hardwareMap, robotConfig);
    }

    public static HardwareConfig createInstance(HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        hardwareConfig = new HardwareConfig(hardwareMap, robotConfig);
        return hardwareConfig;
    }

    public static HardwareConfig getInstance() {
        if(hardwareConfig == null)
            throw new IllegalStateException("HardwareConfig has not been initialized");
        return hardwareConfig;
    }

    public MotorConfig getMotorConfig(MotorEnum motorEnum) throws IllegalArgumentException {
        logger.entering("HardwareConfig", "getMotorConfig", motorEnum);
        MotorConfig motorConfig = motorConfigs.get(motorEnum);
        if (motorConfig == null){
            throw new IllegalArgumentException("Config is not present for " + motorEnum.name());
        }
        logger.exiting("HardwareConfig", "getMotorConfig", motorConfig);
        return motorConfig;
    }

    public ServoConfig getServoConfig(ServoEnum servoEnum) throws IllegalArgumentException {
        logger.entering("HardwareConfig", "getServoConfig", servoEnum);
        ServoConfig servoConfig = servoConfigs.get(servoEnum);
        if (servoConfig == null){
            throw new IllegalArgumentException("Config is not present for " + servoEnum.name());
        }
        logger.exiting("HardwareConfig", "getServoConfig", servoConfig);
        return servoConfig;
    }

    public PinpointConfig getPinpointConfig() {
        logger.entering("HardwareConfig", "getPinpointConfig");
        if(pinpointConfig == null)
            throw new IllegalStateException("PinpointConfig not initialized");
        logger.exiting("HardwareConfig", "getPinpointConfig", pinpointConfig);
        return pinpointConfig;
    }

    public IMUConfig getImuConfig() {
        logger.entering("HardwareConfig", "getImuConfig");
        if(imuConfig == null)
            throw new IllegalStateException("PinpointConfig not initialized");
        logger.exiting("HardwareConfig", "getImuConfig", imuConfig);
        return imuConfig;
    }

    public LimelightConfig getLimelightConfig() {
        logger.entering("HardwareConfig", "getLimelightConfig");
        if(limelightConfig == null)
            throw new IllegalStateException("Limelight3aConfig not initialized");
        logger.exiting("HardwareConfig", "getLimelightConfig", limelightConfig);
    return limelightConfig;}
}
