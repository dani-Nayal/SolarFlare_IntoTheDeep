package org.firstinspires.ftc.teamcode.base.config;

import java.util.EnumMap;
import org.json.JSONException;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareConfig {
    private static HardwareConfig                 hardwareConfig;
    public  final  RobotDimensions                robotDimensions;
    private final  EnumMap<MotorEnum,MotorConfig> motorConfigs;
    private final  EnumMap<ServoEnum,ServoConfig> servoConfigs;
    public  final  IMUConfig                      imuConfig;
    public  final  PinpointConfig                 pinpointConfig;
    public  final  LimelightConfig                limelightConfig;

    private HardwareConfig(HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        robotDimensions   = robotConfig.getRobotDimensions();
        motorConfigs      = new EnumMap<>(MotorEnum.class);
        servoConfigs      = new EnumMap<>(ServoEnum.class);
        for(MotorEnum motorEnum: robotConfig.getMotorEnums()) {
            motorConfigs.put(motorEnum, new MotorConfig(motorEnum, hardwareMap, robotConfig));
        }
        for(ServoEnum servoEnum: robotConfig.getServoEnums()) {
            servoConfigs.put(servoEnum, new ServoConfig(servoEnum, hardwareMap, robotConfig));
        }

        imuConfig         = new IMUConfig(hardwareMap,       robotConfig);
        pinpointConfig    = new PinpointConfig(hardwareMap,  robotConfig);
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

    public LimelightConfig getLimelightConfig(){
        if(limelightConfig == null)
            throw new IllegalStateException("Limelight3aConfig not initialized");
    return limelightConfig;}
}
