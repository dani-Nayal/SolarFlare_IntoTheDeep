package org.firstinspires.ftc.teamcode.base.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.json.JSONException;

public class IMUConfig{
    public String              deviceName;
    public IMU                 imu;
    public IMU.Parameters      parameters;
    public LogoFacingDirection logoDirection;
    public UsbFacingDirection  usbDirection;

    public IMUConfig(HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        this.deviceName    = robotConfig.getIMUString("deviceName");
        this.imu           = hardwareMap.get(IMU.class, this.deviceName);
        this.logoDirection = robotConfig.getIMULogoFacingDirection();
        this.usbDirection  = robotConfig.getIMULUSBFacingDirection();
        this.parameters    = new IMU.Parameters(new RevHubOrientationOnRobot(
                this.logoDirection,
                this.usbDirection));
        imu.initialize(this.parameters);
    }
}
