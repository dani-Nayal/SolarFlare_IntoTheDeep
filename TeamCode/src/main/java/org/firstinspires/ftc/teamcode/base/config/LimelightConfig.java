package org.firstinspires.ftc.teamcode.base.config;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONException;

public class LimelightConfig {
    public Limelight3A limelight;
    public String      deviceName;
    public int         pollingRate;

    public LimelightConfig(HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        deviceName      = robotConfig.getLimelightString("deviceName");
        if(deviceName != null) {
            limelight   = hardwareMap.get(Limelight3A.class, this.deviceName);
            pollingRate = robotConfig.getLimelightInt("pollingRate");
            limelight.setPollRateHz(pollingRate);
        }
    }
}