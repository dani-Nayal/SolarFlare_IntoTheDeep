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
        this.deviceName = robotConfig.getLimelightString("deviceName");
        this.limelight  = hardwareMap.get(Limelight3A.class, this.deviceName);
        this.pollingRate = robotConfig.getLimelightInt("pollingRate");
        limelight.setPollRateHz(pollingRate);
    }
}