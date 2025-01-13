package org.firstinspires.ftc.teamcode.base.config;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONException;

public class PinpointConfig{
    public GoBildaPinpointDriverRR pinpoint;
    public String                  deviceName;

    public PinpointConfig(HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        this.deviceName = robotConfig.getPinpointString("deviceName");
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, deviceName);
    }
}
