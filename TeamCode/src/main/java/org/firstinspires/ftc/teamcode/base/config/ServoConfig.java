package org.firstinspires.ftc.teamcode.base.config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.json.JSONException;

public class ServoConfig{
    public ServoEnum       servoEnum;
    public String          deviceName;
    public Servo           servo;
    public Servo.Direction direction;
    public double          minServoPosition;
    public double          maxServoPosition;
    public double          degreesPerSecond;

    public ServoConfig(ServoEnum servoEnum, HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        this.servoEnum        = servoEnum;
        this.deviceName       = robotConfig.getServoString(servoEnum,"deviceName");
        this.servo            = hardwareMap.servo.get(deviceName);
        this.direction        = robotConfig.getServoDirection(servoEnum);

        this.minServoPosition = robotConfig.getServoDouble(servoEnum,"minServoPosition");
        this.maxServoPosition = robotConfig.getServoDouble(servoEnum,"maxServoPosition");
        this.degreesPerSecond = robotConfig.getServoDouble(servoEnum,"degreesPerSecond");

        servo.setDirection(this.direction);
    }
}
