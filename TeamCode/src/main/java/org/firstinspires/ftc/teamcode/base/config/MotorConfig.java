package org.firstinspires.ftc.teamcode.base.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONException;

public class MotorConfig{
    public MotorEnum motorEnum;
    public String deviceName;
    public DcMotorEx motor;
    public double kP;
    public double kI;
    public double kD;
    public DcMotor.RunMode runMode;
    public DcMotorSimple.Direction direction;
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    public int minTarget;
    public int maxTarget;
    public double maxAcceleration;
    public double maxVelocity;
    public MotorConfig(MotorEnum motorEnum, HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        this.motorEnum         = motorEnum;
        this.deviceName        = robotConfig.getMotorString(motorEnum, "deviceName");
        this.motor             = hardwareMap.get(DcMotorEx.class, deviceName);
        this.kP                = robotConfig.getMotorDouble(motorEnum, "kP");
        this.kI                = robotConfig.getMotorDouble(motorEnum, "kI");
        this.kD                = robotConfig.getMotorDouble(motorEnum, "kD");
        this.runMode           = robotConfig.getMotorRunMode(motorEnum);
        this.direction         = robotConfig.getMotorDirection(motorEnum);
        this.zeroPowerBehavior = robotConfig.getMotorZeroPowerBehavior(motorEnum);
        this.minTarget         = robotConfig.getMotorInt(motorEnum,"minTarget");
        this.maxTarget         = robotConfig.getMotorInt(motorEnum, "maxTarget");
        this.maxAcceleration   = robotConfig.getMotorInt(motorEnum, "maxAcceleration");
        this.maxVelocity       = robotConfig.getMotorInt(motorEnum, "maxVelocity");

        motor.setMode(this.runMode);
        motor.setDirection(this.direction);
        motor.setZeroPowerBehavior(this.zeroPowerBehavior);
    }
}
