package org.firstinspires.ftc.teamcode.base.Calibration;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.config.MotorConfig;

public class MotorCalibration {
    private static final int         CALIBRATION_PAUSE = 500; // milliseconds
    private static final String      POWER_SPEED_FMT   = "Power_Speed-%1$s";
    private static final int         RESOLUTION        = 100;
    private              double      maxPower          = 1;
    private              double[]    time              = new double[RESOLUTION];
    private              double[]    power             = new double[RESOLUTION];
    private              double[]    speed             = new double[RESOLUTION];
    private              MotorConfig motorConfig;

    public MotorCalibration(MotorConfig motorConfig) {
        this.motorConfig = motorConfig;
        this.maxPower    = motorConfig.maxPower;
    }

    public void calibratePower_v_Speed() {
        for(int inc=1; inc<=RESOLUTION; inc++) {
            System.out.println("This is a placeholder");
        }
    }
}
