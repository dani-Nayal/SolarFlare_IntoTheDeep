package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.json.JSONException;

@TeleOp
public class saefhse extends LinearOpMode {
    HardwareConfig hw;
    @Override
    public void runOpMode(){
        try {
            RobotConfig robotConfig = RobotConfig.createInstance("Rig1Motor");
            HardwareConfig.createInstance(hardwareMap, robotConfig);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
        hw = HardwareConfig.getInstance();
        waitForStart();

        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setPower(1);
    }
}
