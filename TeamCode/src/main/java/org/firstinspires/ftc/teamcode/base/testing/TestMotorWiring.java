package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.json.JSONException;
@TeleOp
public class TestMotorWiring extends LinearOpMode {
    RobotConfig robotConfig;
    HardwareConfig hw;
    @Override
    public void runOpMode(){
        robotConfig = RobotConfig.createInstance("IntoTheDeep-V2");
        try {
            hw = HardwareConfig.createInstance(hardwareMap, robotConfig);

        }
        catch (Exception e){
            throw new RuntimeException(e);
        }

        waitForStart();

        hw.getMotorConfig(MotorEnum.LEFT_BACK).motor.setPower(1);
    }
}
