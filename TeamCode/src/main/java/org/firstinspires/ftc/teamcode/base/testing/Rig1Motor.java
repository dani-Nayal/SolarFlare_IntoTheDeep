package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;
import org.json.JSONException;

@Autonomous
public class Rig1Motor extends LinearOpMode {
    RobotConfig    robotConfig;
    HardwareConfig hw;
    RobotState     state;
    MotorControl   motorControl;

    public void runOpMode(){
        telemetry.addData("Starting OpMode", "");
        telemetry.update();
        sleep(3000);
        try {
            robotConfig  = RobotConfig.createInstance("Rig1Motor");
            telemetry.addData("After RobotConfig", "");
            telemetry.update();
            sleep(3000);
            hw           = HardwareConfig.createInstance(hardwareMap, robotConfig);
            telemetry.addData("After HardwareConfig", "");
            telemetry.update();
            sleep(3000);
            motorControl = new MotorControl(MotorEnum.TESTING_MOTOR);
            telemetry.addData("After MotorControl", "");
            telemetry.update();
            sleep(3000);
            state        = RobotState.getInstance();
            telemetry.addData("After RobotState", "");
            telemetry.update();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("After initialization", "");
        telemetry.update();
        sleep(3000);

        waitForStart();

        int choice = 0;
        while (opModeIsActive()) {
            switch(choice) {
                case 0:
                    state.setMotorTarget(MotorEnum.TESTING_MOTOR, 0);
                    break;
                case 500:
                    state.setMotorTarget(MotorEnum.TESTING_MOTOR, 500);
                    break;
                case 1000:
                    state.setMotorTarget(MotorEnum.TESTING_MOTOR, 1000);
                    break;
                case 1500:
                    state.setMotorTarget(MotorEnum.TESTING_MOTOR, 1500);
                    break;
            }

            motorControl.runTrapezoidalMotionProfile();
        }
    }
}
