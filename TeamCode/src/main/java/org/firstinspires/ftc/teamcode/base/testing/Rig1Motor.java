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
        try {
            robotConfig  = RobotConfig.createInstance("Rig2Motor");
            hw           = HardwareConfig.createInstance(hardwareMap, robotConfig);
            motorControl = new MotorControl(MotorEnum.TESTING_MOTOR);
            state        = RobotState.getInstance();
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

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
