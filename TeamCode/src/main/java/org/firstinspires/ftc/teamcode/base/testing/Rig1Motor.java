package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotLogger;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;

@Autonomous
public class Rig1Motor extends LinearOpMode {
    RobotConfig    robotConfig;
    HardwareConfig hw;
    RobotState     state;
    MotorControl   motorControl;

    public void runOpMode(){
        sleep(3000);
        try {
            robotConfig  = RobotConfig.createInstance("Rig1Motor");
            hw           = HardwareConfig.createInstance(hardwareMap, robotConfig);
            motorControl = new MotorControl(MotorEnum.TESTING_MOTOR);
            state        = RobotState.getInstance();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Done with initialization", "");
        telemetry.update();

        waitForStart();

        int choice = 0;
        while (opModeIsActive()) {
            switch(choice) {
                case 0:
                    state.setMotorTarget(MotorEnum.TESTING_MOTOR, 0);
                    RobotLogger.getInstance().addData("information", "Theone", 1, 2.0);
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

            motorControl.runTrapezoidalMotionProfile(telemetry);
        }
    }
}
