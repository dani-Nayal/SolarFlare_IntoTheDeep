package org.firstinspires.ftc.teamcode.base.testing;

import java.util.logging.Logger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.util.logging.Level.INFO;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotLogger;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl1D;

@Autonomous
public class Rig1Motor extends LinearOpMode {
    Logger         logger;
    RobotConfig    robotConfig;
    HardwareConfig hardwareConfig;
    RobotState     robotState;
    MotorControl1D motorControl;

    public void runOpMode(){
        sleep(3000);
        try {
            logger         = RobotLogger.getInstance().getConfigLogger();
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created configLogger");
            robotConfig    = RobotConfig.createInstance("Rig1Motor");
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created robotConfig");
            hardwareConfig = HardwareConfig.createInstance(hardwareMap, robotConfig);
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created hardwareConfig");
            motorControl   = new MotorControl1D(MotorEnum.TESTING_MOTOR);
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created motorControl");
            robotState     = RobotState.getInstance();
            logger.logp(INFO, "Rig1Motor", "runOpMode", "Created robotState");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Done with initialization", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 0);
            }
            else if (gamepad1.b){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 500);
            }
            else if (gamepad1.y){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 1500);
            }
            else if (gamepad1.x){
                robotState.setMotorTarget(MotorEnum.TESTING_MOTOR, 2000);
            }

            motorControl.runTrapezoidalMotionProfile(telemetry);
        }
    }
}
