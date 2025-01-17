package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotLogger;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl1D;

@Autonomous
public class Rig1Motor extends LinearOpMode {
    RobotConfig    robotConfig;
    HardwareConfig hw;
    RobotState     state;
    MotorControl1D motorControl;

    public void runOpMode(){
        sleep(3000);
        try {
            robotConfig  = RobotConfig.createInstance("Rig1Motor");
            hw           = HardwareConfig.createInstance(hardwareMap, robotConfig);
            motorControl = new MotorControl1D(MotorEnum.TESTING_MOTOR);
            state        = RobotState.getInstance();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Done with initialization", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 0);
            }
            else if (gamepad1.b){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 800);
            }
            else if (gamepad1.y){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 1321);
            }
            else if (gamepad1.x){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 1819);
            }

            motorControl.runTrapezoidalMotionProfile(telemetry);
        }
    }
}
