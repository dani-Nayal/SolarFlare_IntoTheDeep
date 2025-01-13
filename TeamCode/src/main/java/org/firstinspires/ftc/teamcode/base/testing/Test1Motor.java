package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;

@TeleOp
public class Test1Motor extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    MotorControl motorControl;

    public void runOpMode(){
        HardwareConfig.createInstance(hardwareMap);
        hw = HardwareConfig.getInstance();
        motorControl = new MotorControl(MotorEnum.TESTING_MOTOR);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 0);
            }
            else if (gamepad1.b){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 500);
            }
            else if (gamepad1.y){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 1000);
            }
            else if (gamepad1.x){
                state.setMotorTarget(MotorEnum.TESTING_MOTOR, 1500);
            }

            motorControl.runTrapezoidalMotionProfile();
        }
    }
}
