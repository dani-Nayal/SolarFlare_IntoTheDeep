package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;
import org.json.JSONException;

@TeleOp
public class Test1Motor extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    MotorControl motorControl;

    public void runOpMode(){
        try {
            RobotConfig robotConfig = RobotConfig.createInstance("Rig1Motor");
            HardwareConfig.createInstance(hardwareMap, robotConfig);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
        motorControl = new MotorControl(MotorEnum.TESTING_MOTOR);


        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()){
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
