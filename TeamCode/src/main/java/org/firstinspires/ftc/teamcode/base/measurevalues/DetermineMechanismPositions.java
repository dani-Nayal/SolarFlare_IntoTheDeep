package org.firstinspires.ftc.teamcode.base.measurevalues;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.config.ServoEnum;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;
import org.json.JSONException;

public class DetermineMechanismPositions extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    MotorEnum testingMotor;
    ServoEnum testingServo;
    MotorControl motorControl = new MotorControl(testingMotor);
    int target = 0;
    double position = 0;
    @Override
    public void runOpMode(){
        try {
            RobotConfig robotConfig = RobotConfig.createInstance("Rig1Motor");
            hw    = HardwareConfig.createInstance(hardwareMap, robotConfig);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        state = RobotState.getInstance();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.left_trigger > 0.5 && hw.getMotorConfig(testingMotor).motor.getCurrentPosition() > hw.getMotorConfig(testingMotor).minTarget){
                target -= 10;
                state.setMotorTarget(testingMotor, target);
            }
            else if (gamepad1.right_trigger > 0.5 && hw.getMotorConfig(testingMotor).motor.getCurrentPosition() < hw.getMotorConfig(testingMotor).maxTarget){
                target += 10;
                state.setMotorTarget(testingMotor, target);
            }

            if (gamepad1.left_bumper && hw.getServoConfig(testingServo).servo.getPosition() > hw.getServoConfig(testingServo).minServoPosition){
                position -= 10;
                state.setServoPosition(testingServo, position);
            }
            if (gamepad1.left_bumper && hw.getServoConfig(testingServo).servo.getPosition() < hw.getServoConfig(testingServo).maxServoPosition){
                position -= 10;
                state.setServoPosition(testingServo, position);
            }

            motorControl.runTrapezoidalMotionProfile(telemetry);
            hw.getServoConfig(testingServo).servo.setPosition(state.getServoPosition(testingServo));
        }
    }
}
