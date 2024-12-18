package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.RobotState;
@TeleOp
public class TestPID extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    @Override
    public void runOpMode(){
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();

        MotorEnum testingMotor = null;

        waitForStart();

        while(testingMotor == null){
            if (gamepad1.a){
                testingMotor = MotorEnum.EXTENDO;
                telemetry.addData("Testing Motor: ",testingMotor);
            }
            else if (gamepad1.b){
                testingMotor = MotorEnum.EXTENDO_PITCH;
                telemetry.addData("Testing Motor: ",testingMotor);
            }
            else if (gamepad1.y){
                testingMotor = MotorEnum.BUCKET_SLIDES;
                telemetry.addData("Testing Motor: ",testingMotor);
            }
            else if (gamepad1.x){
                testingMotor = MotorEnum.HANG;
                telemetry.addData("Testing Motor: ",testingMotor);
            }
            else if (gamepad1.dpad_down){
                testingMotor = null;
                telemetry.addData("Testing Motor: ", (Object) null);
            }
            telemetry.addLine("Press A to test extendo");
            telemetry.addLine("Press B to test extendoPitch");
            telemetry.addLine("Press Y to test bucketSlides");
            telemetry.addLine("Press X to test hang");
            telemetry.addLine("Press DpadDown to reset");
            telemetry.update();
        }

        while (opModeIsActive()){
            if (gamepad1.a){
                state.setMotorTarget(testingMotor, (int) (hw.getMotorConfig(testingMotor).maxTarget * 0.25));
            }
            else if (gamepad1.b){
                state.setMotorTarget(testingMotor, (int) (hw.getMotorConfig(testingMotor).maxTarget * 0.5));
            }
            else if (gamepad1.y){
                state.setMotorTarget(testingMotor, (int) (hw.getMotorConfig(testingMotor).maxTarget * 0.75));
            }
            else if (gamepad1.x){
                state.setMotorTarget(testingMotor, hw.getMotorConfig(testingMotor).maxTarget);
            }

            // Implement PID here
            // Use FTC DashBoard to tune PID Coefficients

            double error = state.getMotorTarget(testingMotor) - hw.getMotorConfig(testingMotor).motor.getCurrentPosition();

            double proportionalPower = error * hw.getMotorConfig(testingMotor).kP;

            hw.getMotorConfig(testingMotor).motor.setPower(proportionalPower);
        }
    }
}
