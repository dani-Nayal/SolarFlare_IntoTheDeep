package org.firstinspires.ftc.teamcode.base.measurevalues;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;
import org.json.JSONException;

@Config
@TeleOp
public class TunePIDCoefficients extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    static MotorEnum testingMotor = MotorEnum.TESTING_MOTOR;
    public static double kP;
    public static double kI;
    public static double kD;

    @Override
    public void runOpMode(){
        try {
            RobotConfig robotConfig = RobotConfig.createInstance("Rig1Motor");
            hw = HardwareConfig.createInstance(hardwareMap, robotConfig);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
        state = RobotState.getInstance();

        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.getMotorConfig(MotorEnum.TESTING_MOTOR).motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry telemetry = dashboard.getTelemetry();

        ElapsedTime timer = new ElapsedTime();

        double lastError = 0;

        double lastReference = 0;

        double integralSum = 0;

        waitForStart();

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
            double reference = state.getMotorTarget(testingMotor);

            double encoderPosition = hw.getMotorConfig(testingMotor).motor.getCurrentPosition();

            double error = state.getMotorTarget(testingMotor) - encoderPosition;

            double derivative = (error - lastError) / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            if (reference != lastReference){
                integralSum = 0;
            }

            double proportionalPower = error * kP;

            double integralPower = integralSum * kI;

            double derivativePower = derivative * kD;

            double outPower = proportionalPower + integralPower + derivativePower;

            hw.getMotorConfig(testingMotor).motor.setPower(outPower);

            lastError = error;

            lastReference = reference;

            timer.reset();

            telemetry.addData("motor target", state.getMotorTarget(testingMotor));
            telemetry.addData("motor position", hw.getMotorConfig(testingMotor).motor.getCurrentPosition());
            telemetry.addData("motor power", outPower);
            telemetry.update();
        }
    }
}
