package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.motorcontrol.PID;

import java.util.Objects;

@TeleOp
@Config
public class MeasureMotorAcceleration extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    PID pid;
    public static String motorType = "extendo";
    MotorEnum motorEnum;
    double greatestSpeed = 0;
    double currentSpeed = 0;
    int lastPosition = 0;
    int currentPosition = 0;
    double lastSpeed = 0;
    double currentAcceleration = 0;
    double lastAcceleration = 0;
    double greatestAcceleration = 0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode(){
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        pid = new PID();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        if (Objects.equals(motorType, "extendo")){
            motorEnum = MotorEnum.EXTENDO;
        }
        else if (Objects.equals(motorType, "extendoPitch")){
            motorEnum = MotorEnum.EXTENDO_PITCH;
        }
        else if (Objects.equals(motorType, "bucketSlides")){
            motorEnum = MotorEnum.BUCKET_SLIDES;
        }
        else if (Objects.equals(motorType, "hang")){
            motorEnum = MotorEnum.HANG;
        }
        else{
            throw new IllegalArgumentException("motorType input wrong");
        }

        waitForStart();

        if (gamepad1.a){
            state.setMotorTarget(motorEnum, 0);
            timer.reset();
        }
        else if (gamepad1.b){
            state.setMotorTarget(motorEnum, hw.getMotorConfig(motorEnum).maxTarget);
            timer.reset();
        }

        currentPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();
        currentSpeed = (currentPosition - lastPosition) / timer.seconds();

        if (currentSpeed > greatestSpeed){
            greatestSpeed = currentSpeed;
        }
        if (currentAcceleration > greatestAcceleration){
            greatestAcceleration = currentAcceleration;
        }

        currentAcceleration = currentSpeed - lastSpeed;

        hw.getMotorConfig(motorEnum).motor.setPower(pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum)));

        telemetry.addData("greatest acceleration", greatestAcceleration);
        telemetry.addData("speed", currentSpeed);
        telemetry.addData("position", currentPosition);
        telemetry.update();

        lastPosition = currentPosition;
        lastSpeed = currentSpeed;
        lastAcceleration = currentAcceleration;

    }
}
