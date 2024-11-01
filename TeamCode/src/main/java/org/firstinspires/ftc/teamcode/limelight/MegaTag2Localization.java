package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.List;


@TeleOp
public class MegaTag2Localization extends LinearOpMode{
    public final double METERS_TO_INCHES = 39.3701;
    public Vector2d rotateVector(Vector2d vector,double angle){
        return new Vector2d(Math.cos(angle)*vector.x - Math.sin(angle)*vector.y, Math.sin(angle)*vector.x + Math.cos(angle)*vector.y);
    }
    @Override
    public void runOpMode(){

        telemetry.setMsTransmissionInterval(11);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        IMU imu = hardwareMap.get(IMU.class,"imu");

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(6);

        waitForStart();

        while (opModeIsActive()){

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                Pose3D botpose_mt2 = result.getBotpose_MT2();

                if (botpose_mt2 != null) {

                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addLine("Rotated 0 Degrees:");
                    telemetry.addData("x pos", x * METERS_TO_INCHES);
                    telemetry.addData("y pos", y * METERS_TO_INCHES);
                    telemetry.addLine("Rotated 90 Degrees:");
                    telemetry.addData("x pos", -y * METERS_TO_INCHES);
                    telemetry.addData("y pos", x * METERS_TO_INCHES);
                    telemetry.addLine("Rotated 180 Degrees:");
                    telemetry.addData("x pos", -x * METERS_TO_INCHES);
                    telemetry.addData("y pos", -y * METERS_TO_INCHES);
                    telemetry.addLine("Rotated 270 Degrees:");
                    telemetry.addData("x pos", y * METERS_TO_INCHES);
                    telemetry.addData("y pos", -x    * METERS_TO_INCHES);
                    telemetry.update();
                }
            }
        }
    }
}
