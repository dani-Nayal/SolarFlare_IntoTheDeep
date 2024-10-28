package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


@Autonomous
public class MegaTag2Localization extends LinearOpMode{
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
                    telemetry.addData("x pos", x);
                    telemetry.addData("y pos", y);
                    telemetry.update();
                }
            }
        }
    }
}
