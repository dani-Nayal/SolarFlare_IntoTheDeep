package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;

@Autonomous
public class TestDetection extends LinearOpMode {
    HardwareConfig hw;
    @Override
    public void runOpMode(){
        HardwareConfig.makeInstance(hardwareMap);
        hw = HardwareConfig.getInstance();

        waitForStart();


        while (opModeIsActive()){

            LLResultTypes.DetectorResult targetDetection = FindTargetSample.findTargetSample("red", "blue", "yellow");

            if (targetDetection != null){
                telemetry.addData("tx", targetDetection.getTargetXDegrees());
                telemetry.addData("ty", targetDetection.getTargetYDegrees());
                telemetry.addData("ta", targetDetection.getTargetArea());
                telemetry.addData("class name", targetDetection.getClassName());
            }
            else {
                telemetry.addLine("no detections found");
            }

            telemetry.update();
        }
    }
}
