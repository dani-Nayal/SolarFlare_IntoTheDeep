package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;

@Autonomous
public class TestLargestDetection extends LinearOpMode {
    HardwareConfig hw;
    @Override
    public void runOpMode(){
        HardwareConfig.makeInstance(hardwareMap);
        hw = HardwareConfig.getInstance();

        hw.getLimelightConfig().limelight.pipelineSwitch(4);

        waitForStart();

        hw.getLimelightConfig().limelight.start();

        while(opModeIsActive()){

            LLResultTypes.DetectorResult closestDetection = FindClosestDetection.findClosestDetection(hw, "red", "blue", "yellow");

            if (closestDetection != null){
                telemetry.addData("closest sample", closestDetection.getClassName());
            }
            else {
                telemetry.addLine("No detections in view");
            }
            telemetry.update();
        }
    }
}
