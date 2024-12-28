package org.firstinspires.ftc.teamcode.autonomous;

import android.content.ClipboardManager;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;

import java.util.List;

public class AutoSubmersibleCycle extends LinearOpMode {
    // top left is 0,0
    int LONG_SIDE_PIXELS = 680;
    int SHORT_SIDE_PIXELS = 480;
    HardwareConfig hw;
    @Override
    public void runOpMode(){
        HardwareConfig.makeInstance(hardwareMap);
        hw = HardwareConfig.getInstance();

        hw.getLimelightConfig().limelight.pipelineSwitch(4);
        hw.getLimelightConfig().limelight.start();

        waitForStart();

        while (opModeIsActive()){
            LLResult result = hw.getLimelightConfig().limelight.getLatestResult();

            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            for (LLResultTypes.DetectorResult detection : detections) {

                String className = detection.getClassName(); // What was detected

                double x = detection.getTargetXDegrees(); // Where it is (left-right)
                double y = detection.getTargetYDegrees(); // Where it is (up-down)
                double area = detection.getTargetArea();
                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("area percent", area);
            }
        }
    }
}
