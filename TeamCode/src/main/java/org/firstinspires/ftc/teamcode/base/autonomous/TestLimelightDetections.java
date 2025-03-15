package org.firstinspires.ftc.teamcode.base.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestLimelightDetections extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(4);

        waitForStart();

        limelight.start();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()){
                for (LLResultTypes.DetectorResult detection : result.getDetectorResults()){
                    double targetX = detection.getTargetXDegrees();
                    double targetY = detection.getTargetYDegrees();
                    telemetry.addData("x pos", targetX);
                    telemetry.addData( "y pos", targetY);
                    telemetry.update();
                }
            }
        }
    }
}
