package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;

import java.util.List;

@Autonomous
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

        while (opModeIsActive()) {
            LLResult result = hw.getLimelightConfig().limelight.getLatestResult();

            telemetry.addData("x", result.getTx());
            telemetry.addData("y", result.getTy());
            telemetry.addData("area percent", result.getTa());
            telemetry.addData("sample type", result.getClass());
            telemetry.update();
        }
    }
}

