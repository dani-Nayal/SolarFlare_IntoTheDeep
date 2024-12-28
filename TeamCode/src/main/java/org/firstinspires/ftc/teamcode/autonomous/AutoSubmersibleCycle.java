package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;

import java.util.List;

@Autonomous
public class AutoSubmersibleCycle extends LinearOpMode {

    // First, position the robot in the optimal pickup position for the sample using tx and ty
    // Next, find which detection is the target result by comparing result tx to detection tx
    // When we know which detection is the target, find the corners of the detection
    // Find the longer side of the target sample detection box (detection box is never rotated)
    // If longer side is on the top / bottom of the rectangle, rotate clawWrist to x position
    // If longer side is on left / right side of the rectagle, rotate clawWrist to y position
    // If the sides are =, rotate clawWrist to y position

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

            if (result.isValid()){
                double x = result.getTx();
                double y = result.getTy();
                double area = result.getTa();



                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("area", area);
            }
            telemetry.update();
        }
    }
}

