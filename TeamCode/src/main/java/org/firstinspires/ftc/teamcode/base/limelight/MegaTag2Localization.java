package org.firstinspires.ftc.teamcode.base.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.base.HardwareConfig;

@Autonomous
public class MegaTag2Localization extends LinearOpMode{
    // TODO: Actually get MegaTag2 to work :(
    HardwareConfig hw;
    @Override
    public void runOpMode(){
        hw = HardwareConfig.getInstance();
        hw.getLimelightConfig().limelight.pipelineSwitch(6);
        hw.getLimelightConfig().limelight.start();

        waitForStart();

        while (opModeIsActive()){
            LLResult result = hw.getLimelightConfig().limelight.getLatestResult();

            double robotYaw = hw.getImuConfig().imu.getRobotYawPitchRollAngles().getYaw();

            hw.getLimelightConfig().limelight.updateRobotOrientation(robotYaw);

            if (result != null && result.isValid()) {

                Pose3D botPose_mt2 = result.getBotpose_MT2();

                if (botPose_mt2 != null) {
                    double x = botPose_mt2.getPosition().x;
                    double y = botPose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    telemetry.update();
                }
            }
        }
    }
}
