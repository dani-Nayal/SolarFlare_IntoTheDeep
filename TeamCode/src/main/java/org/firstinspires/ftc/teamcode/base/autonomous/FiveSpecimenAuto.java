package org.firstinspires.ftc.teamcode.base.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotConfig;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

@Autonomous
public class FiveSpecimenAuto extends LinearOpMode {
    RobotConfig robotConfig;
    HardwareConfig hw;
    RobotState state;
    CustomActions actions;
    @Override
    public void runOpMode() throws InterruptedException {
        robotConfig = RobotConfig.createInstance("IntoTheDeep-V2");

        try {
            hw = HardwareConfig.createInstance(hardwareMap, robotConfig);
        }
        catch(Exception e){
            throw new RuntimeException(e);
        }

        state = RobotState.getInstance();

        actions = new CustomActions(telemetry);
        actions.setInitialDrivePosition("specimen", "specimen");

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalMechanismControl(),
                        new SequentialAction(
                                actions.sleepUntilPose(
                                        new Pose2d(hw.robotDimensions.length / 2,-45, Math.toRadians(90)),
                                        10,
                                        8)
                        )
                )
        );
    }
}
