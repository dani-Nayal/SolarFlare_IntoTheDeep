package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "OnePlusThreespecimen", group = "Autonomous")
public class OneSpecimenPlusThreeSpecimen extends LinearOpMode {
    HardwareConfig hw;
    CustomActions actions;
    RobotState state;

    @Override
    public void runOpMode() {
        hw = new HardwareConfig(hardwareMap);
        state = new RobotState();
        actions = new CustomActions(state, hw);

        Pose2d initialPose = new Pose2d(24, -62, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);



        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalPID(),
                        new SequentialAction(

                        )
                )
        );
    }
}