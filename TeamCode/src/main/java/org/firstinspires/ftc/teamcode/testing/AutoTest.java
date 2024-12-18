package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.autonomous.CustomActions;

@Autonomous
public class AutoTest extends LinearOpMode {
    HardwareConfig hw;
    CustomActions actions;
    RobotState state;
    @Override
    public void runOpMode(){
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        actions = new CustomActions(state,hardwareMap);
        actions.setInitialDrivePosition("specimen", "sample");
        PinpointDrive drive = actions.getDrive();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalPID(),
                        actions.updateTelemetry(telemetry),
                        new SequentialAction(
                                actions.grabGroundSample(
                                        actions.getInitialDrivePosition("specimen", "sample"),
                                        // Scoring Position
                                        new Vector2d(-48.6,-51),
                                        // Scoring heading
                                        Math.toRadians(270),
                                        // Extension length
                                        425
                                ),
                                actions.moveToNetZone(
                                        new Pose2d(-48.6,-51,Math.toRadians(270)),
                                        new Vector2d(-54,-54),
                                        Math.toRadians(225)
                                )
                        )
                )
        );
    }
}
