package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.RobotState;

@Autonomous
public class InProgressAuto extends LinearOpMode {
    HardwareConfig hw;
    CustomActions actions;
    RobotState state;
    @Override
    public void runOpMode() {

        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        actions = new CustomActions(state, hardwareMap);
        actions.setInitialDrivePosition("specimen", "sample");

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalPID(),
                        actions.updateTelemetry(telemetry),
                        new SequentialAction(
                                actions.moveToHighChamberAndScoreSpecimen(
                                        // Scoring Position
                                        new Vector2d(-3,-47),
                                        // Scoring heading
                                        Math.toRadians(270)
                                ),
                                actions.grabGroundSample(
                                        // Scoring Position
                                        new Vector2d(-48.6,-51),
                                        // Scoring heading
                                        Math.toRadians(270),
                                        // Extension length
                                        425
                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone()
                                ),
                                actions.scoreHighBucket(),
                                actions.grabGroundSample(
                                        // Pick up position
                                        new Vector2d(-57,-51),
                                        // Pick up heading
                                        Math.toRadians(273),
                                        // Extension length
                                        425
                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone()
                                ),
                                actions.scoreHighBucket(),
                                actions.grabGroundSample(
                                        // Pick up position
                                        new Vector2d(-67,-51),
                                        // Pick up heading
                                        Math.toRadians(285),
                                        // Extension length
                                        425
                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone()
                                ),
                                actions.scoreHighBucket()
                        )
                )
        );
    }
}