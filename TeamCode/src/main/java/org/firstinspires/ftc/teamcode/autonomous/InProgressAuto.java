package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous
public class InProgressAuto extends LinearOpMode {
    HardwareConfig hw;
    CustomActions actions;
    RobotState state;
    PinpointDrive drive;
    @Override
    public void runOpMode() {
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        actions = new CustomActions(state, hardwareMap);
        actions.setInitialDrivePosition("specimen", "sample");
        drive = actions.getDrive();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalPID(),
                        actions.updateTelemetry(telemetry),
                        new SequentialAction(
                                actions.moveToHighChamberAndScoreSpecimen(
                                        // Initial Drive Position
                                        actions.getInitialDrivePosition("specimen", "sample"),
                                        // Scoring Position
                                        new Vector2d(-3,-47),
                                        // Scoring heading
                                        Math.toRadians(270)

                                ),
                                actions.grabGroundSample(
                                        new Pose2d(-3,-47,Math.toRadians(270)),
                                        // Scoring Position
                                        new Vector2d(-48.6,-51),
                                        // Scoring heading
                                        Math.toRadians(270),
                                        // Extension length
                                        425

                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone(
                                                // Initial Drive Position
                                                new Pose2d(-48.6, -51, Math.toRadians(270)),
                                                // Scoring Position
                                                new Vector2d(-54,-54),
                                                // Scoring Heading
                                                Math.toRadians(225)
                                        )
                                ),
                                actions.scoreHighBucket(),
                                actions.grabGroundSample(
                                        // Initial Drive Position
                                        new Pose2d(-54,-54, Math.toRadians(225)),
                                        // Pick up position
                                        new Vector2d(-57,-51),
                                        // Pick up heading
                                        Math.toRadians(273),
                                        // Extension length
                                        425

                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone(
                                                // Initial Drive Position
                                                new Pose2d(-57, -51,Math.toRadians(273)),
                                                // Scoring Position
                                                new Vector2d(-54,-54),
                                                // Scoring Heading
                                                Math.toRadians(225)
                                        )
                                ),
                                actions.scoreHighBucket(),
                                actions.grabGroundSample(
                                        // Initial Drive Position
                                        new Pose2d(-54,-54, Math.toRadians(225)),
                                        // Pick Up Position
                                        new Vector2d(-67,-51),
                                        // Pick Up Heading
                                        Math.toRadians(285),
                                        // Extension Length
                                        425
                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone(
                                                // Initial Drive Position
                                                new Pose2d(-67,-51,Math.toRadians(285)),
                                                // Scoring Position
                                                new Vector2d(-54,-54),
                                                // Scoring Heading
                                                Math.toRadians(225)
                                        )
                                ),
                                actions.scoreHighBucket(),

                                // Park
                                drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(225)))
                                        .strafeToLinearHeading(new Vector2d(-44,-6), Math.toRadians(0))
                                        .strafeToLinearHeading(new Vector2d(-24.2,-6), Math.toRadians(0))
                                        .build()
                        )
                )
        );
    }
}