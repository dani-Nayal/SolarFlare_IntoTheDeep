package org.firstinspires.ftc.teamcode.base.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class FourSampleAuto extends LinearOpMode {
    CustomActions actions;
    @Override
    public void runOpMode(){
        actions = new CustomActions(telemetry);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalMechanismControl(),
                        new SequentialAction(
                                actions.moveToNetZone(actions.getInitialDrivePosition("sample", "sample")),
                                actions.scoreHighBucket(),
                                actions.grabGroundSample(
                                        new Pose2d(-9, -58, Math.toRadians(90)),
                                        new Vector2d(-48,-53),
                                        Math.toRadians(90),
                                        500
                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone(new Pose2d(-48, -53, Math.toRadians(90)))
                                ),
                                actions.scoreHighBucket(),
                                actions.grabGroundSample(
                                        new Pose2d(-54,-54, Math.toRadians(45)),
                                        new Vector2d(-57, -50),
                                        Math.toRadians(90),
                                        500
                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone(new Pose2d(-57, -50, Math.toRadians(45)))
                                ),
                                actions.scoreHighBucket(),
                                actions.grabGroundSample(
                                        new Pose2d(-54,-54, Math.toRadians(45)),
                                        new Vector2d(-61,-50),
                                        Math.toRadians(105),
                                        500
                                ),
                                new ParallelAction(
                                        actions.transferSample(),
                                        actions.moveToNetZone(new Pose2d(-61,-50, Math.toRadians(45)))
                                ),
                                actions.scoreHighBucket(),
                                actions.parkRobotSampleSide(new Pose2d(-61, -50, Math.toRadians(45 )))
                        )
                )
        );
    }
}
