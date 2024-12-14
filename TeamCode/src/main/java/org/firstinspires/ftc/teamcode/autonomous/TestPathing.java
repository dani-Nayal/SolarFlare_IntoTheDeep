package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.RobotState;

@Autonomous
public class TestPathing extends LinearOpMode {
    RobotState state;
    CustomActions actions;
    HardwareConfig hw;
    @Override
    public void runOpMode(){
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        actions = new CustomActions(state, hardwareMap);

        actions.setInitialDrivePosition("specimen", "sample");

        PinpointDrive drive = actions.getDrive();

        Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-3,-46), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket2 = drive.actionBuilder(new Pose2d(-3,-46, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-54.5,-50.2), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket3 = drive.actionBuilder(new Pose2d(-54.5,-50.2, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-56,-53.2), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-56,-53.2, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-63,-50.5), Math.toRadians(273))
                .build();
        Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-63,-50.5, Math.toRadians(273)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54.5,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-54.5,-54, Math.toRadians(225)))
                // sample zone 3
                .strafeToLinearHeading(new Vector2d(-67,-50), Math.toRadians(285))
                .build();
        Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-67,-50, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54.5,-53.5), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-54.5,-53.5, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-44,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24.2,-6), Math.toRadians(0))

                .build();

        waitForStart();

        Actions.runBlocking(
              new SequentialAction(
                      onePlusThreeBucket1,
                      new SleepAction(1),
                      onePlusThreeBucket2,
                      new SleepAction(1),
                      onePlusThreeBucket3,
                      new SleepAction(1),
                      onePlusThreeBucket4,
                      new SleepAction(1),
                      onePlusThreeBucket5,
                      new SleepAction(1),
                      onePlusThreeBucket6,
                      new SleepAction(1),
                      onePlusThreeBucket7,
                      new SleepAction(1),
                      onePlusThreeBucket8,
                      new SleepAction(1)
              )
        );
    }
}
