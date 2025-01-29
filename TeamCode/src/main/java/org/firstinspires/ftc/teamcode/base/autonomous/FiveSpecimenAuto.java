package org.firstinspires.ftc.teamcode.base.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
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

        Action specimenSplinePathing = actions.getDrive().actionBuilder(new Pose2d(12.4375/2,-70+15.0625/2,Math.toRadians(90)))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(12.4375/2,-45), Math.toRadians(90))
                .waitSeconds(0.001)
                .splineToSplineHeading(new Pose2d(38,-38,Math.toRadians(45)),Math.toRadians(0))
                .setTangent(0)
                .waitSeconds(0.001)
                .splineToLinearHeading(new Pose2d(43,-41,Math.toRadians(-50)),Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(46,-38,Math.toRadians(37)),Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(52,-41,Math.toRadians(-55)),Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(57,-38,Math.toRadians(37)),Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(58,-41,Math.toRadians(-10)),Math.toRadians(-80))
                .splineToSplineHeading(new Pose2d(50,-45,Math.toRadians(-110)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(36,-49,Math.toRadians(-90)),Math.toRadians(225))
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(12.4375/2,-45,Math.toRadians(90)),Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(36,-49,Math.toRadians(-90)),Math.toRadians(-90))
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(12.4375/2,-45,Math.toRadians(90)),Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(36,-49,Math.toRadians(-90)),Math.toRadians(-90))
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(12.4375/2,-45,Math.toRadians(90)),Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(36,-49,Math.toRadians(-90)),Math.toRadians(-90))
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(12.4375/2,-45,Math.toRadians(90)),Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(39,-56,Math.toRadians(-45)),Math.toRadians(-45))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalMechanismControl(),
                        specimenSplinePathing,
                        new SequentialAction(
                                actions.sleepUntilPose(
                                        new Pose2d(actions.getInitialDrivePosition("specimen", "specimen").position.x,-45, Math.toRadians(90)),
                                        15,
                                        3
                                ),
                                actions.scoreHighChamber()
                        )
                )
        );
    }
}
