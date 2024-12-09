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
public class AutoTest extends LinearOpMode {
    HardwareConfig hw;
    CustomActions actions;
    RobotState state;
    @Override
    public void runOpMode(){
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        state.setAutoDefaultState();
        actions = new CustomActions(state,hardwareMap);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalPID(),
                        actions.updateTelemetry(telemetry),
                        new SequentialAction(
                               actions.moveToHighChamberAndScoreSpecimen(new Vector2d(0,-30), Math.toRadians(270))
                        )
                )
        );
    }
}
