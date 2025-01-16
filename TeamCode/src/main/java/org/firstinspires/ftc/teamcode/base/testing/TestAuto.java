package org.firstinspires.ftc.teamcode.base.testing;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.autonomous.CustomActions;

@Autonomous
public class TestAuto extends LinearOpMode {
    CustomActions actions;
    @Override
    public void runOpMode(){
        actions  = new CustomActions(telemetry);

        waitForStart();

        Actions.runBlocking(
          new ParallelAction(
                  actions.globalMechanismControl(),
                  new SequentialAction(
                          actions.moveToHighChamberAndScoreSpecimen(
                                  actions.getInitialDrivePosition("specimin", "sample"),
                                  new Vector2d(-5, -40),
                                  Math.toRadians(90)
                          )
                  )
          )
        );

    }
}
