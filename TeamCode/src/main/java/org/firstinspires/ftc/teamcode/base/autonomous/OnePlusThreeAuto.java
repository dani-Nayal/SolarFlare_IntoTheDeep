package org.firstinspires.ftc.teamcode.base.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OnePlusThreeAuto extends LinearOpMode {
    CustomActions actions;
    @Override
    public void runOpMode(){
        actions = new CustomActions(telemetry);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        actions.globalMechanismControl(),
                        new SequentialAction(
                                actions.moveToHighChamberAndScoreSpecimen(
                                        actions.getInitialDrivePosition("specimen", "sample"),
                                        new Vector2d(-9,-58),
                                        Math.toRadians(90)
                                )
                        )
                )
        );
    }
}
