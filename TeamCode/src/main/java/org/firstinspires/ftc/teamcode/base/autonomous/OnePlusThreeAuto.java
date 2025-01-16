package org.firstinspires.ftc.teamcode.base.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class OnePlusThreeAuto extends LinearOpMode {
    CustomActions actions;
    @Override
    public void runOpMode(){
        actions = new CustomActions();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(

                        )

                )
        );
    }
}
