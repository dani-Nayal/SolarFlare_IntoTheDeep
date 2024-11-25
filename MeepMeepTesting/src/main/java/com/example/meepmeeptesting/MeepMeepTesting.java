package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(12.4375,15.0625)
                .build();
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


        Action onePlusThreeSpecimenV1 = myBot.getDrive().actionBuilder(new Pose2d(24, -62, Math.toRadians(90)))
                // Go to score area first time
                .strafeToLinearHeading(new Vector2d(10,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample 1 in sample zones
                .strafeToLinearHeading(new Vector2d(30,-45), Math.toRadians(55))
                .waitSeconds(1)
                // Go to human player area 1st time
                .strafeToLinearHeading(new Vector2d(34,-46), Math.toRadians(300))
                .waitSeconds(4)
                // Go to score area 2nd time
                .strafeToLinearHeading(new Vector2d(4,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample 2 in sample zones
                .strafeToLinearHeading(new Vector2d(37,-37), Math.toRadians(30))
                .waitSeconds(1)
                // Go to human player zone 2nd time
                .strafeToLinearHeading(new Vector2d(34,-46), Math.toRadians(300))
                .waitSeconds(4)
                // Go to scoring area 3rd time
                .strafeToLinearHeading(new Vector2d(0,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample 3 in sample zones
                .strafeToLinearHeading(new Vector2d(43,-40), Math.toRadians(35))
                .waitSeconds(1)
                // Go to human player zone 3rd time
                .strafeToLinearHeading(new Vector2d(34,-46), Math.toRadians(300))
                .waitSeconds(4)
                // Go to scoring area 4th time
                .strafeToLinearHeading(new Vector2d(4,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                .build();

        // Incomplete
        Action onePlusThreeSpecimenV2 = myBot.getDrive().actionBuilder(new Pose2d(24, -62, Math.toRadians(90)))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(10,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample 1 sample zone
                .strafeToLinearHeading(new Vector2d(42,-37), Math.toRadians(70))
                // Rotate towards observation zone
                .strafeToLinearHeading(new Vector2d(46.7,-48.3), Math.toRadians(-90))
                // Rotate to sample 2 in sample zone
                .strafeToLinearHeading(new Vector2d(54,-35), Math.toRadians(65))
                // Rotate towards observation zone 2nd time
                .strafeToLinearHeading(new Vector2d(46.7,-48.3), Math.toRadians(-90))
                // Rotate towards sample 3 in sample zone
                .strafeToLinearHeading(new Vector2d(60,-30.5), Math.toRadians(30))
                // Rotate towards observation zone
                .strafeToLinearHeading(new Vector2d(46.7,-48.3), Math.toRadians(-90))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(-45))
                // Score second specimen
                .strafeToLinearHeading(new Vector2d(8,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(-45))
                .waitSeconds(0.5)
                // Score fourth specimen
                .strafeToLinearHeading(new Vector2d(4,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(-45))
                .waitSeconds(0.5)
                // Score third specimen
                .strafeToLinearHeading(new Vector2d(2,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Park
                .strafeToLinearHeading(new Vector2d(34,-62), Math.toRadians(90))
                .build();

        Action onePlusThreeSpecimenV3 = myBot.getDrive().actionBuilder(new Pose2d(24,-62, Math.toRadians(90)))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(10,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Move to pushing position 1
                .strafeToLinearHeading(new Vector2d(31.5,-33.7), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40,-13), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(47,-13), Math.toRadians(90))
                // Move down
                .strafeToLinearHeading(new Vector2d(31.5,-33.7), Math.toRadians(90))

                .build();

        Action onePlusFourSpecimen = myBot.getDrive().actionBuilder(new Pose2d(24,-62, Math.toRadians(90)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(9,-58), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(47.8,-44), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 2
                .strafeToLinearHeading(new Vector2d(58,-44), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 3
                .strafeToLinearHeading(new Vector2d(62,-44), Math.toRadians(74))
                .waitSeconds(1)
                // Go to OCP
                .strafeToLinearHeading(new Vector2d(9,-59.5), Math.toRadians(0))
                .waitSeconds(1)
                // Score
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                // Park
                .strafeToLinearHeading(new Vector2d(30,-62), Math.toRadians(90))
                .build();

        Action onePlusThreeSpecimen = myBot.getDrive().actionBuilder(new Pose2d(24,-62, Math.toRadians(90)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(9,-58), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(47.8,-44), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 2
                .strafeToLinearHeading(new Vector2d(58,-44), Math.toRadians(90))
                .waitSeconds(1)
                // Go to OCP
                .strafeToLinearHeading(new Vector2d(9,-59.5), Math.toRadians(0))
                .waitSeconds(1)
                // Score
                .turnTo(Math.toRadians(90))
                .waitSeconds(1.5)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1.5)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1.5)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)
                // Park
                .strafeToLinearHeading(new Vector2d(30,-62), Math.toRadians(90))
                .build();

        Action onePlusThreeBucket = myBot.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9,-58), Math.toRadians(270))
                .waitSeconds(1.5)
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-53), Math.toRadians(270))
                .waitSeconds(1.5)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .waitSeconds(2)
                // Go to sample zone 2
                .strafeToLinearHeading(new Vector2d(-57,-50), Math.toRadians(270))
                .waitSeconds(1.5)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .waitSeconds(2)
                // Turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61,-50), Math.toRadians(285))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .waitSeconds(2)
                // Park
                .strafeToLinearHeading(new Vector2d(-30,-6), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-23.4,-6), Math.toRadians(180))
                .waitSeconds(1)
                .build();
        Action onePlusThreeBucket1 = myBot.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(270)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9,-58), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket2 = myBot.getDrive().actionBuilder(new Pose2d(-9,-58, Math.toRadians(270)))
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-53), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket3 = myBot.getDrive().actionBuilder(new Pose2d(-48,-53, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket4 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // Sample zone 2
                .strafeToLinearHeading(new Vector2d(-57,-50), Math.toRadians(270))
                .build();
        Action onePlusThreeBucket5 = myBot.getDrive().actionBuilder(new Pose2d(-57,-50, Math.toRadians(270)))
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket6 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61,-50), Math.toRadians(285))
                .build();
        Action onePlusThreeBucket7 = myBot.getDrive().actionBuilder(new Pose2d(-61,-50, Math.toRadians(285)))
                // turn and score bucket
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                .build();
        Action onePlusThreeBucket8 = myBot.getDrive().actionBuilder(new Pose2d(-54,-54, Math.toRadians(225)))
                // park
                .strafeToLinearHeading(new Vector2d(-35,-6), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-23.4,-6), Math.toRadians(180))
                .build();






        // onePlusThreeSpecimenV1

        // Strafe to 10, 33.7 with linear heading at 90
        // Score specimen top rung
        // Strafe to 30, -37 with linear heading at 0
        // Extend extendo
        // Pick up first sample zone sample
        // retract extendo
        // Strafe to 34, -43 will linear heading at 300
        // Extend extendo
        // Drop sample
        // Pick up specimen
        // Retract extendo
        // Strafe to 4, -33.7 with linear heading at 90
        // Score specimen on top rung
        // Strafe to 37,-41 with linear heading at 30
        // Extend extendo
        // Pick up second sample zone sample
        // Retract extendo
        // Strafe to 40, -43 with linear heading at -300
        // Extend extendo
        // Drop sample
        // Pick up specimen
        // Retract extendo
        // Strafe to 4, -33.7 with linear heading at 90
        // Score specimen on top rung
        // Strafe to 43, -40 with linear heading at 45
        // Extend extendo
        // Pick up third sample from sample zone
        // Retract extendo
        // Strafe to 37, -43 with linear heading at 30
        // Extend extendo
        // Drop sample
        // Pick up specimen
        // Retract extendo
        // Strafe to 1, -33.7 with linear heading at 90
        // Score specimen on top rung


        // Extendo max length is 18.5 inches
        // Outtake length is 6.5 inches


                /*
                myBot.runAction(new SequentialAction(
                        onePlusThreeBucket1,
                        new SleepAction(2),
                        onePlusThreeBucket2,
                        new SleepAction(2),
                        onePlusThreeBucket3,
                        new SleepAction(2),
                        onePlusThreeBucket4,
                        new SleepAction(2),
                        onePlusThreeBucket5,
                        new SleepAction(2),
                        onePlusThreeBucket6,
                        new SleepAction(2),
                        onePlusThreeBucket7,
                        new SleepAction(2),
                        onePlusThreeBucket8,
                        new SleepAction(2)

                        ));
                */

        myBot.runAction(onePlusThreeSpecimenV2);


        /*
        myBot.runAction(onePlusThreeBucket3);
        myBot.runAction(onePlusThreeBucket4);
        myBot.runAction(onePlusThreeBucket5);
        myBot.runAction(onePlusThreeBucket6);
        myBot.runAction(onePlusThreeBucket7);
        myBot.runAction(onePlusThreeBucket8);

*/

    }
}