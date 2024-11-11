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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
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
                .strafeToLinearHeading(new Vector2d(34,-43), Math.toRadians(300))
                .waitSeconds(4)
                // Go to score area 2nd time
                .strafeToLinearHeading(new Vector2d(4,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample 2 in sample zones
                .strafeToLinearHeading(new Vector2d(37,-37), Math.toRadians(30))
                .waitSeconds(1)
                // Go to human player zone 2nd time
                .strafeToLinearHeading(new Vector2d(34,-43), Math.toRadians(300))
                .waitSeconds(4)
                // Go to scoring area 3rd time
                .strafeToLinearHeading(new Vector2d(0,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample 3 in sample zones
                .strafeToLinearHeading(new Vector2d(43,-40), Math.toRadians(35))
                .waitSeconds(1)
                // Go to human player zone 3rd time
                .strafeToLinearHeading(new Vector2d(34,-43), Math.toRadians(300))
                .waitSeconds(4)
                // Go to scoring area 4th time
                .strafeToLinearHeading(new Vector2d(-4,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                .build();

        // Incomplete
        Action onePlusThreeSpecimenV2 = myBot.getDrive().actionBuilder(new Pose2d(24, -62, Math.toRadians(90)))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(10,-33.7), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample 1 sample zone
                .strafeToLinearHeading(new Vector2d(40,-47), Math.toRadians(70))
                .waitSeconds(1)
                // Rotate towards human player zone
                .strafeToLinearHeading(new Vector2d(39,-47), Math.toRadians(300))
                .waitSeconds(1)
                // Rotate to sample 2 in sample zone
                .strafeToLinearHeading(new Vector2d(45,-47), Math.toRadians(60))
                .waitSeconds(1)
                // Rotate towards human player zone 2nd time
                .strafeToLinearHeading(new Vector2d(45,-46), Math.toRadians(280))
                .waitSeconds(1)
                // Rotate towards sample 3 in sample zone
                .strafeToLinearHeading(new Vector2d(57,-47), Math.toRadians(60))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55,-47), Math.toRadians(260))
                .waitSeconds(1)
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
                .strafeToLinearHeading(new Vector2d(62,-44), Math.toRadians(76))
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

        Action onePlusThreeBucket = myBot.getDrive().actionBuilder(new Pose2d(-42,-62.5,Math.toRadians(90)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(-9,-58), Math.toRadians(90))
                .waitSeconds(1.5)
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(-48,-53), Math.toRadians(90))
                .waitSeconds(1)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-55,-55), Math.toRadians(45))
                .waitSeconds(2)
                // Go to sample zone 2
                .strafeToLinearHeading(new Vector2d(-57,-50), Math.toRadians(90))
                .waitSeconds(1)
                // Score bucket
                .strafeToLinearHeading(new Vector2d(-60,-54), Math.toRadians(67))
                .waitSeconds(2)
                // Turn and score bucket
                .strafeToLinearHeading(new Vector2d(-61,-50), Math.toRadians(105))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-58,-55), Math.toRadians(55))
                .waitSeconds(2)
                // Park
                .strafeToLinearHeading(new Vector2d(-30,-6), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-23.4,-6), Math.toRadians(0))
                .waitSeconds(1)
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


        // Extendo max length is 21 inches
        // Outtake length is 6.5 inches
        myBot.runAction(onePlusThreeBucket);
    }
}