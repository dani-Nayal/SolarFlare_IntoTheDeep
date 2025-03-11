package org.firstinspires.ftc.teamcode.base.autonomous;

import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendoPitch;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions;

import java.util.Objects;

@Autonomous
public class EightSampleAuto extends OpMode {
    public double[][] values = new double[4][3];
    public int selectedRow = 0;
    public int selectedColumn = 0;
    public boolean dpadPressed = false;
    public boolean firstLoop=true;
    public String[][] labelArray = new String[][]{new String[]{"Samp 1 X","Samp 1 Y","Samp 1 Deg"}, new String[]{"Samp 2 X","Samp 2 Y","Samp 2 Deg"}, new String[]{"Samp 3 X","Samp 3 Y","Samp 3 Deg"}, new String[]{"Samp 4 X","Samp 4 Y","Samp 4 Deg"}};

    PinpointDrive drive;
    public double robotLength = 15.364;
    public double robotWidth  = 14.375;
    public boolean endOfInit=false;
    public final double TICK_TO_IN = 1.1811;
    public final int SERVO_SPEED=555;
    public final int EXTENDO_RETRACTED = 0;
    public final int EXTENDO_EXTENDED = 500;
    public final int EXTENDO_SCORE_SPECIMEN_UP = 480;
    public final int EXTENDO_SCORE_SPECIMEN_DOWN = 300;
    public final int EXTENDO_PITCH_TRANSFER = 0;
    public final int EXTENDO_PITCH_SCORE_SPECIMEN = 0;
    public final int EXTENDO_PITCH_PICK_UP = -1000;
    public final int EXTENDO_PITCH_GRAB_SPECIMEN = -960;
    public final int BUCKET_SLIDES_HIGH_BUCKET = 1070;
    public final int BUCKET_SLIDES_TRANSFER = 0;
    public final int BUCKET_SLIDES_SCORING_SPECIMEN = 350;
    public final int CLAW_FINGERS_OPEN = 92;
    public final int CLAW_FINGERS_CLOSED = 20;
    public final int CLAW_WRIST_DEFAULT = 95;
    public final int CLAW_PITCH_PICK_UP = 13;
    public final int CLAW_PITCH_HOVER = 68;
    public final int CLAW_PITCH_TRANSFER = 100;
    public final int CLAW_PITCH_BACK_OFF = 72;
    public final int CLAW_PITCH_GRAB_SPECIMEN = 145;
    public final int CLAW_PITCH_SCORE_SPECIMEN = 13;
    public final int INNER_CLAW_PITCH_PICK_UP = 82;
    public final int INNER_CLAW_PITCH_HOVER = 20;
    public final int INNER_CLAW_PITCH_TRANSFER = 204;
    public final int INNER_CLAW_PITCH_BACK_OFF = 100;
    public final int INNER_CLAW_PITCH_GRAB_SPECIMEN = 78;
    public final int INNER_CLAW_PITCH_SCORE_SPECIMEN = 82;
    public final int BUCKET_TRANSFER = 36;
    public final int BUCKET_DEPOSIT = 140;
    DcMotorEx extendo;
    DcMotorEx extendoPitch;
    DcMotorEx bucketSlides;
    Servo clawPitchLeft;
    Servo clawPitchRight;
    Servo innerClawPitch;
    Servo clawFingers;
    Servo bucket;
    Servo clawWrist;
    int extendoTarget = EXTENDO_RETRACTED;
    int extendoPitchTarget = EXTENDO_PITCH_TRANSFER;
    int bucketSlidesTarget = BUCKET_SLIDES_TRANSFER;
    double bucketPos = 36;
    @Override
    public void stop(){

    }

    public void generatePath(){
        drive = new PinpointDrive(hardwareMap, new Pose2d(-47.5, -62, Math.toRadians(45)));
        goToFirstPickup = drive.actionBuilder(new Pose2d(-47.5, -62, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-65, -53.3), Math.toRadians(66.5))
                .build();
        goToFirstDeposit = drive.actionBuilder(new Pose2d(-65, -53.3, Math.toRadians(66.5)))
                .strafeToLinearHeading(new Vector2d(-68, -53), Math.toRadians(80))
                .build();
        goToSecondPickup = drive.actionBuilder(new Pose2d(-68, -53, Math.toRadians(80)))
                .strafeToLinearHeading(new Vector2d(-62, -57.3), Math.toRadians(83))
                .build();
        goToSecondDeposit = drive.actionBuilder(new Pose2d(-62, -57.3, Math.toRadians(83)))
                .strafeToLinearHeading(new Vector2d(-68, -51.5), Math.toRadians(70))
                .build();
        goToThirdPickup = drive.actionBuilder(new Pose2d(-68, -51.5, Math.toRadians(70)))
                .strafeToLinearHeading(new Vector2d(-65, -56), Math.toRadians(113))
                .build();
        goToThirdDeposit = drive.actionBuilder(new Pose2d(-65, -56, Math.toRadians(113)))
                .strafeToLinearHeading(new Vector2d(-60, -56), Math.toRadians(45))
                .build();
        goToFirstSub = drive.actionBuilder(new Pose2d(-60, -56, Math.toRadians(45)))
                .setTangent(45)
                .splineToLinearHeading(
                        new Pose2d(
                                -24+1.1811*values[0][0]-29.7,
                                -24+1.1811*values[0][1],
                                Math.toRadians(0)
                        ),
                        Math.toRadians(20)
                )
                .build();
        depositFirstSubAndGoToSecond = drive.actionBuilder(new Pose2d(-24+1.1811*values[0][0]-29.7, -24+1.1811*values[0][1], Math.toRadians(0)))
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(-57,-49,Math.toRadians(45)),Math.toRadians(-85))
                .splineToLinearHeading(
                        new Pose2d(
                                -24+1.1811*values[1][0]-29.7,
                                -24+1.1811*values[1][1],
                                Math.toRadians(0)
                        ),
                        Math.toRadians(20))
                .build();
        depositSecondSubAndGoToThird = drive.actionBuilder(new Pose2d(-24+1.1811*values[1][0]-29.7, -24+1.1811*values[1][1], Math.toRadians(0)))
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(-57,-49,Math.toRadians(45)),Math.toRadians(-85))
                .splineToLinearHeading(
                        new Pose2d(
                                -24+1.1811*values[2][0]-29.7,
                                -24+1.1811*values[2][1],
                                Math.toRadians(0)
                        ),
                        Math.toRadians(20))
                .build();
        depositThirdSubAndGoToFourth = drive.actionBuilder(new Pose2d(-24+1.1811*values[2][0]-29.7, -24+1.1811*values[2][1], Math.toRadians(0)))
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(-57,-49,Math.toRadians(45)),Math.toRadians(-85))
                .splineToLinearHeading(
                        new Pose2d(
                                -24+1.1811*values[3][0]-29.7,
                                -24+1.1811*values[3][1],
                                Math.toRadians(0)
                        ),
                        Math.toRadians(20))
                .build();
        goToFourthSubDeposit = drive.actionBuilder(new Pose2d(-24+1.1811*values[3][0]-29.7, -24+1.1811*values[3][1], Math.toRadians(0)))
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-56,-59,Math.toRadians(45)),Math.toRadians(-85))
                .build();
        path = new ParallelAction(
                new MotorPID(),
                new SequentialAction(
                        new ParallelAction(
                                goToFirstPickup,
                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET),
                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                new ParallelAction(
                                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                        new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                        new SetBucketPositionAction(BUCKET_TRANSFER),
                                        new SetClawWristPositionAction(95),
                                        new SequentialAction(
                                                new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() < -700)),
                                                new SetExtendoTargetAction(EXTENDO_EXTENDED)
                                        )
                                )
                        ),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SleepAction(0.4),
                        new ParallelAction(
                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER),
                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                new SequentialAction(
                                        new SequentialAction(
                                                new ParallelAction(
                                                        new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                                ),
                                                new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                                new SleepAction(0.07)
                                        ),
                                        new ParallelAction(
                                                goToFirstDeposit,
                                                new SequentialAction(
                                                        new ParallelAction(
                                                                new ParallelAction(
                                                                        new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_TRANSFER),
                                                                        new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                                                                        new SetClawWristPositionAction(95),
                                                                        new SequentialAction(
                                                                                new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() > -43)),
                                                                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                                new SleepAction(0.07),
                                                                                new ParallelAction(
                                                                                        new SetClawPitchPositionAction(CLAW_PITCH_BACK_OFF),
                                                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_BACK_OFF)
                                                                                ),
                                                                                new ParallelAction(
                                                                                        new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET),
                                                                                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                                                                        new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                                                                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                                        new SetClawWristPositionAction(95),
                                                                                        new SequentialAction(
                                                                                                new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() < -700)),
                                                                                                new SetExtendoTargetAction(EXTENDO_EXTENDED)
                                                                                        )
                                                                                )
                                                                        )
                                                                )
                                                        )
                                                )
                                        )
                                )
                        ),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SleepAction(0.4),
                        new ParallelAction(
                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER),
                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        goToSecondPickup,
                                        new SleepUntilTrue(()->(
                                                Math.sqrt((drive.pose.position.x-(-62))*(drive.pose.position.x-(-62))+
                                                        (drive.pose.position.y-(-57.3))*(drive.pose.position.y-(-57.3)))<7
                                        )),
                                        new ParallelAction(
                                                new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                        ),
                                        new SleepAction(0.07),
                                        new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                        new SleepAction(0.07)
                                )
                        ),
                        new ParallelAction(
                                goToSecondDeposit,
                                new SequentialAction(
                                        new ParallelAction(
                                                new ParallelAction(
                                                        new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_TRANSFER),
                                                        new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                                                        new SetClawWristPositionAction(95),
                                                        new SequentialAction(
                                                                new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() > -43)),
                                                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                new SleepAction(0.07),
                                                                new ParallelAction(
                                                                        new SetClawPitchPositionAction(CLAW_PITCH_BACK_OFF),
                                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_BACK_OFF)
                                                                ),
                                                                new ParallelAction(
                                                                        new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET),
                                                                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                                                        new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                                                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                        new SetClawWristPositionAction(125),
                                                                        new SequentialAction(
                                                                                new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() < -700)),
                                                                                new SetExtendoTargetAction(EXTENDO_EXTENDED)
                                                                        )
                                                                )
                                                        )
                                                )
                                        )
                                )
                        ),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SleepAction(0.4),
                        new ParallelAction(
                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER),
                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                goToThirdPickup,
                                new SequentialAction(
                                        new SleepUntilTrue(()->(
                                                Math.sqrt((drive.pose.position.x-(-65))*(drive.pose.position.x-(-65))+
                                                (drive.pose.position.y-(-56))*(drive.pose.position.y-(-56)))<7
                                                &&Math.abs(Math.toDegrees(drive.pose.heading.toDouble())-113)<9
                                        )),
                                        new ParallelAction(
                                                new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                        ),
                                        new SleepAction(0.07),
                                        new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                        new SleepAction(0.07)
                                )
                        ),
                        new ParallelAction(
                                goToThirdDeposit,
                                new SequentialAction(
                                        new ParallelAction(
                                                new ParallelAction(
                                                        new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_TRANSFER),
                                                        new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                                                        new SetClawWristPositionAction(95),
                                                        new SequentialAction(
                                                                new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() > -43)),
                                                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                new SleepAction(0.07),
                                                                new ParallelAction(
                                                                        new SetClawPitchPositionAction(CLAW_PITCH_BACK_OFF),
                                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_BACK_OFF)
                                                                ),
                                                                new ParallelAction(
                                                                        new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET),
                                                                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                                                        new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                                                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                        new SetClawWristPositionAction(125)
                                                                )
                                                        )
                                                )
                                        )
                                )
                        ),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SleepAction(0.4),
                        new ParallelAction(
                                goToFirstSub,
                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER),
                                new SetClawWristPositionAction(95 + values[0][2]),
                                new SequentialAction(
                                        new SleepUntilTrue(() -> (
                                                Math.abs(Math.toDegrees(drive.pose.heading.toDouble()) - 0) < 15
                                        )),
                                        new SetExtendoTargetAction(EXTENDO_EXTENDED),
                                        new SleepUntilTrue(()->(
                                                Math.sqrt((drive.pose.position.x-(-24+1.1811*values[0][0]-29.7))*(drive.pose.position.x-(-24+1.1811*values[0][0]-29.7))+
                                                        (drive.pose.position.y-(-24+1.1811*values[0][1]))*(drive.pose.position.y-(-24+1.1811*values[0][1])))<6
                                        )),
                                        new ParallelAction(
                                                new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                        ),
                                        new SleepAction(0.07),
                                        new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                        new SleepAction(0.07)
                                )
                        ),
                        new ParallelAction(
                                depositFirstSubAndGoToSecond,
                                new SequentialAction(
                                        new ParallelAction(
                                                new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_TRANSFER),
                                                new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                                                new SetClawWristPositionAction(95),
                                                new SequentialAction(
                                                        new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() > -43)),
                                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                        new SleepAction(0.07),
                                                        new ParallelAction(
                                                                new SetClawPitchPositionAction(CLAW_PITCH_BACK_OFF),
                                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_BACK_OFF)
                                                        ),
                                                        new ParallelAction(
                                                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET),
                                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                                                new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                                                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                new SetClawWristPositionAction(95 + values[1][2])
                                                        )
                                                )
                                        ),
                                        new SleepUntilTrue(() -> (
                                                (-60 - drive.pose.position.x) * (-60 - drive.pose.position.x) + (-56 - drive.pose.position.y) * (-56 - drive.pose.position.y) < 9
                                        )),
                                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                                        new SleepAction(0.4),
                                        new ParallelAction(
                                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER)
                                        ),
                                        new SequentialAction(
                                                new SleepUntilTrue(() -> (
                                                        Math.abs(Math.toDegrees(drive.pose.heading.toDouble()) - 0) < 15
                                                )),
                                                new SetExtendoTargetAction(EXTENDO_EXTENDED),
                                                new SleepUntilTrue(()->(
                                                        Math.sqrt((drive.pose.position.x-(-24+1.1811*values[1][0]-29.7))*(drive.pose.position.x-(-24+1.1811*values[1][0]-29.7))+
                                                                (drive.pose.position.y-(-24+1.1811*values[1][1]))*(drive.pose.position.y-(-24+1.1811*values[1][1])))<6
                                                )),
                                                new ParallelAction(
                                                        new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                                ),
                                                new SleepAction(0.07),
                                                new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                                new SleepAction(0.07)
                                        )
                                )
                        ),
                        new ParallelAction(
                                depositSecondSubAndGoToThird,
                                new SequentialAction(
                                        new ParallelAction(
                                                new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_TRANSFER),
                                                new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                                                new SetClawWristPositionAction(95),
                                                new SequentialAction(
                                                        new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() > -43)),
                                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                        new SleepAction(0.07),
                                                        new ParallelAction(
                                                                new SetClawPitchPositionAction(CLAW_PITCH_BACK_OFF),
                                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_BACK_OFF)
                                                        ),
                                                        new ParallelAction(
                                                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET),
                                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                                                new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                                                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                new SetClawWristPositionAction(95 + values[2][2])
                                                        )
                                                )
                                        ),
                                        new SleepUntilTrue(() -> (
                                                (-60 - drive.pose.position.x) * (-60 - drive.pose.position.x) + (-56 - drive.pose.position.y) * (-56 - drive.pose.position.y) < 9
                                        )),
                                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                                        new SleepAction(0.4),
                                        new ParallelAction(
                                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER)
                                        ),
                                        new SequentialAction(
                                                new SleepUntilTrue(() -> (
                                                        Math.abs(Math.toDegrees(drive.pose.heading.toDouble()) - 0) < 15
                                                )),
                                                new SetExtendoTargetAction(EXTENDO_EXTENDED),
                                                new SleepUntilTrue(()->(
                                                        Math.sqrt((drive.pose.position.x-(-24+1.1811*values[2][0]-29.7))*(drive.pose.position.x-(-24+1.1811*values[2][0]-29.7))+
                                                                (drive.pose.position.y-(-24+1.1811*values[2][1]))*(drive.pose.position.y-(-24+1.1811*values[2][1])))<6
                                                )),
                                                new ParallelAction(
                                                        new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                                ),
                                                new SleepAction(0.07),
                                                new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                                new SleepAction(0.07)
                                        )
                                )
                        ),
                        new ParallelAction(
                                depositThirdSubAndGoToFourth,
                                new SequentialAction(
                                        new ParallelAction(
                                                new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_TRANSFER),
                                                new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                                                new SetClawWristPositionAction(95),
                                                new SequentialAction(
                                                        new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() > -43)),
                                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                        new SleepAction(0.07),
                                                        new ParallelAction(
                                                                new SetClawPitchPositionAction(CLAW_PITCH_BACK_OFF),
                                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_BACK_OFF)
                                                        ),
                                                        new ParallelAction(
                                                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET),
                                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                                                new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                                                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                                new SetClawWristPositionAction(95 + values[3][2])
                                                        )
                                                )
                                        ),
                                        new SleepUntilTrue(() -> (
                                                (-60 - drive.pose.position.x) * (-60 - drive.pose.position.x) + (-56 - drive.pose.position.y) * (-56 - drive.pose.position.y) < 9
                                        )),
                                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                                        new SleepAction(0.4),
                                        new ParallelAction(
                                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER)
                                        ),
                                        new SequentialAction(
                                                new SleepUntilTrue(() -> (
                                                        Math.abs(Math.toDegrees(drive.pose.heading.toDouble()) - 0) < 15
                                                )),
                                                new SetExtendoTargetAction(EXTENDO_EXTENDED),
                                                new SleepUntilTrue(()->(
                                                        Math.sqrt((drive.pose.position.x-(-24+1.1811*values[3][0]-29.7))*(drive.pose.position.x-(-24+1.1811*values[3][0]-29.7))+
                                                                (drive.pose.position.y-(-24+1.1811*values[3][1]))*(drive.pose.position.y-(-24+1.1811*values[3][1])))<6
                                                )),
                                                new ParallelAction(
                                                        new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                                ),
                                                new SleepAction(0.07),
                                                new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                                new SleepAction(0.07)
                                        )
                                )
                        ),
                        new ParallelAction(
                                goToFourthSubDeposit,
                                new SequentialAction(
                                        new ParallelAction(
                                                new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_TRANSFER),
                                                new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                                                new SetClawWristPositionAction(95),
                                                new SequentialAction(
                                                        new SleepUntilTrue(() -> (extendoPitch.getCurrentPosition() > -43)),
                                                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                                        new SleepAction(0.07),
                                                        new ParallelAction(
                                                                new SetClawPitchPositionAction(CLAW_PITCH_BACK_OFF),
                                                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_BACK_OFF)
                                                        ),
                                                        new SetBucketSlidesTargetAction(BUCKET_SLIDES_HIGH_BUCKET)
                                                )

                                        )
                                )
                        ),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SleepAction(0.4),
                        new ParallelAction(
                                new SetBucketPositionAction(BUCKET_TRANSFER),
                                new SetBucketSlidesTargetAction(BUCKET_SLIDES_TRANSFER)
                        ),
                        new ParallelAction(
                                new SetClawPitchPositionAction(CLAW_PITCH_TRANSFER),
                                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER)
                        )
                )
        );
    }

    public static class SleepUntilTrue implements Action {
        public LambdaInterfaces.Condition condition;
        public double timeout;
        private ElapsedTime timeOutTimer = null;
        public boolean isStart = true;
        public LambdaInterfaces.Condition returnCondition;
        public SleepUntilTrue(LambdaInterfaces.Condition condition, double timeout){
            this.condition=condition;
            this.timeout=timeout;
            if (timeout!=Double.POSITIVE_INFINITY) {
                returnCondition = () -> (condition.call() && timeOutTimer.time() < timeout);
            }
            else{
                returnCondition = condition;
            }
        }
        public SleepUntilTrue(LambdaInterfaces.Condition condition){
            this(condition, Double.POSITIVE_INFINITY);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isStart){
                isStart=false;
                if (this.timeout!=Double.POSITIVE_INFINITY) {
                    timeOutTimer = new ElapsedTime();
                }
            }
            return !returnCondition.call();
        }
    }
    public class MotorPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            double extendoPower = (extendoTarget - extendo.getCurrentPosition()) * 0.013;
            double extendoPitchPower = (extendoPitchTarget - extendoPitch.getCurrentPosition()) * 0.01;
            double bucketSlidesPower = (bucketSlidesTarget - bucketSlides.getCurrentPosition()) * 0.015;

            extendo.setPower(extendoPower);
            extendoPitch.setPower(extendoPitchPower);
            bucketSlides.setPower(bucketSlidesPower);
            bucket.setPosition(bucketPos/270);

            telemetry.addData("extendo target", extendoTarget);
            telemetry.addData("extendoPitch target", extendoPitchTarget);
            telemetry.addData("bucketSlidesTarget target", bucketSlidesTarget);

            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendoPitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("bucketSlides position", bucketSlides.getCurrentPosition());

            telemetry.addData("x",drive.pose.position.x);
            telemetry.addData("y",drive.pose.position.y);
            telemetry.addData("heading",drive.pose.heading.toDouble());
            telemetry.update();

            return true;
        }
    }

    public class SetExtendoTargetAction implements Action {
        int target;
        public SetExtendoTargetAction(int target){
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            extendoTarget = target;
            return !(Math.abs(extendoTarget - extendo.getCurrentPosition()) < 30);
        }
    }

    public class SetExtendoPitchTargetAction implements Action{
        int target;
        public SetExtendoPitchTargetAction(int target){
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            extendoPitchTarget = target;
            return !(Math.abs(extendoPitchTarget - extendoPitch.getCurrentPosition()) < 30);
        }
    }

    public class SetBucketSlidesTargetAction implements Action{
        int target;
        public SetBucketSlidesTargetAction(int target){
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            bucketSlidesTarget = target;
            return !(Math.abs(bucketSlidesTarget - bucketSlides.getCurrentPosition()) < 30);
        }
    }

    public class SetClawPitchPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetClawPitchPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                clawPitchLeft.setPosition(position / 270);
                clawPitchRight.setPosition(position / 270);
                time=Math.abs(position-clawPitchLeft.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetInnerClawPitchPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetInnerClawPitchPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                innerClawPitch.setPosition(position / 270);
                time=Math.abs(position-innerClawPitch.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetClawFingersPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetClawFingersPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                clawFingers.setPosition(position / 180);
                time=Math.abs(position-clawFingers.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetBucketPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetBucketPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                bucket.setPosition(position / 270);
                bucketPos=position;
                time=Math.abs(position-bucket.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetClawWristPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetClawWristPositionAction(double position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart){
                isStart=false;
                clawWrist.setPosition(position / 270);
                time=Math.abs(position-clawWrist.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds()> time);
        }
    }
    Action goToFirstPickup;
    Action goToFirstDeposit;
    Action goToSecondPickup;
    Action goToSecondDeposit;
    Action goToThirdPickup;
    Action goToThirdDeposit;
    Action goToFirstSub;
    Action depositFirstSubAndGoToSecond;
    Action depositSecondSubAndGoToThird;
    Action depositThirdSubAndGoToFourth;
    Action goToFourthSubDeposit;

    Action path = null;

    @Override
    public void init() {}

    @Override
    public void init_loop() {

        if (!endOfInit) {

            if (gamepad1.dpad_up && selectedRow > 0 && !dpadPressed) {
                selectedRow -= 1;
            } else if (gamepad1.dpad_down && selectedRow < 3 && !dpadPressed) {
                selectedRow += 1;
            } else if (gamepad1.dpad_left && selectedColumn > 0 && !dpadPressed) {
                selectedColumn -= 1;
            } else if (gamepad1.dpad_right && selectedColumn < 2 && !dpadPressed) {
                selectedColumn += 1;
            }
            dpadPressed = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;

            if (!labelArray[selectedRow][selectedColumn].startsWith("[")) {
                labelArray[selectedRow][selectedColumn] = "[" + labelArray[selectedRow][selectedColumn] + "]";
            }
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 3; y++) {
                    if (x != selectedRow || y != selectedColumn) {
                        if (labelArray[x][y].startsWith("[")) {
                            labelArray[x][y] = (String) labelArray[x][y].subSequence(1, labelArray[x][y].length() - 1);
                        }
                    }
                }
            }

            if (gamepad1.right_trigger > 0) {
                if (selectedColumn != 2) {
                    values[selectedRow][selectedColumn] += 0.01;
                } else {
                    if (values[selectedRow][selectedColumn] < 89.95) {
                        values[selectedRow][selectedColumn] += 0.05;
                    } else {
                        values[selectedRow][selectedColumn] = 90;
                    }
                }
            } else if (gamepad1.left_trigger > 0) {
                if (selectedColumn != 2) {
                    values[selectedRow][selectedColumn] -= 0.01;
                } else {
                    if (values[selectedRow][selectedColumn] > -89.95) {
                        values[selectedRow][selectedColumn] -= 0.05;
                    } else {
                        values[selectedRow][selectedColumn] = -90;
                    }
                }
            } else if (gamepad1.right_bumper) {
                if (selectedColumn != 2) {
                    values[selectedRow][selectedColumn] += 0.001;
                } else {
                    if (values[selectedRow][selectedColumn] < 89.995) {
                        values[selectedRow][selectedColumn] += 0.005;
                    } else {
                        values[selectedRow][selectedColumn] = 90;
                    }
                }
            } else if (gamepad1.left_bumper) {
                if (selectedColumn != 2) {
                    values[selectedRow][selectedColumn] -= 0.001;
                } else {
                    if (values[selectedRow][selectedColumn] > -89.995) {
                        values[selectedRow][selectedColumn] -= 0.005;
                    } else {
                        values[selectedRow][selectedColumn] = -90;
                    }
                }
            }
            if (gamepad1.a) {
                endOfInit = true;
            }

            telemetry.addData(labelArray[0][0] + ": " + String.format("%.3f", values[0][0]) + ", " + labelArray[0][1] + ": " + String.format("%.3f", values[0][1]) + ", " + labelArray[0][2] + ": " + String.format("%.3f", values[0][2]), "");
            telemetry.addData(labelArray[1][0] + ": " + String.format("%.3f", values[1][0]) + ", " + labelArray[1][1] + ": " + String.format("%.3f", values[1][1]) + ", " + labelArray[1][2] + ": " + String.format("%.3f", values[1][2]), "");
            telemetry.addData(labelArray[2][0] + ": " + String.format("%.3f", values[2][0]) + ", " + labelArray[2][1] + ": " + String.format("%.3f", values[2][1]) + ", " + labelArray[2][2] + ": " + String.format("%.3f", values[2][2]), "");
            telemetry.addData(labelArray[3][0] + ": " + String.format("%.3f", values[3][0]) + ", " + labelArray[3][1] + ": " + String.format("%.3f", values[3][1]) + ", " + labelArray[3][2] + ": " + String.format("%.3f", values[3][2]), "");
            telemetry.addData("Kindly press A to avoid an existential crisis.","");
            telemetry.addData("I mean it.","");
            telemetry.addData("PRESSAPRESSAKUGFUYGKUDYAUYGKUDAF!!!!!!!!","");
            telemetry.update();
        }
        else if (firstLoop){
            firstLoop=false;
            generatePath();
        }
    }
    @Override
    public void loop() {
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendoPitch = hardwareMap.get(DcMotorEx.class, "extendoPitch");
        bucketSlides = hardwareMap.get(DcMotorEx.class, "bucketSlides");

        clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        innerClawPitch = hardwareMap.servo.get("innerClawPitch");
        clawFingers = hardwareMap.servo.get("clawFingers");
        bucket = hardwareMap.servo.get("bucket");
        clawWrist = hardwareMap.servo.get("clawWrist");

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        bucketSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        clawPitchRight.setDirection(Servo.Direction.REVERSE);
        innerClawPitch.setDirection(Servo.Direction.REVERSE);

        bucket.setPosition((double) 36 / 270);

        if (Objects.isNull(path)){
            generatePath();
        }

        Actions.runBlocking(
                path
        );
    }
}

