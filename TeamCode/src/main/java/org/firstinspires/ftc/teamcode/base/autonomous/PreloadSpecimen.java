package org.firstinspires.ftc.teamcode.base.autonomous;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.base.motorcontrol.PID;

@Autonomous
public class PreloadSpecimen extends LinearOpMode {
    PID extendoPID;
    PID extendoPitchPID;
    PID bucketSlidesPID;
    PinpointDrive drive;
    public double robotLength = 15.364;
    public double robotWidth  = 14.375;

    public final int SERVO_SPEED=555;
    public final int EXTENDO_RETRACTED = 0;
    public final int EXTENDO_SCORE_SPECIMEN_UP = 480;
    public final int EXTENDO_SCORE_SPECIMEN_DOWN = 300;
    public final int EXTENDO_PITCH_TRANSFER = 0;
    public final int EXTENDO_PITCH_SCORE_SPECIMEN = 0;
    public final int EXTENDO_PITCH_PICK_UP = -1030;
    public final int EXTENDO_PITCH_GRAB_SPECIMEN = -960;
    public final int BUCKET_SLIDES_HIGH_BUCKET = 1030;
    public final int BUCKET_SLIDES_TRANSFER = 0;
    public final int BUCKET_SLIDES_SCORING_SPECIMEN = 350;
    public final int CLAW_FINGERS_OPEN = 86;
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
    public final int INNER_CLAW_PITCH_TRANSFER = 200;
    public final int INNER_CLAW_PITCH_BACK_OFF = 100;
    public final int INNER_CLAW_PITCH_GRAB_SPECIMEN = 78;
    public final int INNER_CLAW_PITCH_SCORE_SPECIMEN = 82;
    public final int BUCKET_TRANSFER = 46;
    public final int BUCKET_DEPOSIT = 158;
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
    public class MotorPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            double extendoPower = (extendoTarget - extendo.getCurrentPosition()) * 0.014;
            double extendoPitchPower = (extendoPitchTarget - extendoPitch.getCurrentPosition()) * 0.01;
            double bucketSlidesPower = (bucketSlidesTarget - bucketSlides.getCurrentPosition()) * 0.015;

            extendo.setPower(extendoPower);
            extendoPitch.setPower(extendoPitchPower);
            bucketSlides.setPower(bucketSlidesPower);

            telemetry.addData("extendo target", extendoTarget);
            telemetry.addData("extendoPitch target", extendoPitchTarget);
            telemetry.addData("bucketSlidesTarget target", bucketSlidesTarget);

            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendoPitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("bucketSlides position", bucketSlides.getCurrentPosition());
            telemetry.update();

            return true;
        }
    }
    public Action intakePickUpSample(int extendoPosition){
        return new SequentialAction(
            new ParallelAction(
                    new SetClawWristPositionAction(CLAW_WRIST_DEFAULT),
                    new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN)
            ),
            new SetExtendoTargetAction(extendoPosition),
                new ParallelAction(
                        new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                ),
        new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED)
        );
    }

    public Action intakePickUpHover(){
        return new SequentialAction(
                new ParallelAction(
                        new SetClawWristPositionAction(CLAW_WRIST_DEFAULT),
                        new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER),
                        new SetClawFingersPositionAction(CLAW_FINGERS_OPEN)
                )
        );
    }
    public Action pickUpSideSpecimenPosition(){
        return new SequentialAction(
                new ParallelAction(
                        new SetClawWristPositionAction(CLAW_WRIST_DEFAULT),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_GRAB_SPECIMEN),
                        new SetExtendoTargetAction(EXTENDO_RETRACTED),
                        new SetClawPitchPositionAction(CLAW_PITCH_GRAB_SPECIMEN),
                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_GRAB_SPECIMEN)
                ),
                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN)
        );
    }

    public Action scoreSpecimenUpPosition(){
        return new SequentialAction(
                new ParallelAction(
                        new SetClawWristPositionAction(CLAW_WRIST_DEFAULT),
                        new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_SCORE_SPECIMEN),
                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_SCORE_SPECIMEN),
                        new SetClawPitchPositionAction(CLAW_PITCH_SCORE_SPECIMEN)
                ),
                new SetBucketSlidesTargetAction(BUCKET_SLIDES_SCORING_SPECIMEN),
                new SetExtendoTargetAction(EXTENDO_SCORE_SPECIMEN_UP)
        );
    }

    public Action pickUpSideSpecimen(){
        return new SequentialAction(
                new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                new ParallelAction(
                        new SetClawWristPositionAction(CLAW_WRIST_DEFAULT),
                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                        new SetBucketSlidesTargetAction(BUCKET_SLIDES_SCORING_SPECIMEN)
                ),
                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_TRANSFER),
                new ParallelAction(
                        new SetExtendoPitchTargetAction(EXTENDO_PITCH_SCORE_SPECIMEN),
                        new SequentialAction(
                                new SleepAction(0.3),
                                new ParallelAction(
                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_SCORE_SPECIMEN),
                                        new SetClawPitchPositionAction(CLAW_PITCH_SCORE_SPECIMEN)
                                )
                        )
                )
        );
    }

    public Action scoreSpecimenDownPosition(){
        return new SequentialAction(
                new SetExtendoTargetAction(EXTENDO_SCORE_SPECIMEN_DOWN)
        );
    }

    public class SetExtendoTargetAction implements Action{
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

    @Override
    public void runOpMode(){

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

        drive = new PinpointDrive(hardwareMap, new Pose2d((robotWidth / 2), -70 + (robotLength / 2), Math.toRadians(90)));

        extendoPID = new PID();
        extendoPitchPID = new PID();
        bucketSlidesPID = new PID();

        Action fourSpecimenPathing1 = drive.actionBuilder(new Pose2d((robotWidth / 2), -70 + (robotLength / 2), Math.toRadians(90)))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(8,-33), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing2 = drive.actionBuilder(new Pose2d(8, -33, Math.toRadians(90)))
                // Go to sample 1 sample zone
                .strafeToLinearHeading(new Vector2d(38,-44), Math.toRadians(40))
                .build();
        Action fourSpecimenPathing3 = drive.actionBuilder(new Pose2d(38, -44, Math.toRadians(40)))
                // Rotate towards observation zone 1st time
                .turnTo(Math.toRadians(-45))
                .build();
        Action fourSpecimenPathing4 = drive.actionBuilder(new Pose2d(38, -44, Math.toRadians(-45)))
                // Rotate to sample 2 in sample zone
                .strafeToLinearHeading(new Vector2d(43,-40), Math.toRadians(48))
                .build();
        Action fourSpecimenPathing5 = drive.actionBuilder(new Pose2d(43, -40, Math.toRadians(48)))
                // Rotate towards observation zone 2nd time
                .turnTo(Math.toRadians(-70))
                .build();
        Action fourSpecimenPathing6 = drive.actionBuilder(new Pose2d(43, -40, Math.toRadians(-70)))
                // Rotate towards sample 3 in sample zone
                .strafeToLinearHeading(new Vector2d(52,-40), Math.toRadians(40))
                .build();
        Action fourSpecimenPathing7 = drive.actionBuilder(new Pose2d(52, -40, Math.toRadians(40)))
                // Rotate towards observation zone 3rd time
                .strafeToLinearHeading(new Vector2d(52, -45), Math.toRadians(-100))
                .build();
        Action fourSpecimenPathing8 = drive.actionBuilder(new Pose2d(43, -40, Math.toRadians(-70)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-49), Math.toRadians(-90))
                .build();
        Action fourSpecimenPathing9 = drive.actionBuilder(new Pose2d(34, -49, Math.toRadians(-90)))
                // Score second specimen
                .strafeToLinearHeading(new Vector2d(4,-36), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing10 = drive.actionBuilder(new Pose2d(4, -36, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-49), Math.toRadians(-90))
                .build();
        Action fourSpecimenPathing11 = drive.actionBuilder(new Pose2d(34, -49, Math.toRadians(-90)))
                // Score third specimen
                .strafeToLinearHeading(new Vector2d(0,-36), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing12 = drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(34,-49), Math.toRadians(-90))
                .build();
        Action fourSpecimenPathing13 = drive.actionBuilder(new Pose2d(34, -49, Math.toRadians(-90)))
                // Score fourth specimen
                .strafeToLinearHeading(new Vector2d(-4,-36), Math.toRadians(90))
                .build();
        Action fourSpecimenPathing14 = drive.actionBuilder(new Pose2d(-4, -36, Math.toRadians(90)))
                // Park
                .strafeToLinearHeading(new Vector2d(34,-62), Math.toRadians(90))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new MotorPID(),
                        new SequentialAction(
                                new ParallelAction(
                                        new SetBucketSlidesTargetAction(BUCKET_SLIDES_SCORING_SPECIMEN),
                                        new SetBucketSlidesTargetAction(BUCKET_DEPOSIT),
                                        new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                        new SetClawPitchPositionAction(CLAW_PITCH_SCORE_SPECIMEN),
                                        new SetClawWristPositionAction(CLAW_WRIST_DEFAULT),
                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_SCORE_SPECIMEN),
                                        new SetBucketPositionAction(BUCKET_DEPOSIT),
                                        fourSpecimenPathing1,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_SCORE_SPECIMEN),
                                                new SetExtendoTargetAction(EXTENDO_SCORE_SPECIMEN_UP)
                                        )
                                ),
                                new SleepAction(0.3),
                                new SetExtendoTargetAction(EXTENDO_SCORE_SPECIMEN_DOWN),
                                new SleepAction(0.3),
                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                new ParallelAction(
                                        fourSpecimenPathing2,
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                new ParallelAction(
                                                        new SetExtendoTargetAction(EXTENDO_RETRACTED),
                                                        new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                        new SetClawWristPositionAction(50),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER)
                                                ),
                                                new SleepAction(0.5),
                                                new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                                                new SleepAction(0.5),
                                                new SetExtendoTargetAction(EXTENDO_SCORE_SPECIMEN_DOWN)

                                        )
                                ),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                ),

                                new SleepAction(0.5),
                                new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                new SleepAction(0.5),
                                fourSpecimenPathing3,
                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),

                                new ParallelAction(
                                        fourSpecimenPathing4,
                                        new SequentialAction(
                                                //new SleepAction(0.5),
                                                new ParallelAction(
                                                        new SetClawPitchPositionAction(CLAW_PITCH_HOVER),
                                                        new SetClawWristPositionAction(50),
                                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_HOVER)
                                                )
                                        )
                                ),
                                new ParallelAction(
                                        new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                                        new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP)
                                ),
                                new SleepAction(0.5),
                                new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED),
                                new SleepAction(0.5),
                                fourSpecimenPathing5,
                                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN),
                                new ParallelAction(
                                        pickUpSideSpecimenPosition(),
                                        fourSpecimenPathing8
                                ),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        pickUpSideSpecimen(),
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                fourSpecimenPathing9
                                        )
                                ),
                                scoreSpecimenUpPosition(),
                                new SleepAction(0.5),
                                scoreSpecimenDownPosition(),

                                new ParallelAction(
                                        pickUpSideSpecimenPosition(),
                                        fourSpecimenPathing10
                                ),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        pickUpSideSpecimen(),
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                fourSpecimenPathing11
                                        )
                                ),
                                scoreSpecimenUpPosition(),
                                new SleepAction(0.5),
                                scoreSpecimenDownPosition(),
                                new ParallelAction(
                                        pickUpSideSpecimenPosition(),
                                        fourSpecimenPathing12
                                ),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        pickUpSideSpecimen(),
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                fourSpecimenPathing13
                                        )
                                ),
                                scoreSpecimenUpPosition(),
                                new SleepAction(0.5),
                                scoreSpecimenDownPosition()









                        )

                )

        );
    }
}
