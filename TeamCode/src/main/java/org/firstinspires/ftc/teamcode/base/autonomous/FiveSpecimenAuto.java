package org.firstinspires.ftc.teamcode.base.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.motorcontrol.PID;

@Autonomous
public class FiveSpecimenAuto extends LinearOpMode {
    PID extendoPID;
    PID extendoPitchPID;
    PinpointDrive drive;
    public double robotLength = 15.364;
    public double robotWidth  = 14.375;
    public final int EXTENDO_PITCH_TRANSFER = 0;
    public final int EXTENDO_PITCH_PICK_UP = -960;
    public final int BUCKET_SLIDES_HIGH_BUCKET = 1030;
    public final int BUCKET_SLIDES_TRANSFER = 0;
    public final int CLAW_FINGERS_OPEN = 20;
    public final int CLAW_FINGERS_CLOSED = 92;
    public final int CLAW_WRIST_DEFAULT = 95;
    public final int CLAW_PITCH_PICK_UP = 22;
    public final int CLAW_PITCH_HOVER = 73;
    public final int CLAW_PITCH_TRANSFER = 115;
    public final int CLAW_PITCH_BACK_OFF = 77;
    public final int INNER_CLAW_PITCH_PICK_UP = 62;
    public final int INNER_CLAW_PITCH_HOVER = 0;
    public final int INNER_CLAW_PITCH_TRANSFER = 186;
    public final int INNER_CLAW_PITCH_BACK_OFF = 170;
    public final int BUCKET_TRANSFER = 46;
    public final int BUCKET_DEPOSIT = 158;

    // Specimen stuff
    public final int EXTENDO_SCORE_SPECIMEN = 800;
    public final int EXTENDO_PITCH_SCORE_SPECIMEN = -620;
    public final int CLAW_PITCH_SCORE_SPECIMEN = 127;
    public final int INNER_CLAW_PITCH_SCORE_SPECIMEN = 179;

    public final int EXTENDO_RETRACTED = 0;
    public final int EXTENDO_PITCH_GRAB_SPECIMEN = -960;
    public final int CLAW_PITCH_GRAB_SPECIMEN = 151;
    public final int INNER_CLAW_PITCH_GRAB_SPECIMEN = 73;
    DcMotorEx extendo;
    DcMotorEx extendoPitch;
    Servo clawPitchLeft;
    Servo clawPitchRight;
    Servo innerClawPitch;
    Servo clawFingers;
    int extendoTarget;
    int extendoPitchTarget;
    public class MotorPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            double extendoPower = extendoPID.getPIDOutput(extendo, extendoTarget, 0.014, 0, 0.0032);
            double extendoPitchPower = extendoPitchPID.getPIDOutput(extendoPitch, extendoTarget, 0.01, 0, 0.0003);

            extendo.setPower(extendoPower);
            extendoPitch.setPower(extendoPitchPower);
            return true;
        }
    }
    public Action intakePickUpSample(int extendoPosition){
        return new SequentialAction(
        new ParallelAction(
                new SetExtendoPitchTargetAction(EXTENDO_PITCH_PICK_UP),
                new SetClawPitchPositionAction(CLAW_PITCH_PICK_UP),
                new SetInnerClawPitchPositionAction(INNER_CLAW_PITCH_PICK_UP),
                new SetClawFingersPositionAction(CLAW_FINGERS_OPEN)
        ),
        new SetExtendoTargetAction(extendoPosition),
        new SetClawFingersPositionAction(CLAW_FINGERS_CLOSED)
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
            return !((extendoTarget - extendo.getCurrentPosition()) < 10);
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
            return !((extendoPitchTarget - extendoPitch.getCurrentPosition()) < 10);
        }
    }

    public class SetClawPitchPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        public SetClawPitchPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            timer.reset();
            clawPitchLeft.setPosition(position / 270);
            clawPitchRight.setPosition(position / 270);
            return !(timer.seconds() > 0.5);
        }
    }

    public class SetInnerClawPitchPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        public SetInnerClawPitchPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            timer.reset();
            innerClawPitch.setPosition(position / 270);
            return !(timer.seconds() > 0.5);
        }
    }

    public class SetClawFingersPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        public SetClawFingersPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            timer.reset();
            clawFingers.setPosition(position / 180);
            return !(timer.seconds() > 0.5);
        }
    }

    @Override
    public void runOpMode(){

        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendoPitch = hardwareMap.get(DcMotorEx.class, "extendoPitch");

        clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        innerClawPitch = hardwareMap.servo.get("innerClawPitch");
        clawFingers = hardwareMap.servo.get("clawFingers");

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        extendoPitch.setDirection(DcMotorSimple.Direction.FORWARD);

        clawPitchRight.setDirection(Servo.Direction.REVERSE);
        innerClawPitch.setDirection(Servo.Direction.REVERSE);

        drive = new PinpointDrive(hardwareMap, new Pose2d(-(robotWidth / 2), -70 + (robotLength / 2), 90));

        extendoPID = new PID();
        extendoPitchPID = new PID();

        Action onePlusThreeSpecimen1 = drive.actionBuilder(new Pose2d(-(robotWidth / 2), -70 + (robotLength / 2), 90))
                // Go to scoring zone first time
                .strafeToLinearHeading(new Vector2d(8,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen2 = drive.actionBuilder(new Pose2d(8, -46, Math.toRadians(90)))
                // Go to sample 1 sample zone
                .strafeToLinearHeading(new Vector2d(30.7,-45.1), Math.toRadians(50))
                .build();
        Action onePlusThreeSpecimen3 = drive.actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(50)))
                // Rotate towards observation zone 1st time
                .turnTo(Math.toRadians(-45))
                .build();
        Action onePlusThreeSpecimen4 = drive.actionBuilder(new Pose2d(30.7, -45.1, Math.toRadians(-45)))
                // Rotate to sample 2 in sample zone
                .strafeToLinearHeading(new Vector2d(40,-41), Math.toRadians(40))
                .build();
        Action onePlusThreeSpecimen5 = drive.actionBuilder(new Pose2d(40, -41, Math.toRadians(40)))
                // Rotate towards observation zone 2nd time
                .turnTo(Math.toRadians(-70))
                .build();
        Action onePlusThreeSpecimen6 = drive.actionBuilder(new Pose2d(40, -41, Math.toRadians(-70)))
                // Rotate towards sample 3 in sample zone
                .strafeToLinearHeading(new Vector2d(51,-41), Math.toRadians(40))
                .build();
        Action onePlusThreeSpecimen7 = drive.actionBuilder(new Pose2d(51, -41, Math.toRadians(40)))
                // Rotate towards observation zone 3rd time
                .turnTo(Math.toRadians(-100))
                .build();
        Action onePlusThreeSpecimen8 = drive.actionBuilder(new Pose2d(51, -41, Math.toRadians(-100)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(-45))
                .build();
        Action onePlusThreeSpecimen9 = drive.actionBuilder(new Pose2d(29, -52, Math.toRadians(-45)))
                // Score second specimen
                .strafeToLinearHeading(new Vector2d(4,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen10 = drive.actionBuilder(new Pose2d(4, -46, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(-45))
                .build();
        Action onePlusThreeSpecimen11 = drive.actionBuilder(new Pose2d(29, -52, Math.toRadians(-45)))
                // Score third specimen
                .strafeToLinearHeading(new Vector2d(0,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen12 = drive.actionBuilder(new Pose2d(0, -46, Math.toRadians(90)))
                // Go to pickup zone
                .strafeToLinearHeading(new Vector2d(29,-52), Math.toRadians(-45))
                .build();
        Action onePlusThreeSpecimen13 = drive.actionBuilder(new Pose2d(29, -52, Math.toRadians(-45)))
                // Score fourth specimen
                .strafeToLinearHeading(new Vector2d(-4,-46), Math.toRadians(90))
                .build();
        Action onePlusThreeSpecimen14 = drive.actionBuilder(new Pose2d(-4, -46, Math.toRadians(90)))
                // Park
                .strafeToLinearHeading(new Vector2d(34,-62), Math.toRadians(90))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new MotorPID(),
                        new SequentialAction(

                        )
                )
        );
    }

}
