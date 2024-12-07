package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.ServoEnum;

// Contains all non-default roadrunner actions that are used in our autonomous routines
public class CustomActions {
    RobotState state;
    HardwareConfig hw;
    PinpointDrive drive;
    public CustomActions(RobotState state, HardwareMap hardwareMap) {
        hw = HardwareConfig.getHardwareConfig();
        this.state = state;
        drive = new PinpointDrive(hardwareMap, new Pose2d(-42, -62.5, Math.toRadians(270)));
    }

    public class SetMotorTargetAction implements Action {
        MotorEnum motorEnum;

        // In ticks (pulses)
        int target;

        public SetMotorTargetAction (MotorEnum motorEnum, int target) {
            this.motorEnum = motorEnum;
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            state.setMotorTarget(motorEnum, target);
            return false;
        }
    }

    public class SetServoPositionAction  implements Action {
        ServoEnum servoEnum;

        // In degrees
        double position;

        public SetServoPositionAction (ServoEnum servoEnum, double position) {
            this.servoEnum = servoEnum;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            state.setServoPosition(servoEnum, position);
            return false;
        }
    }

    public class GlobalPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            HardwareConfig hw = HardwareConfig.getHardwareConfig();
            hw.getMotorConfig(MotorEnum.EXTENDO).motor.setPower
                    ((state.getMotorTarget(MotorEnum.EXTENDO) - hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition()) * hw.getMotorConfig(MotorEnum.EXTENDO).kP);

            hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.setPower
                    ((state.getMotorTarget(MotorEnum.EXTENDO_PITCH)- hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.getCurrentPosition()) * hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).kP);

            hw.getMotorConfig(MotorEnum.HANG).motor.setPower
                    ((state.getMotorTarget(MotorEnum.HANG) - hw.getMotorConfig(MotorEnum.HANG).motor.getCurrentPosition()) * hw.getMotorConfig(MotorEnum.HANG).kP);

            hw.getMotorConfig(MotorEnum.BUCKET_SLIDES).motor.setPower
                    ((state.getMotorTarget(MotorEnum.BUCKET_SLIDES) - hw.getMotorConfig(MotorEnum.BUCKET_SLIDES).motor.getCurrentPosition()) * hw.getMotorConfig(MotorEnum.BUCKET_SLIDES).kP);

            hw.getServoConfig(ServoEnum.CLAW_PITCH_LEFT).servo.setPosition
                    (state.getServoPosition(ServoEnum.CLAW_PITCH_LEFT) / hw.getServoConfig(ServoEnum.CLAW_PITCH_LEFT).maxServoPosition);

            hw.getServoConfig(ServoEnum.CLAW_PITCH_RIGHT).servo.setPosition
                    (state.getServoPosition(ServoEnum.CLAW_PITCH_RIGHT) / hw.getServoConfig(ServoEnum.CLAW_PITCH_RIGHT).maxServoPosition);

            hw.getServoConfig(ServoEnum.BUCKET).servo.setPosition
                    (state.getServoPosition(ServoEnum.BUCKET) / hw.getServoConfig(ServoEnum.BUCKET).maxServoPosition);

            hw.getServoConfig(ServoEnum.CLAW_FINGERS).servo.setPosition
                    (state.getServoPosition(ServoEnum.CLAW_FINGERS) / hw.getServoConfig(ServoEnum.CLAW_FINGERS).maxServoPosition);

            hw.getServoConfig(ServoEnum.CLAW_WRIST).servo.setPosition
                    (state.getServoPosition(ServoEnum.CLAW_WRIST) / hw.getServoConfig(ServoEnum.CLAW_WRIST).maxServoPosition);

            return true;
        }
    }
    public class UpdateTelemetry implements Action{
        Telemetry telemetry;
        public UpdateTelemetry(Telemetry telemetry){
            this.telemetry = telemetry;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            HardwareConfig hw = HardwareConfig.getHardwareConfig();

            // Motor telemetry
            telemetry.addData("extendo position", hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition());
            telemetry.addData("extendo target", state.getMotorTarget(MotorEnum.EXTENDO));
            telemetry.addData("extendo pitch position", hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.getCurrentPosition());
            telemetry.addData("extendo pitch target", state.getMotorTarget(MotorEnum.EXTENDO_PITCH));
            telemetry.addData("hang pos", hw.getMotorConfig(MotorEnum.HANG).motor.getCurrentPosition());
            telemetry.addData("hang target", state.getMotorTarget(MotorEnum.HANG));
            telemetry.addData("bucketSlides pos", hw.getMotorConfig(MotorEnum.BUCKET_SLIDES).motor.getCurrentPosition());
            telemetry.addData("bucketSlides target", state.getMotorTarget(MotorEnum.BUCKET_SLIDES));

            // Servo telemetry (in degrees)
            telemetry.addData("left claw pitch position", hw.getServoConfig(ServoEnum.CLAW_PITCH_LEFT).servo.getPosition());
            telemetry.addData("right claw pitch position",hw.getServoConfig(ServoEnum.CLAW_PITCH_RIGHT).servo.getPosition());
            telemetry.addData("claw finger position",  hw.getServoConfig(ServoEnum.CLAW_FINGERS).servo.getPosition());
            telemetry.addData("claw wrist position",  hw.getServoConfig(ServoEnum.CLAW_WRIST).servo.getPosition());
            telemetry.addData("bucket position", hw.getServoConfig(ServoEnum.BUCKET).servo.getPosition());

            // Misc telemetry
            telemetry.addData("pinpoint heading", hw.pinpoint.pinpoint.getYawScalar());
            telemetry.addData("Control hub IMU heading", hw.imu.imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("pinpoint x", hw.pinpoint.pinpoint.getPosX());
            telemetry.addData("pinpoint y", hw.pinpoint.pinpoint.getPosY());
            telemetry.update();
            return true;
        }
    }
    public class MoveToBucketAndScore implements Action{
        Pose2d initialPose;
        public MoveToBucketAndScore(Pose2d initialPose){
            this.initialPose = initialPose;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            // Retract extendo, transfer and move to scoring pos
            new SequentialAction(
                    new ParallelAction(
                            // Move to scoring position
                            drive.actionBuilder(initialPose)
                                    // Score bucket
                                    .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                                    .build(),
                            new SequentialAction(
                                    // Retract extendo
                                    setExtendoTarget(0),
                                    // Claw pitch transfer position
                                    setClawPitchPosition(200),
                                    new SleepAction(0.3),
                                    // Extendo pitch transfer position
                                    setExtendoPitchTarget(0),
                                    new SleepAction(0.8),
                                    // Open claw fully bc bucketSlides coming down later
                                    setClawFingerPosition(120)
                            )

                    ),
                    // Wait for sample to settle in bucket
                    new SleepAction(0.5),
                    // Move bucketSlides up to scoring position
                    setBucketSlidesTarget(1100),
                    new SleepAction(0.6),
                    // Rotate bucket to score
                    setBucketPosition(205),
                    new SleepAction(0.7),
                    // Move bucket back to default position
                    setBucketPosition(85),
                    // Avoid level 4 hang
                    new SleepAction(0.4),
                    // Move bucketSlides back to down position
                    setBucketSlidesTarget(0)
            );
            return false;
        }
    }
    public class ScorePreloadSpecimen implements Action{
        Pose2d initialPose;
        public ScorePreloadSpecimen(Pose2d initialPose){
            this.initialPose = initialPose;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            new SequentialAction(
                    // Close claw
                    setClawFingerPosition(39),
                    setBucketPosition(205),
                    // Drive and prepare extendo pitch
                    new SleepAction(0.5),
                    new ParallelAction(
                            drive.actionBuilder(new Pose2d(-42, -62.5, Math.toRadians(270)))
                                    // Score preload
                                    .strafeToLinearHeading(new Vector2d(-7, -47), Math.toRadians(270))
                                    .build(),
                            setExtendoPitchTarget(400),
                            setClawPitchPosition(104)
                    ),
                    new SleepAction(0.5),
                    // Raise extendo, lower extendoPitch slightly, lower claw pitch slightly
                    new ParallelAction(
                            setExtendoTarget(500),
                            setExtendoPitchTarget(400),
                            setClawPitchPosition(100)
                    ),
                    new SleepAction(0.5),
                    // Lower extendo
                    setExtendoPitchTarget(750),
                    new SleepAction(1),
                    // Open claw
                    setClawFingerPosition(90),
                    new SleepAction(0.5)
            );
            return false;
        }
    }

    public Action updateTelemetry(Telemetry telemetry) {return new UpdateTelemetry(telemetry);}
    public Action globalPID() {return new GlobalPID();}
    public Action setExtendoTarget(int target) {return new SetMotorTargetAction(MotorEnum.EXTENDO, target);}
    public Action setExtendoPitchTarget(int target) {return new SetMotorTargetAction(MotorEnum.EXTENDO_PITCH, target);}
    public Action setBucketSlidesTarget(int target) {return new SetMotorTargetAction(MotorEnum.BUCKET_SLIDES, target);}
    public Action setClawPitchPosition(double degrees) {return new ParallelAction(new SetServoPositionAction(ServoEnum.CLAW_PITCH_LEFT, degrees), new SetServoPositionAction(ServoEnum.CLAW_PITCH_RIGHT, degrees));}
    public Action setClawFingerPosition(double degrees) {return new SetServoPositionAction(ServoEnum.CLAW_FINGERS, degrees);}
    //public Action setClawWristPosition(double degrees) {return new SetServoPositionAction(ServoEnum.CLAW_WRIST, degrees);}
    public Action setBucketPosition(double degrees) {return new SetServoPositionAction(ServoEnum.BUCKET, degrees);}
    public Action scorePreloadSpecimen(Pose2d initialPose) {return new ScorePreloadSpecimen(initialPose);}
    public Action moveToBucketAndScore(Pose2d initialPose) {return new MoveToBucketAndScore(initialPose);}
    Action onePlusThreeBucket1 = drive.actionBuilder(new Pose2d(-42, -62.5, Math.toRadians(270)))
            // Score preload
            .strafeToLinearHeading(new Vector2d(-7, -47), Math.toRadians(270))
            .build();
    Action onePlusThreeBucket2 = drive.actionBuilder(new Pose2d(-7, -47, Math.toRadians(270)))
            // Go to sample zone 1
            .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(270))
            .build();
    Action onePlusThreeBucket3 = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(270)))
            // Score bucket
            .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
            .build();
    Action onePlusThreeBucket4 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
            // Sample zone 2
            .strafeToLinearHeading(new Vector2d(-63, -52), Math.toRadians(273))
            .build();
    Action onePlusThreeBucket5 = drive.actionBuilder(new Pose2d(-63, -52, Math.toRadians(273)))
            // Score bucket
            .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
            .build();
    Action onePlusThreeBucket6 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
            // sample zone 3
            .strafeToLinearHeading(new Vector2d(-67, -52), Math.toRadians(285))
            .build();
    Action onePlusThreeBucket7 = drive.actionBuilder(new Pose2d(-67, -52, Math.toRadians(285)))
            // turn and score bucket
            .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
            .build();
    Action onePlusThreeBucket8 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(225)))
            // park
            .strafeToLinearHeading(new Vector2d(-44, -6), Math.toRadians(0))
            .strafeToLinearHeading(new Vector2d(-24, -6), Math.toRadians(0))
            .build();
}
