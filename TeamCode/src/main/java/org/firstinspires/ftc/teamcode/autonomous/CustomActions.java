package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.ServoEnum;

// Contains all non-default roadrunner actions that are used in our autonomous routines
public class CustomActions {
    RobotState state;
    HardwareConfig hw;
    PinpointDrive drive;
    int extendoRetractedTarget = 0;
    int extendoScoringSpecimenTarget = 500;
    int extendoPitchPickUpTarget = 1350;
    int extendoPitchSpecimenUpTarget = 500;
    int extendoPitchSpecimenDownTarget = 800;
    int extendoPitchTransferTarget = 0;
    double clawFingersClosedPosition = 39.0;
    double clawFingersOpenPosition = 90.0;
    double clawFingersFullyOpenPosition = 120.0;
    double bucketWhenScoringSpecimenPosition = 200.0;
    double bucketScoringPosition = 205.0;
    double bucketTransferPosition = 85.0;
    double clawPitchPickUpPosition = 30.5;
    double clawPitchPitchTransferPosition = 200;
    double clawPitchSpecimenScoringPosition = 104;
    int bucketSlidesHighBasketTarget = 1100;
    int bucketSlidesDownTarget = 0;
    double clawWristDefaultPosition = 79.5;

    public CustomActions(RobotState state, HardwareMap hardwareMap) {
        hw = HardwareConfig.getHardwareConfig();
        this.state = state;
        drive = new PinpointDrive(hardwareMap, new Pose2d(-hw.ROBOT_WIDTH/2, -70+(hw.ROBOT_LENGTH/2), Math.toRadians(270)));

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
            double error = state.getMotorTarget(motorEnum) - hw.getMotorConfig(motorEnum).motor.getCurrentPosition();
            // Returns false when error is less than or = to 3
            if (error<15){
                return false;
            }
            else{
                return true;
            }
        }
    }

    public class SetServoPositionAction implements Action {
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
            double error = (state.getServoPosition(servoEnum) / 270) - hw.getServoConfig(servoEnum).servo.getPosition();
            // Returns false when the servo position is at the target position
            return false;
        }
    }

    public class GlobalPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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
    public class MoveToNetZoneAndScoreHighBucket implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            Actions.runBlocking(
                    // Retract extendo, transfer and move to scoring pos
                    new SequentialAction(
                            new ParallelAction(
                                    // Default positions
                                    setClawFingerPosition(clawFingersFullyOpenPosition),
                                    setExtendoPitchTarget(extendoPitchTransferTarget),
                                    setExtendoTarget(extendoRetractedTarget),
                                    setExtendoPitchTarget(extendoPitchTransferTarget),
                                    setClawWristPosition(clawWristDefaultPosition),
                                    setBucketSlidesTarget(bucketSlidesDownTarget),
                                    setBucketPosition(bucketTransferPosition)
                            ),
                            // Move to scoring position
                            drive.actionBuilder(drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225)).build(),
                            // Move bucketSlides up to scoring position
                            setBucketSlidesTarget(bucketSlidesHighBasketTarget),
                            // Rotate bucket to score
                            setBucketPosition(bucketScoringPosition),
                            // Wait for sample slide out of bucket
                            new SleepAction(0.5),
                            // Move bucket back to default position
                            setBucketPosition(bucketTransferPosition),
                            // Move bucketSlides back to down position
                            setBucketSlidesTarget(0)
                    )
            );
            return false;
        }
    }
    public class PickUpGroundSample implements Action{
        Vector2d pickUpPose;
        double heading;
        int extendoPosition;

        public PickUpGroundSample(Vector2d pickUpPose, double heading, int extendoPosition){
            this.pickUpPose = pickUpPose;
            this.heading = heading;
            this.extendoPosition = extendoPosition;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    // Default positions
                                    setClawFingerPosition(clawFingersOpenPosition),
                                    setBucketSlidesTarget(bucketSlidesDownTarget),
                                    setExtendoTarget(extendoRetractedTarget),
                                    setExtendoPitchTarget(extendoPitchTransferTarget),
                                    setClawWristPosition(clawWristDefaultPosition),
                                    setClawPitchPosition(clawPitchPitchTransferPosition),
                                    // Move to pickup pose
                                    drive.actionBuilder(drive.pose)
                                            .strafeToLinearHeading(pickUpPose,heading).build(),
                                    // Lower extendoPitch
                                    setExtendoPitchTarget(extendoPitchPickUpTarget)
                            ),
                            // Extend out to custom position
                            setExtendoTarget(extendoPosition),
                            // Lower clawPitch
                            setClawPitchPosition(clawPitchPickUpPosition),
                            // Close claw
                            setClawFingerPosition(clawFingersClosedPosition)
                    )
            );

            return false;
        }
    }
    public class MoveToHighChamberAndScoreSpecimen implements Action{
        Vector2d scoringPose;
        double scoringHeading;
        public MoveToHighChamberAndScoreSpecimen(Vector2d scoringPose, double scoringHeading){
            this.scoringPose = scoringPose;
            this.scoringHeading = scoringHeading;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    // Default positions
                                    setBucketPosition(bucketWhenScoringSpecimenPosition),
                                    setClawFingerPosition(clawFingersClosedPosition),

                                    // Move extendo pitch up, move claw pitch up
                                    setExtendoPitchTarget(extendoPitchSpecimenUpTarget),
                                    setClawPitchPosition(clawPitchSpecimenScoringPosition),
                                    drive.actionBuilder(drive.pose)
                                            // Go to scoring position and heading
                                            .strafeToLinearHeading(scoringPose,scoringHeading).build(),
                                    setExtendoTarget(extendoScoringSpecimenTarget)
                            ),
                            // Lower extendoPitch
                            setExtendoPitchTarget(extendoPitchSpecimenDownTarget),
                            new SleepAction(0.5),
                            setClawFingerPosition(clawFingersOpenPosition)
                    )
            );
            return false;
        }
    }
    public class TransferSample implements Action{
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    // Default positions
                                    setExtendoTarget(extendoRetractedTarget),
                                    setClawFingerPosition(clawFingersClosedPosition),
                                    setClawWristPosition(clawWristDefaultPosition),
                                    setBucketPosition(bucketTransferPosition),
                                    setBucketSlidesTarget(bucketSlidesDownTarget),
                                    // Claw pitch transfer
                                    setClawPitchPosition(clawPitchPitchTransferPosition),
                                    // Raise extendo pitch
                                    setExtendoPitchTarget(extendoPitchTransferTarget)
                            ),
                            setClawFingerPosition(clawFingersFullyOpenPosition)
                    )
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
    public Action setClawWristPosition(double degrees) {return new SetServoPositionAction(ServoEnum.CLAW_WRIST, degrees);}
    public Action setBucketPosition(double degrees) {return new SetServoPositionAction(ServoEnum.BUCKET, degrees);}
    public Action moveToHighChamberAndScoreSpecimen(Vector2d scoringPose, double heading) {return new MoveToHighChamberAndScoreSpecimen(scoringPose, heading);}
    public Action moveToNetZoneAndScoreHighBucket() {return new MoveToNetZoneAndScoreHighBucket();}
    public Action transferSample(){return new TransferSample();}
    public Action pickUpGroundSample(Vector2d pickUpPose, double heading, int extendoPosition){return new PickUpGroundSample(pickUpPose,heading,extendoPosition);}

}
