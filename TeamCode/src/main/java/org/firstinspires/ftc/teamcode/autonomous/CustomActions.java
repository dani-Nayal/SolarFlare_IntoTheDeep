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
import org.firstinspires.ftc.teamcode.PresetMechanismPositions;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.ServoEnum;

// Contains all non-default roadrunner actions that are used in our autonomous routines
public class CustomActions {
    RobotState state;
    HardwareConfig hw;
    PinpointDrive drive;
    HardwareMap hardwareMap;
    PresetMechanismPositions presets;
    public int SERVO_SPEED = 400;

    public CustomActions(RobotState state, HardwareMap hardwareMap) {
        hw = HardwareConfig.getHardwareConfig();
        this.state = state;
        this.hardwareMap = hardwareMap;
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
            double error = Math.abs(state.getMotorTarget(motorEnum) - hw.getMotorConfig(motorEnum).motor.getCurrentPosition());
            // Returns false when error is less 15
            return !(error < 15);
        }
    }
    // Initial position based off preload and cycle type
    public void setInitialDrivePosition(String preloadType, String cycleType){
        // Right side of robot is on the middle line of field, Outtake side is touching field perimeter
        if (preloadType.equals("specimen") && cycleType.equals("sample")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(-hw.ROBOT_WIDTH/2, -70+(hw.ROBOT_LENGTH/2), Math.toRadians(270)));
        }
        // Right side of robot is touching field perimeter, Outtake side is on the left side of seam that is on the edge of field tile
        // Barely outside of net zone
        else if (preloadType.equals("sample") && cycleType.equals("sample")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(-32.125-(hw.ROBOT_LENGTH/2), -70+(hw.ROBOT_WIDTH/2), Math.toRadians(180)));
        }
        // Right side of robot is touching field perimeter, Outtake side is on the left side of seam that is on the edge of field tile
        // Barely outside of net zone
        else if (preloadType.equals("sample") && cycleType.equals("specimen")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(-32.125-(hw.ROBOT_LENGTH/2), -70+(hw.ROBOT_WIDTH/2), Math.toRadians(180)));
        }
        else if (preloadType.equals("specimen") && cycleType.equals("specimen")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(hw.ROBOT_WIDTH/2, -70+(hw.ROBOT_LENGTH/2), Math.toRadians(270)));
        }
        // Left side of robot is on the middle line of field, Outtake side is touching field perimeter
        else{
            throw new IllegalArgumentException("setInitialDrivePosition parameter incorrect");
        }
    }
    public Pose2d getInitialDrivePosition(String preloadType, String cycleType){
        if (preloadType.equals("specimen") && cycleType.equals("sample")) {
            return new Pose2d(-hw.ROBOT_WIDTH/2, -70+(hw.ROBOT_LENGTH/2), Math.toRadians(270));
        }
        // Right side of robot is touching field perimeter, Outtake side is on the left side of seam that is on the edge of field tile
        // Barely outside of net zone
        else if (preloadType.equals("sample") && cycleType.equals("sample")) {
            return new Pose2d(-32.125-(hw.ROBOT_LENGTH/2), -70+(hw.ROBOT_WIDTH/2), Math.toRadians(180));
        }
        // Right side of robot is touching field perimeter, Outtake side is on the left side of seam that is on the edge of field tile
        // Barely outside of net zone
        else if (preloadType.equals("sample") && cycleType.equals("specimen")) {
            return new Pose2d(-32.125-(hw.ROBOT_LENGTH/2), -70+(hw.ROBOT_WIDTH/2), Math.toRadians(180));
        }
        else if (preloadType.equals("specimen") && cycleType.equals("specimen")) {
            return new Pose2d(hw.ROBOT_WIDTH/2, -70+(hw.ROBOT_LENGTH/2), Math.toRadians(270));
        }
        // Left side of robot is on the middle line of field, Outtake side is touching field perimeter
        else{
            throw new IllegalArgumentException("getInitialDrivePosition parameter incorrect");
        }
    }
    public PinpointDrive getDrive(){
        if (drive != null){
            return drive;
        }
        throw new NullPointerException("drive variable is null");
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
            telemetry.addData("pinpoint heading", hw.pinpointConfig.pinpoint.getYawScalar());
            telemetry.addData("Control hub IMU heading", hw.imuConfig.imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("robot pose", String.valueOf(drive.pose));
            telemetry.update();
            return true;
        }
    }
    public Action updateTelemetry(Telemetry telemetry) {return new UpdateTelemetry(telemetry);}
    public Action globalPID() {return new GlobalPID();}
    public Action setExtendoTarget(int target) {return new SetMotorTargetAction(MotorEnum.EXTENDO, target);}
    public Action setExtendoPitchTarget(int target) {return new SetMotorTargetAction(MotorEnum.EXTENDO_PITCH, target);}
    public Action setBucketSlidesTarget(int target) {return new SetMotorTargetAction(MotorEnum.BUCKET_SLIDES, target);}

    public Action getServoSleepAction(ServoEnum servoEnum, double position) {
        // Calculates amount of time needed for the servo to reach the target position
        double sleepTime = Math.abs((position - state.getServoPosition(servoEnum))) / SERVO_SPEED;
        return new SleepAction(sleepTime + 0.1);
    }

    public Action setClawPitchPosition(double degrees) {
        Action sleepPLAction = getServoSleepAction(ServoEnum.CLAW_PITCH_LEFT, degrees);
        Action sleepPRAction = getServoSleepAction(ServoEnum.CLAW_PITCH_RIGHT, degrees);
        return new ParallelAction(
                new SequentialAction(
                        new SetServoPositionAction(ServoEnum.CLAW_PITCH_LEFT, degrees),
                        sleepPLAction),
                new SequentialAction(
                        new SetServoPositionAction(ServoEnum.CLAW_PITCH_RIGHT, degrees),
                        sleepPRAction)
            );
    }

    public Action setClawFingerPosition(double degrees) {
        Action sleepAction = getServoSleepAction(ServoEnum.CLAW_FINGERS, degrees);
        return new SequentialAction(
                new SetServoPositionAction(ServoEnum.CLAW_FINGERS, degrees),
                sleepAction);
    }

    public Action setClawWristPosition(double degrees) {
        Action sleepAction = getServoSleepAction(ServoEnum.CLAW_WRIST, degrees);
        return new SequentialAction(
                new SetServoPositionAction(ServoEnum.CLAW_WRIST, degrees),
                sleepAction);
    }

    public Action setBucketPosition(double degrees) {
        Action sleepAction = getServoSleepAction(ServoEnum.BUCKET, degrees);
        return new SequentialAction(
                new SetServoPositionAction(ServoEnum.BUCKET, degrees),
                sleepAction);
    }

    public SequentialAction moveToHighChamberAndScoreSpecimen(Pose2d initialDrivePose, Vector2d scoringPose, double heading) {
        return new SequentialAction(
                new ParallelAction(
                        // Default positions
                        setBucketPosition(presets.bucketWhenScoringSpecimenPosition),
                        setClawFingerPosition(presets.clawFingersClosedPosition),

                        // Move extendo pitch up, move claw pitch up, move extendo up
                        setExtendoPitchTarget(presets.extendoPitchSpecimenUpTarget),
                        setClawPitchPosition(presets.clawPitchSpecimenScoringPosition),
                        setExtendoTarget(presets.extendoScoringSpecimenTarget),

                        // Go to scoring position and heading
                        drive.actionBuilder(initialDrivePose)
                                .strafeToLinearHeading(scoringPose,heading).build()
                ),
                // Lower extendoPitch
                setExtendoPitchTarget(presets.extendoPitchSpecimenDownTarget),
                setClawFingerPosition(presets.clawFingersOpenPosition)
        );
    }
    public SequentialAction moveToNetZone(Pose2d initialDrivePose, Vector2d scoringPosition, double scoringHeading) {
        return new SequentialAction(
                // Move to scoring position
                drive.actionBuilder(initialDrivePose)
                        .strafeToLinearHeading(scoringPosition, scoringHeading).build()
        );
    }
    public SequentialAction scoreHighBucket(){
        return new SequentialAction(
                // Move bucketSlides up to scoring position
                setBucketSlidesTarget(presets.bucketSlidesHighBasketTarget),
                // Rotate bucket to score
                setBucketPosition(presets.bucketScoringPosition),
                // Wait for sample slide out of bucket
                new SleepAction(0.5),
                // Move bucket back to default position
                setBucketPosition(presets.bucketTransferPosition),
                // Move bucketSlides back to down position
                setBucketSlidesTarget(presets.bucketSlidesDownTarget)
        );
    }
    public SequentialAction transferSample(){
        return new SequentialAction(
                new ParallelAction(
                        // Retract extendo
                        setExtendoTarget(presets.extendoRetractedTarget),
                        // Claw pitch transfer
                        setClawPitchPosition(presets.clawPitchTransferPosition),
                        // Raise extendo pitch
                        setExtendoPitchTarget(presets.extendoPitchTransferTarget)
                ),
                setClawFingerPosition(presets.clawFingersFullyOpenPosition)
        );
    }
    public Action grabGroundSample(Pose2d initialDrivePose, Vector2d pickUpPose, double heading, int extendoPosition) {
        return new SequentialAction(
                new ParallelAction(
                        // Default positions
                        setClawFingerPosition(presets.clawFingersOpenPosition),
                        setBucketSlidesTarget(presets.bucketSlidesDownTarget),
                        setExtendoTarget(presets.extendoRetractedTarget),
                        setExtendoPitchTarget(presets.extendoPitchTransferTarget),
                        setClawWristPosition(presets.clawWristDefaultPosition),
                        setClawPitchPosition(presets.clawPitchTransferPosition),
                        setBucketPosition(presets.bucketTransferPosition),
                        // Move to pickup pose
                        drive.actionBuilder(initialDrivePose)
                                .strafeToLinearHeading(pickUpPose,heading).build(),
                        // Lower extendoPitch
                        setExtendoPitchTarget(presets.extendoPitchPickUpTarget)
                ),
                // Extend out to custom position
                setExtendoTarget(extendoPosition),
                // Lower clawPitch
                setClawPitchPosition(presets.clawPitchPickUpPosition),
                // Close claw
                setClawFingerPosition(presets.clawFingersClosedPosition)
        );
    }

}
