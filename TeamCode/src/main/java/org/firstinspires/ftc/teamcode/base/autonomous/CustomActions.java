package org.firstinspires.ftc.teamcode.base.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;
import org.firstinspires.ftc.teamcode.base.config.ServoEnum;
import org.firstinspires.ftc.teamcode.base.motorcontrol.MotorControl;

public class CustomActions {
    RobotState state;
    HardwareConfig hw;
    PinpointDrive drive;
    HardwareMap hardwareMap;
    MotorControl extendoControl;
    MotorControl extendoPitchControl;
    MotorControl bucketSlidesControl;
    MotorControl hangControl;
    Telemetry telemetry;
    public CustomActions(Telemetry telemetry) {
        this.telemetry = telemetry;
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
    }
    public class SetMotorTargetAction implements Action {
        MotorEnum motorEnum;
        int target;
        public SetMotorTargetAction (MotorEnum motorEnum, int target) {
            this.motorEnum = motorEnum;
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            state.setMotorTarget(motorEnum, target);
            double error = Math.abs(state.getMotorTarget(motorEnum) - hw.getMotorConfig(motorEnum).motor.getCurrentPosition());
            return !(error == 0);
        }
    }
    public class SetServoPositionAction implements Action {
        private final ServoEnum servoEnum;
        private final double position;
        private final ElapsedTime sleepTimer;
        private final double sleepTime;
        private boolean initialized = false;

        public SetServoPositionAction(ServoEnum servoEnum, double position) {
            this.servoEnum = servoEnum;
            this.position = position;
            this.sleepTimer = new ElapsedTime();
            // Pre-calculate sleep time during initialization
            this.sleepTime = Math.abs((position - state.getServoPosition(servoEnum))) /
                    hw.getServoConfig(servoEnum).degreesPerSecond + 0.1; // TODO: Optimize additional sleeping time
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                // Set the servo position and start the timer only once
                state.setServoPosition(servoEnum, position);
                sleepTimer.reset();
                initialized = true;
            }

            // Check if the elapsed time has surpassed the sleep time
            return sleepTimer.seconds() <= sleepTime;
        }
    }
    // Initial position based off preload and cycle type
    public void setInitialDrivePosition(String preloadType, String cycleType){
        // Right side of robot is on the middle line of field, Outtake side is touching field perimeter
        if (preloadType.equals("specimen") && cycleType.equals("sample")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(
                    -hw.robotDimensions.width/2,
                    -70+(hw.robotDimensions.length/2), Math.toRadians(270)));
        }
        // Right side of robot is touching field perimeter, Outtake side is on the left side of seam that is on the edge of field tile
        // Barely outside of net zone
        else if (preloadType.equals("sample") && cycleType.equals("sample")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(
                    -32.125-(hw.robotDimensions.length/2),
                    -70+(hw.robotDimensions.width/2), Math.toRadians(180)));
        }
        // Right side of robot is touching field perimeter, Outtake side is on the left side of seam that is on the edge of field tile
        // Barely outside of net zone
        else if (preloadType.equals("sample") && cycleType.equals("specimen")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(
                    -32.125-(hw.robotDimensions.length/2),
                    -70+(hw.robotDimensions.width/2), Math.toRadians(180)));
        }
        else if (preloadType.equals("specimen") && cycleType.equals("specimen")) {
            drive = new PinpointDrive(hardwareMap, new Pose2d(
                    hw.robotDimensions.width/2,
                    -70+(hw.robotDimensions.length/2), Math.toRadians(270)));
        }
        // Left side of robot is on the middle line of field, Outtake side is touching field perimeter
        else{
            throw new IllegalArgumentException("setInitialDrivePosition parameter incorrect");
        }
    }
    public Pose2d getInitialDrivePosition(String preloadType, String cycleType){
        if (preloadType.equals("specimen") && cycleType.equals("sample")) {
            return new Pose2d(-hw.robotDimensions.width/2,
                    -70+(hw.robotDimensions.length/2), Math.toRadians(270));
        }

        else if (preloadType.equals("sample") && cycleType.equals("sample")) {
            return new Pose2d(
                    -32.125-(hw.robotDimensions.length/2),
                    -70+(hw.robotDimensions.width/2), Math.toRadians(180));
        }

        else if (preloadType.equals("sample") && cycleType.equals("specimen")) {
            return new Pose2d(
                    -32.125-(hw.robotDimensions.length/2),
                    -70+(hw.robotDimensions.width/2), Math.toRadians(180));
        }
        else if (preloadType.equals("specimen") && cycleType.equals("specimen")) {
            return new Pose2d(
                    hw.robotDimensions.width/2,
                    -70+(hw.robotDimensions.length/2), Math.toRadians(270));
        }

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
    // TODO: Create custom actions
    public class GlobalMechanismControl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendoControl.runTrapezoidalMotionProfile(telemetry);
            bucketSlidesControl.runTrapezoidalMotionProfile(telemetry);
            extendoPitchControl.runTrapezoidalMotionProfile(telemetry);
            hangControl.runPIDMotorControl();
            return true;
        }
    }
    public Action globalMechanismControl() {return new GlobalMechanismControl();}
    public SequentialAction moveToHighChamberAndScoreSpecimen(Pose2d initialDrivePose, Vector2d scoringPose, double heading) {
        return new SequentialAction(
            drive.actionBuilder(initialDrivePose)
                    .strafeToLinearHeading(scoringPose, heading).build()
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

        );
    }
    public SequentialAction transferSample(){
        return new SequentialAction(

        );
    }
    public Action grabGroundSample(Pose2d initialDrivePose, Vector2d pickUpPose, double heading, int extendoPosition) {
        return new SequentialAction(

        );
    }

}


