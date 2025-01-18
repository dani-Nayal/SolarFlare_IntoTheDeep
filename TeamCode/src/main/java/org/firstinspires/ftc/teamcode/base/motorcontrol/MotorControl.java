package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

public class MotorControl {
    HardwareConfig hw;
    RobotState state;
    PID pid;
    MotionProfiles badProfile;
    OldTrapezoidalMotionProfile oldProfile;
    TrapezoidalMotionProfile profile;
    public ElapsedTime timer;
    int currentPosition;
    double motorPower;
    double maxAcceleration;
    double maxVelocity;
    int previousLoopTarget = 0;
    double lastMaxAcceleration = 0;
    double lastMaxVelocity = 0;
    boolean isMaxVelocityChanged = false;
    boolean isMaxAccelerationChanged = false;
    double initialVelocity;
    double distance;
    MotorEnum motorEnum;
    public MotorControl(MotorEnum motorEnum){
        this.motorEnum = motorEnum;
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
        pid = new PID();
        oldProfile = new OldTrapezoidalMotionProfile();
        badProfile = new MotionProfiles();
        profile = new TrapezoidalMotionProfile();
        timer = new ElapsedTime();
        maxAcceleration = hw.getMotorConfig(motorEnum).maxAcceleration;
        maxVelocity = hw.getMotorConfig(motorEnum).maxVelocity;
    }
    // Run this method in a loop
    public void runTrapezoidalMotionProfile(Telemetry telemetry){

        int currentTarget = state.getMotorTarget(motorEnum);

        // If the target, maxVelocity, or maxAcceleration changes
        if (currentTarget != previousLoopTarget || isMaxVelocityChanged || isMaxAccelerationChanged){

            currentPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();
            distance = currentTarget - currentPosition;
            initialVelocity = hw.getMotorConfig(motorEnum).motor.getVelocity();

            timer.reset();

            profile.resetProfile(
                    maxAcceleration,
                    maxVelocity,
                    initialVelocity,
                    distance,
                    currentPosition);

            isMaxVelocityChanged = false;
            isMaxAccelerationChanged = false;
        }

        double instantTargetPosition = profile.runProfile(timer.seconds());

        motorPower = pid.getPIDOutput(motorEnum, instantTargetPosition);

        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);

        previousLoopTarget = currentTarget;
        lastMaxVelocity = maxVelocity;
        lastMaxAcceleration = maxAcceleration;
        telemetry.addData("motor power", motorPower);
        telemetry.addData("state target position", state.getMotorTarget(motorEnum));
        telemetry.addData("current position", hw.getMotorConfig(motorEnum).motor.getCurrentPosition());
        telemetry.addData("instant target pos", instantTargetPosition);
        telemetry.addData("distance", distance );
        telemetry.addData("initial speed", initialVelocity);
        telemetry.addData("acceleration time", profile.accelerationTime);
        telemetry.addData("acceleration distance", profile.accelerationDistance);
        telemetry.addData("cruise time", profile.cruiseTime);
        telemetry.addData("cruise distance", profile.cruiseDistance);
        telemetry.addData("deceleration time", profile.decelerationTime);
        telemetry.addData("deceleration distance", profile.decelerationDistance);
        telemetry.addData("total time", profile.totalTime);
        telemetry.addData("max acceleration", profile.maxAcceleration);
        telemetry.addData("max deceleration", profile.maxDeceleration);
        telemetry.addData("max velocity", profile.maxVelocity);
        telemetry.addData("current velocity", hw.getMotorConfig(motorEnum).motor.getVelocity());

        telemetry.update();
    }
    public void runOldTrapezoidalMotionProfile(){
        int currentTarget = state.getMotorTarget(motorEnum);

        int currentPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();

        double distance = currentTarget - currentPosition;

        if (previousLoopTarget != currentTarget){
            timer.reset();
        }

        double instantTargetPosition = badProfile.runTrapezoidalMotionProfile(
                hw.getMotorConfig(motorEnum).maxVelocity,
                hw.getMotorConfig(motorEnum).maxAcceleration,
                distance,
                timer.seconds()
        );

        double motorPower = pid.getPIDOutput(motorEnum, instantTargetPosition);

        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);
        previousLoopTarget = currentTarget;
    }
    public void setMaxAcceleration(double maxAcceleration){
        this.maxAcceleration = maxAcceleration;
        isMaxAccelerationChanged = true;
    }
    public void setMaxVelocity(double maxVelocity){
        this.maxVelocity = maxVelocity;
        isMaxVelocityChanged = true;
    }

    // Run this method in a loop
    public void runPIDMotorControl(){
        motorPower = pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum));
        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);
    }
}
