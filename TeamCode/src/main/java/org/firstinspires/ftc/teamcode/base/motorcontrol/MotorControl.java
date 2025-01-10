package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.MotorEnum;
import org.firstinspires.ftc.teamcode.base.RobotState;

public class MotorControl {
    HardwareConfig hw;
    RobotState state;
    PID pid;
    TrapezoidalMotionProfile profile;
    public ElapsedTime timer;
    int currentPosition;
    double motorPower;
    double maxAcceleration;
    double maxVelocity;
    int previousLoopTarget = 0;
    double lastMaxAcceleration = 0;
    double lastMaxVelocity = 0;
    boolean maxVelocityChanged = false;
    boolean maxAccelerationChanged = false;
    MotorEnum motorEnum;
    public MotorControl(MotorEnum motorEnum){
        this.motorEnum = motorEnum;
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
        pid = new PID();
        profile = new TrapezoidalMotionProfile();
        timer = new ElapsedTime();
        maxAcceleration = hw.getMotorConfig(motorEnum).maxAcceleration;
        maxVelocity = hw.getMotorConfig(motorEnum).maxVelocity;
    }
    // Run this method in a loop
    public void runTrapezoidalMotionProfile(){

        int currentTarget = state.getMotorTarget(motorEnum);

        // If the target, maxVelocity, or maxAcceleration changes
        if (currentTarget != previousLoopTarget || maxVelocityChanged || maxAccelerationChanged){

            currentPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();
            double distance = currentTarget - currentPosition;

            timer.reset();

            profile.resetProfile(
                    maxAcceleration,
                    maxVelocity,
                    hw.getMotorConfig(motorEnum).motor.getVelocity(),
                    distance);
            maxVelocityChanged = false;
            maxAccelerationChanged = false;
        }

        double instantTargetPosition = profile.runProfile(timer.seconds());

        motorPower = pid.getPIDOutput(motorEnum, instantTargetPosition);

        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);

        previousLoopTarget = currentTarget;
        lastMaxVelocity = maxVelocity;
        lastMaxAcceleration = maxAcceleration;
    }
    public void setMaxAcceleration(double maxAcceleration){
        this.maxAcceleration = maxAcceleration;
        maxAccelerationChanged = true;
    }
    public void setMaxVelocity(double maxVelocity){
        this.maxVelocity = maxVelocity;
        maxVelocityChanged = true;
    }

    // Run this method in a loop
    public void runPIDMotorControl(MotorEnum motorEnum){
        motorPower = pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum));
        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);
    }
}
