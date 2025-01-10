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
    int previousLoopTarget = 0;
    int currentPosition;
    double motorPower;
    public MotorControl(){
        hw = HardwareConfig.getInstance();
        state = RobotState.getInstance();
        pid = new PID();
        profile = new TrapezoidalMotionProfile();
        timer = new ElapsedTime();
    }
    // Run this method in a loop
    public void runTrapezoidalMotionProfile(MotorEnum motorEnum){
        int currentTarget = state.getMotorTarget(motorEnum);

        // If the target changes
        if (currentTarget != previousLoopTarget){
            currentPosition = hw.getMotorConfig(motorEnum).motor.getCurrentPosition();
            double distance = currentTarget - currentPosition;
            timer.reset();

            profile.resetProfile(
                    hw.getMotorConfig(motorEnum).maxAcceleration,
                    hw.getMotorConfig(motorEnum).maxVelocity,
                    hw.getMotorConfig(motorEnum).motor.getVelocity(),
                    distance);
        }

        double instantTargetPosition = profile.runProfile(timer.seconds());

        motorPower = pid.getPIDOutput(motorEnum, instantTargetPosition);

        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);

        previousLoopTarget = currentTarget;
    }

    // Run this method in a loop
    public void runPIDMotorControl(MotorEnum motorEnum){
        motorPower = pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum));
        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);
    }
}
