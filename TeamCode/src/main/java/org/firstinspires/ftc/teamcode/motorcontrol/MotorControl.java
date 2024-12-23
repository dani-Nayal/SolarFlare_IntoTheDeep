package org.firstinspires.ftc.teamcode.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.RobotState;

public class MotorControl {
    HardwareConfig hw;
    RobotState state;
    PID pid;
    MotionProfiles profiles;
    public ElapsedTime timer;
    int previousLoopTarget = 0;
    int lastTargetPosition;
    public MotorControl(){
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        pid = new PID();
        profiles = new MotionProfiles();
        timer = new ElapsedTime();
    }
    // Meant to be ran in a loop
    public void runTrapezoidalMotorControl(MotorEnum motorEnum){
        int currentTarget = state.getMotorTarget(motorEnum);

        if (currentTarget != previousLoopTarget){
            lastTargetPosition = previousLoopTarget;
            timer.reset();
        }

        double instantTargetPosition = profiles.runTrapezoidalMotionProfile(
                hw.getMotorConfig(motorEnum).maxVelocity,
                hw.getMotorConfig(motorEnum).maxAcceleration,
                currentTarget - lastTargetPosition,
                timer.seconds());

        double motorPower = pid.getPIDOutput(motorEnum, instantTargetPosition);

        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);

        previousLoopTarget = currentTarget;
    }
}
