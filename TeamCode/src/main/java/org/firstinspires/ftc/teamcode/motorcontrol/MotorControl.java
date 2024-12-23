package org.firstinspires.ftc.teamcode.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.RobotState;

public class MotorControl {
    HardwareConfig hw;
    RobotState state;
    PID extendoPID;
    PID extendoPitchPID;
    PID bucketSlidesPID;
    PID hangPID;
    MotionProfiles profiles;
    public ElapsedTime timer;
    int previousLoopTarget = 0;
    int lastTargetPosition;
    double motorPower;
    public MotorControl(){
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();
        extendoPID = new PID();
        extendoPitchPID = new PID();
        bucketSlidesPID = new PID();
        hangPID = new PID();
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

        if (motorEnum == MotorEnum.EXTENDO){
            motorPower = extendoPID.getPIDOutput(motorEnum, instantTargetPosition);
        }
        else if (motorEnum == MotorEnum.EXTENDO_PITCH){
            motorPower = extendoPitchPID.getPIDOutput(motorEnum, instantTargetPosition);
        }
        else if (motorEnum == MotorEnum.BUCKET_SLIDES){
            motorPower = bucketSlidesPID.getPIDOutput(motorEnum, instantTargetPosition);
        }
        else if (motorEnum == MotorEnum.HANG){
            motorPower = hangPID.getPIDOutput(motorEnum, instantTargetPosition);
        }

        hw.getMotorConfig(motorEnum).motor.setPower(motorPower);

        previousLoopTarget = currentTarget;
    }
}
