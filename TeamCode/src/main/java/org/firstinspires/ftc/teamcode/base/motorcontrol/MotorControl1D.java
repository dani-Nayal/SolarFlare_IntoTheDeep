package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

public class MotorControl1D {
    HardwareConfig             hw;
    RobotState                 state;
    PID                        pid;
    TrapezoidalMotionProfile1D profile;
    public ElapsedTime         timer;
    /**
     * Initial position
     */
    int                        Pi;
    double                     motorPower;
    /**
     * Maximum Acceleration. At this stage it would be an absolute number
     */
    double                     Amax;
    /**
     * Maximum Velocity - at this state it would be an absolute number
     */
    double                     Vmax;

    int                        previousLoopTarget       = 0;
    double                     lastAmax                 = 0;
    double                     lastVmax                 = 0;
    boolean                    isMaxVelocityChanged     = false;
    boolean                    isMaxAccelerationChanged = false;
    /**
     * Initial Velocity, when setting the profile up
     */
    double                     Vi;
    double                     distance;
    MotorEnum                  motorEnum;
    MotorConfig                motorConfig;

    public MotorControl1D(MotorEnum motorEnum_in) {
        motorEnum         = motorEnum_in;
        hw                = HardwareConfig.getInstance();
        motorConfig       = hw.getMotorConfig(motorEnum);
        state             = RobotState.getInstance();
        pid               = new PID();
        profile           = new TrapezoidalMotionProfile1D();
        timer             = new ElapsedTime();
        Amax              = motorConfig.maxAcceleration;
        Vmax              = motorConfig.maxVelocity;
    }

    /**
     * This is meant to be run in a loop
     */
    public void runTrapezoidalMotionProfile(Telemetry telemetry) {
        int currentTarget = state.getMotorTarget(motorEnum);

        // If the target, maxVelocity, or maxAcceleration changes
        if (currentTarget != previousLoopTarget ||
                isMaxVelocityChanged            ||
                isMaxAccelerationChanged ) {
            Pi                       = motorConfig.motor.getCurrentPosition();
            Vi                       = motorConfig.motor.getVelocity();
            distance                 = currentTarget - Pi;

            profile.resetProfile(Amax,Vmax,Vi,distance,Pi);

            isMaxVelocityChanged     = false;
            isMaxAccelerationChanged = false;
            timer.reset();
        }

        double instantTargetPosition = profile.runProfile(timer.seconds());

        motorPower = pid.getPIDOutput(motorEnum, instantTargetPosition);

        motorConfig.motor.setPower(motorPower);

        previousLoopTarget = currentTarget;

        lastVmax                     = Vmax;
        lastAmax                     = Amax;
        telemetry.addData("instant target pos", instantTargetPosition);
        telemetry.addData("distance", distance );
        telemetry.addData("initial speed", Vi);
        telemetry.addData("acceleration time", profile.Ta);
        telemetry.addData("acceleration distance", profile.Sa);
        telemetry.addData("cruise time", profile.Tc);
        telemetry.addData("cruise distance", profile.Sc);
        telemetry.addData("decceleration time", profile.Td);
        telemetry.addData("decceleration distance", profile.Sd);
        telemetry.addData("max accel", Amax);
        telemetry.addData("max velocity", Vmax);
        telemetry.addData("current velocity", motorConfig.motor.getVelocity());

        telemetry.update();
    }

    // Run this method in a loop
    public void runPIDMotorControl(){
        motorPower = pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum));
        motorConfig.motor.setPower(motorPower);
    }
}
