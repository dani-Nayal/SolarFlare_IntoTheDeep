package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotLogger;
import org.firstinspires.ftc.teamcode.base.config.RobotMetricsFile;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

import java.util.logging.Logger;

public class MotorControl1D {
    Logger                     logger;
    RobotMetricsFile           metricsFile;
    HardwareConfig             hw;
    RobotState                 state;
    PID                        pid;
    TrapezoidalMotionProfile1D profile;
    Integer                    iter;
    public ElapsedTime         timer;
    /**
     * Initial position
     */
    int                        Pi;
    Double                     targetMotorPower;
    /**
     * Maximum Acceleration. At this stage it would be an absolute number
     */
    double                     Amax;
    /**
     * Maximum Deceleration.
     */
    double                     Dmax;
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
    DcMotorEx                  motor;

    public MotorControl1D(MotorEnum motorEnum_in) {
        hw                = HardwareConfig.getInstance();
        logger            = RobotLogger.getInstance().getConfigLogger();
        metricsFile       = RobotLogger.getInstance().getMetricsFile("motion_profile");
        motorEnum         = motorEnum_in;
        motorConfig       = hw.getMotorConfig(motorEnum);
        motor             = motorConfig.motor;
        iter              = 0;

        state             = RobotState.getInstance();
        pid               = new PID();
        profile           = new TrapezoidalMotionProfile1D();
        timer             = new ElapsedTime();
        Amax              = motorConfig.maxAcceleration;
        Dmax              = motorConfig.maxAcceleration;
        Vmax              = motorConfig.maxVelocity;
    }

    /**
     * This is meant to be run in a loop
     */
    public void runTrapezoidalMotionProfile(Telemetry telemetry) {
        int targetPosition = state.getMotorTarget(motorEnum);

        // If the target, maxVelocity, or maxAcceleration changes
        if (targetPosition != previousLoopTarget ||
                isMaxVelocityChanged            ||
                isMaxAccelerationChanged ) {
            Pi                       = motor.getCurrentPosition();
            Vi                       = motor.getVelocity();
            distance                 = targetPosition - Pi;

            profile.resetProfile(Amax, Vmax, Vi, distance, Pi);

            isMaxVelocityChanged     = false;
            isMaxAccelerationChanged = false;

            timer.reset();
        }

        Double now                   = timer.seconds();
        Integer targetMotorPosition  = profile.runProfile(now);

        targetMotorPower             = pid.getPIDOutput(motorEnum, targetMotorPosition);
        motor.setPower(targetMotorPower);
        Integer motorPosition        = motor.getCurrentPosition();
        Double  motorPower           = motor.getPower();
        Double  motorVelocity        = motor.getVelocity();
        Double  testVariable         = 12345.9876543;

        metricsFile.addData(
                iter,
                targetMotorPosition,
                motorPosition,
                targetMotorPower,
                motorPower,
                motorVelocity,
                testVariable);

        previousLoopTarget           = targetPosition;
        lastVmax                     = Vmax;
        lastAmax                     = Amax;

        telemetry.addData("iteration",           iter++);
        telemetry.addData("now",                 now);
        telemetry.addData("distance",            profile.dist );
        telemetry.addData("initial velocity",    profile.Vi);
        telemetry.addData("cruise velocity",     profile.Vc);
        telemetry.addData("accel time",          profile.Ta);
        telemetry.addData("accel distance",      profile.Sa);
        telemetry.addData("cruise time",         profile.Tc);
        telemetry.addData("cruise distance",     profile.Sc);
        telemetry.addData("decl time",           profile.Td);
        telemetry.addData("decl distance",       profile.Sd);
        telemetry.addData("total time",          profile.Tt);
        telemetry.addData("max accel",           profile.Amax);
        telemetry.addData("max decl",            profile.Dmax);
        telemetry.addData("max velocity",        profile.Vmax);
        telemetry.addData("target pos",          targetPosition);
        telemetry.addData("targetMotorPosition", targetMotorPosition);
        telemetry.addData("motorPosition",       motorPosition);
        telemetry.addData("targetMotorPower",    targetMotorPower);
        telemetry.addData("motorPower",          motorPower);
        telemetry.addData("motorVelocity",       motorVelocity);

        telemetry.update();
    }

    // Run this method in a loop
    public void runPIDMotorControl() {
        targetMotorPower = pid.getPIDOutput(motorEnum, state.getMotorTarget(motorEnum));
        motor.setPower(targetMotorPower);
    }
}
