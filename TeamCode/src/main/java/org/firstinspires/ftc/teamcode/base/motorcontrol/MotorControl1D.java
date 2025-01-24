/*
 * Copyright (c) 2025 Murad Nayal
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name Murad Nayal nor the names of contributors to this material may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.RobotLogger;
import org.firstinspires.ftc.teamcode.base.config.RobotMetricsFile;
import org.firstinspires.ftc.teamcode.base.config.RobotState;

import java.util.logging.Logger;

public class MotorControl1D {
    Telemetry                  telemetryDash = FtcDashboard.getInstance().getTelemetry();
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

            profile.resetProfile(distance, Pi, Vi, Vmax, Amax, Amax);

            isMaxVelocityChanged     = false;
            isMaxAccelerationChanged = false;

            timer.reset();
        }

        Double now                   = timer.seconds();
        Integer targetMotorPosition  = profile.runProfile(now);

        targetMotorPower             = pid.getPIDOutput(motorEnum, targetMotorPosition);
        motor.setPower(targetMotorPower);
        /*
        try {
            Thread.currentThread().sleep(100);
        } catch(InterruptedException e) {
            e.printStackTrace();
            return;
        }
        */
        Integer motorPosition        = motor.getCurrentPosition();
        Double  motorPower           = motor.getPower();
        Double  motorVelocity        = motor.getVelocity();

        telemetryDash.addData("iteration",           iter);
        telemetryDash.addData("targetPosition",      targetPosition);
        telemetryDash.addData("targetMotorPosition", targetMotorPosition);
        telemetryDash.addData("motorPosition",       motorPosition);
        telemetryDash.addData("targetMotorPower",    targetMotorPower);
        telemetryDash.addData("motorPower",          motorPower);
        telemetryDash.addData("motorVelocity",       motorVelocity);
        telemetryDash.update();

        metricsFile.addData(
                iter,
                targetMotorPosition,
                motorPosition,
                targetMotorPower,
                motorPower,
                motorVelocity);

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

        /*
        telemetry.addData("motor.getDirection()", motor.getDirection());
        telemetry.addData("motor.isBusy()",       motor.isBusy());
        telemetry.addData("motor.ZPB",            motor.getZeroPowerBehavior());
        telemetry.addData("motor.getPowerFloat()", motor.getPowerFloat());
        telemetry.addData("motor.getMode()", motor.getMode());
        telemetry.addData("motor.getCurrent()", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor.isMotorEnabled", motor.isMotorEnabled());
        telemetry.addData("motor.getConnectionInfo()", motor.getConnectionInfo());
        telemetry.addData("motor.getPortNumber()", motor.getPortNumber());
        telemetry.addData("motor.isOverCurrent()", motor.isOverCurrent());
        telemetry.addData("motor.getCurrentAlert()", motor.getCurrentAlert(CurrentUnit.AMPS));
        telemetry.addData("motor.getDirection()", motor.getDirection());
        */

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
