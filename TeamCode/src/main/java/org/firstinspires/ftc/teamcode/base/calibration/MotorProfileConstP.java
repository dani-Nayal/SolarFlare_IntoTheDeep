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
package org.firstinspires.ftc.teamcode.base.calibration;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;

public class MotorProfileConstP {
    private DcMotorEx   motor;
    private double      minTimeInc;
    private int         timeResolution;
    private double      power;
    private int         Pi;
    private int         Pf;
    private double[]    t;
    private double[]    V;
    private double[]    P;
    private double[]    A;

    public MotorProfileConstP(MotorConfig motorConfig) {
        motor            = motorConfig.motor;
        minTimeInc       = motorConfig.motorCalibConfig.minTimeInc;
        timeResolution   = motorConfig.motorCalibConfig.timeResolution;
        t                = new double[timeResolution];
        V                = new double[timeResolution];
        P                = new double[timeResolution];
        A                = new double[timeResolution];
    }

    protected void gotoPi() {
        DcMotor.RunMode runMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(Pi);
        motor.setPower(1.0);
        while(motor.isBusy())
            continue;
        motor.setMode(runMode);
    }

    public void calcProfile(double power, int Pi, int Pf) {
        this.power        = power;
        this.Pi           = Pi;
        this.Pf           = Pf;

        ElapsedTime timer = new ElapsedTime();
        double      dt    = 0;
        double      tNow  = 0;
        double      tPrev = 0;
        int         tIdx  = 0;
        gotoPi();
        timer.reset();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(power);
        while(tIdx < timeResolution && motor.getCurrentPosition() < Pf && dt < minTimeInc) {
            tNow          = timer.milliseconds();
            dt           += tNow - tPrev;
            if(dt > minTimeInc) {
                t[tIdx]   = tNow;
                P[tIdx]   = motor.getCurrentPosition();
                V[tIdx]   = motor.getVelocity(AngleUnit.DEGREES);
            }
        }
        motor.setPower(0);
    }
}
