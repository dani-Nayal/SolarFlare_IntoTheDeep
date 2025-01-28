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

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.config.JSONWritable;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.utils.JSONUtils;

import java.util.Arrays;
import java.util.Locale;

public class MotorProfileConstP implements JSONWritable {
    private final MotorEnum   motorEnum;
    private final DcMotorEx   motor;
    private final double      minTimeInc;
    /**
     * Encoder resolution of the motor itself at the shaft output (PPR)
     */
    private final double      encoderResolution;
    private final int         timeResolution;
    private       double      power;
    private       int         Pi;
    private       int         Pf;
    /**
     * Index where data stops. i.e. if we reach the Pf before we fill out
     * the entire array (before timeResolution)
     */
    private       int         tIdxLast;
    /**
     * Time coordinate
     */
    private final double[]    t;
    /**
     * Velocity coordinate
     */
    private final double[]    V;
    /**
     * Position coordinate
     */
    private final double[]    P;
    /**
     * Acceleration coordinate
     */
    private final double[]    A;
    /**
     *
     */
    private final double[]    C;
    /**
     * Constructor requires information about the motor
     * @param motorConfig: The configuration of the motor being calibrated
     */
    public MotorProfileConstP(MotorConfig motorConfig) {
        motor             = motorConfig.motor;
        motorEnum         = motorConfig.motorEnum;
        minTimeInc        = motorConfig.motorCalibConfig.minTimeInc;
        encoderResolution = motorConfig.encoderResolution;
        timeResolution    = motorConfig.motorCalibConfig.timeResolution;
        t                 = new double[timeResolution];
        V                 = new double[timeResolution];
        P                 = new double[timeResolution];
        A                 = new double[timeResolution];
        C                 = new double[timeResolution];
    }

    protected void gotoPi() {
        RunMode runMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(Pi);
        motor.setPower(1.0);
        while(motor.isBusy())
            continue;
        motor.setMode(runMode);
    }

    private void calcAcceleration() {
        for(int tIdx=1; tIdx<=tIdxLast; tIdx++) {
            A[tIdx]       = (V[tIdx]-V[tIdx-1])/(t[tIdx]-t[tIdx-1]);
        }
    }

    public void calcProfile(double power_in, int Pi_in, int Pf_in) {
        power             = power_in;
        Pi                = Pi_in;
        Pf                = Pf_in;

        ElapsedTime timer = new ElapsedTime();
        double  dt;
        double  tPrev     = 0;
        int     tIdx      = 0;

        gotoPi();
        RunMode runMode   = motor.getMode();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        motor.setPower(power);
        while(tIdx<timeResolution && motor.getCurrentPosition()<Pf) {
            double tNow   = timer.milliseconds();
            double PNow   = motor.getCurrentPosition();
            double CNow   = motor.getCurrent(CurrentUnit.AMPS);
            /// With no arguments getVelocity() returns Ticks Per Second
            double VNow   = motor.getVelocity();
            dt            = tNow - tPrev;
            if(dt >= minTimeInc) {
                t[tIdx]   = tNow;
                P[tIdx]   = PNow;
                V[tIdx]   = VNow;
                C[tIdx++] = CNow;
                tPrev     = tNow;
            }
            calcAcceleration();
        }
        motor.setPower(0);
        motor.setMode(runMode);
        tIdxLast          = tIdx;
    }

    public String getJSONFileId() {
        return String.format(Locale.US, "%1$s-%2$.4f", motorEnum, power);
    }

    public void writeJSON() {
        JSONUtils.writeJSON(this);
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();
        sb.append("MotorProfileConstP\n");
        sb.append("  JSONFileId=")       .append(getJSONFileId())   .append("\n");
        sb.append("  motorEnum=")        .append(motorEnum)         .append("\n");
        sb.append("  minTimeInc=")       .append(minTimeInc)        .append("\n");
        sb.append("  encoderResolution=").append(encoderResolution) .append("\n");
        sb.append("  timeResolution=")   .append(timeResolution)    .append("\n");
        sb.append("  power=")            .append(power)             .append("\n");
        sb.append("  Pi=")               .append(Pi)                .append("\n");
        sb.append("  Pf=")               .append(Pf)                .append("\n");
        sb.append("  tIdxLast=")         .append(tIdxLast)          .append("\n");
        sb.append("  t=\n")              .append(Arrays.toString(t)).append("\n");
        sb.append("  V=\n")              .append(Arrays.toString(V)).append("\n");
        sb.append("  P=\n")              .append(Arrays.toString(P)).append("\n");
        sb.append("  A=\n")              .append(Arrays.toString(A)).append("\n");
        sb.append("  C=\n")              .append(Arrays.toString(C)).append("\n");

        return sb.toString();
    }

    public static void main(String[] args) {
    }
}
