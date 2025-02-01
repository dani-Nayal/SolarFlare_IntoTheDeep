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

import static java.lang.Math.abs;
import static java.lang.Math.max;

import java.util.Arrays;
import java.util.Locale;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.config.JSONWritable;
import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.MotorEnum;
import org.firstinspires.ftc.teamcode.base.config.Validatable;
import org.firstinspires.ftc.teamcode.base.logging.MetricsWritable;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetrics;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetricsFile;
import org.firstinspires.ftc.teamcode.base.utils.JSONUtils;
import org.firstinspires.ftc.teamcode.base.validate.Validation;

public class MotorProfileConstP implements JSONWritable, MetricsWritable, Validatable {
    private       MotorEnum   motorEnum;
    private       MotorConfig motorConfig;
    private       DcMotorEx   motor;
    /**
     * Calibration Direction: FORWARD, REVERSE
     */
    public        Direction   calibDirection;

    public        double      minTimeInc;
    /**
     * Encoder resolution of the motor itself at the shaft output (PPR)
     */
    public        double      encoderResolution;
    public        int         timeResolution;
    public        double      power;
    /**
     * Starting Position
     */
    public        int         Pi;
    /**
     * Final Position
     */
    public        int         Pf;
    /**
     * Index where data stops. i.e. if we reach the Pf before we fill out
     * the entire array (before timeResolution)
     */
    public        int         tIdxMax;
    /**
     * The number of periods used to compute Aavg and Vavg
     */
    public        int         averagingPeriods;
    /**
     * Time coordinate
     */
    private       double[]    t;
    /**
     * Time to extract position
     */
    private       double[]    tPextract;
    /**
     * Time to extract velocity
     */
    private       double[]    tVextract;
    /**
     * Time to extract current
     */
    private       double[]    tCextract;
    /**
     * Cycle time
     */
    private       double[]    tCycle;
    /**
     * Position coordinate
     */
    private       double[]    P;
    /**
     * Velocity coordinate
     */
    private       double[]    V;
    /**
     * Moving Average Velocity
     */
    private       double[]    Vavg;
    /**
     * Acceleration coordinate
     */
    private       double[]    A;
    /**
     * Moving Average Acceleration
     */
    private       double[]    Aavg;
    /**
     *
     */
    private       double[]    C;
    /**
     * Maximum velocity. should be close the stread state velocity
     */
    public        double      Vmax;
    /**
     * Maximum Acceleration
     */
    public        double      Amax;
    /**
     * Maximum Deceleration
     */
    public        double      Dmax;
    /**
     * Has the profile reached the target position Pf
     */
    public        boolean     isTargetReached;
    /**
     * Index of steady state for Vavg
     */
    public        Integer     ssIdxVavg;
    /**
     * steady state Vavg. null if it does not obtain
     */
    public        Double      ssVavg;
    /**
     * Index of steady state of Aavg
     */
    public        Integer     ssIdxAavg;
    /**
     * Steady state Aavg. null if it does not obtain
     */
    public        Double      ssAavg;
    /**
     * Constructor requires information about the motor
     * @param motorConfig_in: The configuration of the motor being calibrated
     */
    public MotorProfileConstP(MotorConfig motorConfig_in, Direction calibDirection_in) {
        motorConfig       = motorConfig_in;
        calibDirection    = calibDirection_in;
        motor             = motorConfig.motor;
        motorEnum         = motorConfig.motorEnum;
        encoderResolution = motorConfig.getEncoderResolution();
        minTimeInc        = motorConfig.calibParams.minTimeInc;
        timeResolution    = motorConfig.calibParams.timeResolution;
        t                 = new double[timeResolution];
        tPextract         = new double[timeResolution];
        tVextract         = new double[timeResolution];
        tCextract         = new double[timeResolution];
        tCycle            = new double[timeResolution];
        V                 = new double[timeResolution];
        Vavg              = new double[timeResolution];
        P                 = new double[timeResolution];
        A                 = new double[timeResolution];
        Aavg              = new double[timeResolution];
        C                 = new double[timeResolution];
    }

    protected void gotoStart() {
        RunMode runMode   = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(Pi);
        motor.setPower(1.0);

        while(motor.isBusy())
            continue;
        motor.setMode(runMode);
    }

    public double getPLast() {
        return P[tIdxMax];
    }

    private void calcAveragingPeriods() {
        double tPeriod   = (t[tIdxMax] - t[0])/ (tIdxMax + 1.0);
        averagingPeriods = (int) (motorConfig.calibParams.averagingTime / tPeriod);
    }

    private void calcDerivedData() {
        calcAveragingPeriods();
        Vmax                  = 0.0;
        Amax                  = Double.NEGATIVE_INFINITY;
        Dmax                  = Double.POSITIVE_INFINITY;
        /// tIdx references the original arrays
        for(int tIdx=1; tIdx<=tIdxMax; tIdx++) {
            int tIdx0         = max(tIdx-averagingPeriods, 0);
            double tNow       = t[tIdx];
            double VNow       = V[tIdx];
            double ANow       = (VNow-V[tIdx-1])/(tNow-t[tIdx-1]);
            Vavg[tIdx]        = (P[tIdx]-P[tIdx0])/(tNow-t[tIdx0]);
            Aavg[tIdx]        = (VNow-V[tIdx0])/(tNow-t[tIdx0]);
            A[tIdx]           = ANow;

            if(abs(VNow) > abs(Vmax))
                Vmax          = VNow;

            if(ANow > Amax)
                Amax          = ANow;
            if(ANow < Dmax)
                Dmax          = ANow;
        }

        ssIdxVavg             = Math.getSteadyStateStartPredicate(
                V,
                averagingPeriods,
                (Double v1, Double v2) -> abs(v1-v2) < abs(Vmax)/250.0);
        ssVavg                = ssIdxVavg!=null ? Vavg[ssIdxVavg] : null;

        ssIdxAavg             = Math.getSteadyStateStartPredicate(
                A,
                averagingPeriods,
                (Double a1, Double a2) -> abs(a1-a2) < abs(Amax)/250.0);
        ssAavg                = ssIdxAavg!=null ? Aavg[ssIdxAavg] : null;

        isTargetReached       = Math.approxEquals(P[tIdxMax],Pf,5.0/Pf);
    }

    public void calcProfile(double power_in, int Pi_in, int Pf_in) {
        power                   = power_in;
        Pi                      = calibDirection == Direction.FORWARD? Pi_in : Pf_in;
        Pf                      = calibDirection == Direction.FORWARD? Pf_in : Pi_in;

        ElapsedTime timer       = new ElapsedTime();
        ElapsedTime eTimer      = new ElapsedTime();
        ElapsedTime eTimer2     = new ElapsedTime();
        double  dt;
        double  tPrev           = 0;
        int     tIdx            = 0;

        gotoStart();
        RunMode runMode         = motor.getMode();
        motor.setDirection(calibDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double  tCycleNow       = 0;
        double  tPextractNow    = 0;
        double  tVextractNow    = 0;
        double  tCextractNow    = 0;
        double  tNow;
        double  PNow;
        double  VNow;
        double  CNow;
        boolean shortOfTarget;
        timer.reset();
        eTimer2.reset();
        motor.setPower(power);
        do {
            tCycleNow          += eTimer2.seconds();
            eTimer2.reset();
            tNow                = timer.seconds();

            /// Pull current position info
            eTimer.reset();
            PNow                = motor.getCurrentPosition();
            tPextractNow       += eTimer.seconds();

            /// Pull velocity info
            /// With no arguments getVelocity() returns Ticks Per Second
            eTimer.reset();
            VNow                = motor.getVelocity();
            tVextractNow       += eTimer.seconds();

            /// Pull current info
            eTimer.reset();
            CNow                = motor.getCurrent(CurrentUnit.AMPS);
            tCextractNow       += eTimer.seconds();

            dt                  = tNow - tPrev;
            if(dt >= minTimeInc) {
                t        [tIdx] = tNow;
                P        [tIdx] = PNow;
                V        [tIdx] = VNow;
                C        [tIdx] = CNow;
                tCycle   [tIdx] = tCycleNow;
                tPextract[tIdx] = tPextractNow;
                tVextract[tIdx] = tVextractNow;
                tCextract[tIdx] = tCextractNow;
                tPrev           = tNow;
                tCycleNow       = 0;
                tPextractNow    = 0;
                tVextractNow    = 0;
                tCextractNow    = 0;

                tIdx++;
            }
            shortOfTarget       = calibDirection == Direction.FORWARD? PNow<Pf : PNow>Pf;
        } while(tIdx<timeResolution && shortOfTarget);

        motor.setPower(0);
        motor.setMode(runMode);
        /// tIdx is one position ahead of the last valid slot in the data arrays
        tIdxMax                 = tIdx - 1;

        calcDerivedData();
        trimArrays();
    }

    private double[] trimArray(double[] data) {
        double[] trimmedArray = new double[tIdxMax+1];
        System.arraycopy(data, 0, trimmedArray, 0, tIdxMax+1);
        return trimmedArray;
    }

    private void trimArrays() {
        t                 = trimArray(t);
        tPextract         = trimArray(tPextract);
        tVextract         = trimArray(tVextract);
        tCextract         = trimArray(tCextract);
        tCycle            = trimArray(tCycle);
        V                 = trimArray(V);
        Vavg              = trimArray(Vavg);
        P                 = trimArray(P);
        A                 = trimArray(A);
        Aavg              = trimArray(Aavg);
        C                 = trimArray(C);
    }

    public boolean hasSteadyStateV() {
        return ssVavg != null;
    }

    public boolean hasSteadyStateA() {
        return ssAavg != null;
    }

    public boolean hasReachedTarget() {
        return isTargetReached;
    }

    /**
     * Returns time to reach steady state in seconds
     * @return time to reach steady state
     */
    public Double getTimeToSteadyState() {
        return hasSteadyStateV() ? t[ssIdxVavg] : null;
    }

    /**
     * Returns time to reach target
     * @return time to reach target
     */
    public Double getTimeToTarget() {
        return hasReachedTarget()? t[tIdxMax] : null;
    }

    public Double getSteadyStateV() {
        return ssVavg;
    }

    public Double getSteadyStateA() {
        return ssAavg;
    }

    public MotorProfileConstP getCopyForJSON() {
        MotorProfileConstP profile = new MotorProfileConstP(motorConfig, calibDirection);
        profile.motorEnum          = this.motorEnum;
        profile.motorConfig        = null;
        profile.motor              = null;
        profile.minTimeInc         = this.minTimeInc;
        profile.encoderResolution  = this.encoderResolution;
        profile.timeResolution     = this.timeResolution;
        profile.power              = this.power;
        profile.Pi                 = this.Pi;
        profile.Pf                 = this.Pf;
        /// This is the last -valid- index into the data arrays
        profile.tIdxMax            = this.tIdxMax;
        profile.t                  = this.t;
        profile.tPextract          = this.tPextract;
        profile.tVextract          = this.tVextract;
        profile.tCextract          = this.tCextract;
        profile.tCycle             = this.tCycle;
        profile.V                  = this.V;
        profile.Vavg               = this.Vavg;
        profile.P                  = this.P;
        profile.A                  = this.A;
        profile.Aavg               = this.Aavg;
        profile.C                  = this.C;

        return profile;
    }

    public String getJSONFileId() {
        return String.format(Locale.US, "%1$s-%2$s-%3$.2f", motorEnum, calibDirection.name(), power);
    }

    public void writeJSON() {
        MotorProfileConstP trimmedThis = getCopyForJSON();
        JSONUtils.writeJSON(trimmedThis);
    }

    public String getMetricsFileId() {
        return String.format(Locale.US, "%1$s-%2$s-%3$.4f", motorEnum, calibDirection.name(), power);
    }

    public String getMetricsTableType() {
        return "MotorProfileConstP";
    }

    public void writeMetrics() {
        RobotMetricsFile metricsFile = RobotMetrics.getInstance().getMetricsFile(this);
        for(int tIdx=0; tIdx<=tIdxMax; tIdx++) {
            metricsFile.addData(
                    t[tIdx], tPextract[tIdx], tVextract[tIdx], tCextract[tIdx], tCycle[tIdx],
                    P[tIdx], V[tIdx],         Vavg[tIdx],      A[tIdx],         Aavg[tIdx],
                    C[tIdx]
            );
        }

        metricsFile.close();
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();
        sb.append("MotorProfileConstP\n");
        sb.append("  JSONFileId=")       .append(getJSONFileId())           .append("\n");
        sb.append("  motorEnum=")        .append(motorEnum)                 .append("\n");
        sb.append("  minTimeInc=")       .append(minTimeInc)                .append("\n");
        sb.append("  encoderResolution=").append(encoderResolution)         .append("\n");
        sb.append("  timeResolution=")   .append(timeResolution)            .append("\n");
        sb.append("  power=")            .append(power)                     .append("\n");
        sb.append("  Pi=")               .append(Pi)                        .append("\n");
        sb.append("  Pf=")               .append(Pf)                        .append("\n");
        sb.append("  tIdxMax=")          .append(tIdxMax)                   .append("\n");
        sb.append("  t=\n")              .append(Arrays.toString(t))        .append("\n");
        sb.append("  tPextract=\n")      .append(Arrays.toString(tPextract)).append("\n");
        sb.append("  tVextract=\n")      .append(Arrays.toString(tVextract)).append("\n");
        sb.append("  tCextract=\n")      .append(Arrays.toString(tCextract)).append("\n");
        sb.append("  tCycle=\n")         .append(Arrays.toString(tCycle))   .append("\n");
        sb.append("  V=\n")              .append(Arrays.toString(V))        .append("\n");
        sb.append("  Vavg=\n")           .append(Arrays.toString(Vavg))     .append("\n");
        sb.append("  P=\n")              .append(Arrays.toString(P))        .append("\n");
        sb.append("  A=\n")              .append(Arrays.toString(A))        .append("\n");
        sb.append("  Aavg=\n")           .append(Arrays.toString(Aavg))     .append("\n");
        sb.append("  C=\n")              .append(Arrays.toString(C))        .append("\n");

        return sb.toString();
    }

    public boolean isValid() {
        return Validation.validate("motorEnum", motorEnum)                                                        &&
                Validation.validate("motorConfig", motorConfig)                                                   &&
                Validation.validate("motor",motor)                                                                &&
                Validation.validate("minTimeInc", minTimeInc, (Double x) -> x>0.0)                                &&
                Validation.validate("encoderResolution", encoderResolution, (Double x) -> x>0)                    &&
                Validation.validate("timeResolution",timeResolution,(Integer i) -> i>0)                           &&
                Validation.validate("power", power, (Double x) -> x>=-1 && x<=1)                                  &&
                Validation.validate("tIdxMax", tIdxMax, (Integer i) -> i>=0 && i<timeResolution)                  &&
                Validation.validate("averagingPeriods", averagingPeriods, (Integer i) -> i>0 && i<timeResolution) &&
                Validation.validate("t", t)                                                                       &&
                Validation.validate("tPextract", tPextract)                                                       &&
                Validation.validate("tVextract", tVextract)                                                       &&
                Validation.validate("tCextract", tCextract)                                                       &&
                Validation.validate("tCycle", tCycle)                                                             &&
                Validation.validate("P", P)                                                                       &&
                Validation.validate("V", V)                                                                       &&
                Validation.validate("Vavg", Vavg)                                                                 &&
                Validation.validate("A", A)                                                                       &&
                Validation.validate("Aavg", Aavg)                                                                 &&
                Validation.validate("C", C)                                                                       &&
                Validation.validate("Vmax", Vmax)                                                                 &&
                Validation.validate("Amax", Amax)                                                                 &&
                Validation.validate("Dmax", Dmax)                                                                 &&
                Validation.validate("ssIdxVavg", ssIdxVavg, (Integer i) -> i>=0 && i<timeResolution)              &&
                Validation.validate("ssVavg", ssVavg)                                                             &&
                Validation.validate("ssIdxAavg", ssIdxAavg, (Integer i) -> i>=0 && i<timeResolution)              &&
                Validation.validate("ssAavg", ssAavg);
    }

    public static void main(String[] args) {
    }
}
