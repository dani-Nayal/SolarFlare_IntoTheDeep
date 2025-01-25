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

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.round;

import java.util.Locale;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import static org.firstinspires.ftc.teamcode.base.calibration.Math.solveQuadraticEquation;
import static org.firstinspires.ftc.teamcode.base.calibration.Math.approxEquals;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.calibration.CalculationException;
import org.firstinspires.ftc.teamcode.base.calibration.ComplexNumberPair;
import org.firstinspires.ftc.teamcode.base.regtest.RegTest;

@SuppressWarnings({"SpellCheckingInspection"})
public class TrapezoidalMotionProfile1D implements MotionProfile {
    /**
     * initialized: Has this profile been initialized?
     */
    private boolean   initialized = false;
    /**
     * FTC Dashboard Telemetry
     */
    private Telemetry telemetryDash;
    /**
     * Lead-in Profile. To be used if:
     *   1- signum(Vi) == -signum(dist) or
     *   2- Vi will overshoot distance even if maximum deceleration was applied immediately
     */
    private TrapezoidalMotionProfile1D leadInProfile = null;
    /**
     * Distance to travel
     */
    private double    dist;
    /**
     * Last distance to travel
     */
    private double    distLast;
    /**
     * Initial Position
     */
    private double    Pi;
    /**
     * Initial Position for the last profile
     */
    private double    PiLast;
    /**
     * Initial Velocity
     */
    private double    Vi;
    /**
     * Maximum Velocity
     */
    private double    Vmax;
    /**
     * Initial brake phase - Tb time to brake speed down to Vmax
     */
    private double    Tb;
    /**
     * Initial brake phase - Sb distance to brake speed down to Vmax
     */
    private double    Sb;
    /**
     * Initial Velocity for the last phase
     */
    private double    ViLast;
    /**
     * Maximum Acceleration
     */
    private double    Amax;
    /**
     * Acceleration Time
     */
    private double    Ta;
    /**
     * Span (distaance) while accelerating
     */
    private double    Sa;
    /**
     * Top Velocity for the triangular case
     */
    private double    Vt;
    /**
     * Cruise Velocity
     */
    private double    Vc;
    /**
     * Cruise Time
     */
    private double    Tc;
    /**
     * Span (distance) cruising
     */
    private double    Sc;
    /**
     * Max Deceleration
     */
    private double    Dmax;
    /**
     * Deceleration Time
     */
    private double    Td;
    /**
     * Span (distance) decelerating
     */
    private double    Sd;
    /**
     * Total Time
     */
    private double    Tt;

    public boolean isInitialized() {
        return initialized;
    }

    protected Telemetry getTelemetryDash() {
        if(telemetryDash == null)
            telemetryDash = FtcDashboard.getInstance().getTelemetry();
        return telemetryDash;
    }

    /**
     * All parameters are positive,
     *  except dist, Vinit can be either negative or positive
     *  We adjust the signs of Vmax, Amax, and Dmax to direct motion towards dist
     *  Consider the possibility of Vi <= Vmax
     * @param Amax_in  Max Acceleration
     * @param Vmax_in  Max Velocity
     * @param Vi_in Initial Velocity
     * @param dist_in  Distance to travel
     * @param Pi_in Initial Position
     */
    public void resetProfile(double dist_in,
                             double Pi_in,
                             double Vi_in,
                             double Vmax_in,
                             double Amax_in,
                             double Dmax_in)
    {
        initialized             = true;
        dist                    = dist_in;
        Pi                      = Pi_in;
        Vi                      = Vi_in;
        Vmax                    = signum(dist)*abs(Vmax_in);
        Amax                    = signum(dist)*abs(Amax_in);
        Dmax                    = -1*signum(dist)*abs(Dmax_in);

        if((Vmax == 0) || (Amax == 0) || (Dmax == 0)) {
            String errorMsg = "TrapezoidalMotionProfile1D.resetProfile()";
            errorMsg += Vmax == 0 ? " - Vmax == 0" : "";
            errorMsg += Amax == 0 ? " - Amax == 0" : "";
            errorMsg += Dmax == 0 ? " - Dmax == 0" : "";
            throw new CalculationException(errorMsg);
        }

        /*
           If signum(Vi) == -signum(dist) the we need a lead-in profile to brake Vi first
         */
        /*
        if(signum(Vi) == -1.0*signum(dist)) {
            // Time to Reverse: Time required to bring the motor to a stop i.e. Vi==0
            double Tr           = abs(Vi/Dmax);
            // Distance to reverse
            double Sr           = signum(Vi)*abs(Vi*Tr -0.5*signum(Vi)*abs(Dmax)*Tr*Tr);
            leadInProfile       = new TrapezoidalMotionProfile1D();
            leadInProfile.resetProfile(
                    Sr,
                    Pi,
                    Vi,
                    signum(Vi)*abs(Vmax),
                    signum(Vi)*abs(Amax),
                    -signum(Vi)*abs(Dmax));
            PiLast               = Pi + leadInProfile.dist;
        } else {
        */
            // System.out.println("signum(Vi)=" + signum(Vi));
            // System.out.println("-1*signum(dist)=" + -1.0*signum(dist));
            PiLast               = Pi;
        // }

        /* Is a brake phase necessary?
           signum(Vi) == -signum(dist) is dealt with prior to this. so the assumption here is
           that signum(Vi) == signum(dist)
         */

        if((signum(Vi) == signum(Vmax)) && (abs(Vi) > abs(Vmax))) {
            // decelerate first to get Vi down to Vmax. then proceed with the normal calculation
            Tb                  = abs((Vi-Vmax_in)/Dmax_in);
            Sb                  = Vi*Tb + 0.5*Dmax_in*Tb*Tb;
            distLast            = dist - Sb;
            ViLast              = Vmax_in;
        } else {
            Tb                  = 0.0;
            Sb                  = 0.0;
            distLast            = dist;
            ViLast              = Vi;
        }

        // Solve for Triangular Motion Profile first
        // Vc is a positive quantity here. But it will be assigned the direction (sign) of
        // distance to travel later
        // Vc is a solution to the following quadratic equation aX^2 + bX + c = 0
        double            a     = Dmax - Amax;
        double            b     = 0;
        double            c     = -Dmax*(ViLast*ViLast + 2*distLast*Amax);
        ComplexNumberPair roots = solveQuadraticEquation(a, b, c);
        if(roots.n1.isComplex() || roots.n2.isComplex()) {
            throw new CalculationException("Could not solve for cruise velocity");
        } else if(signum(roots.n1.a) != signum(roots.n2.a)) {
            Vt                  = signum(roots.n1.a) == signum(distLast) ? roots.n1.a : roots.n2.a;
        } else if(roots.n1.compareTo(roots.n2) >= 0) {
            Vt                  = roots.n1.a;
        } else {
            Vt                  = roots.n2.a;
        }

        Vc                      = abs(Vt)<abs(Vmax) ? Vt : Vmax;

        Ta                      = abs((Vc-ViLast)/Amax);
        Td                      = abs(Vc/Dmax);
        Sa                      = ViLast*Ta + 0.5*Amax*Ta*Ta;
        Sd                      = 0.5*Vc*Td;
        Sc                      = distLast - Sa - Sd;
        Tc                      = Sc/Vc;
        Tt                      = Tb + Ta + Tc + Td;

        /*
        telemetryDash = getTelemetryDash();
        telemetryDash.addData("Pi", Pi);
        telemetryDash.addData("Vi", Vi);
        telemetryDash.addData("distance", dist);
        telemetryDash.addData("Max Velocity", Vmax);
        telemetryDash.addData("Cruise Velocity", Vc);
        telemetryDash.addData("Accel Span", Sa);
        telemetryDash.addData("Cruise Span", Sc);
        telemetryDash.addData("Decel Span", Sd);
        telemetryDash.update();
         */
    }

    // Run this method in a loop
    public int runProfile(double t) {
        double leadInPi    = 0;
        double leadInDist  = 0;
        double leadInTt    = 0;

        if(leadInProfile != null) {
            if (t < leadInProfile.Tt) {
                return leadInProfile.runProfile(t);
            } else {
                leadInPi = leadInProfile.Pi;
                leadInDist = leadInProfile.dist;
                leadInTt = leadInProfile.Tt;
            }
        }

        t                 -= leadInTt;

        if(t < Tb) {
            return (int) round(leadInPi + leadInDist + Pi + Vi * t + 0.5 * Dmax*t*t);
        } else if (t < Tb+Ta) {
            return (int) round(leadInPi + leadInDist + Sb + Pi + Vi * t + 0.5 * Amax*t*t);
        } else if (t < (Tb+Ta+Tc)){
            return (int) round(leadInPi + leadInDist + Pi + Sb + Sa + Vc * (t - Ta));
        } else if (t < Tt){
            double tDec         = t - Ta - Tc;
            return (int) round(leadInPi + leadInDist + Pi + Sb + Sa + Sc + Vc*tDec + 0.5*Dmax*tDec*tDec);
        }  else {
            return (int) round(leadInPi + leadInDist + Pi + dist);
        }
    }

    public boolean approxEqual(TrapezoidalMotionProfile1D other) {
        if(other == null) {
            System.out.println("TranpezoidalMotionProfile1D other is null");
            return false;
        }

        if((leadInProfile == null) ^ (other.leadInProfile == null)) {
            System.out.println("leadInProfile       == null: " + (leadInProfile       == null));
            System.out.println("other.leadInProfile == null: " + (other.leadInProfile == null));
            System.out.println("One leadInProfiles null and the other is not. Returning false equality");
            return false;
        }
        boolean leadInTest   = true;
        // if this is true this would imply that other.leadInProfile != null
        if(leadInProfile != null) {
            System.out.println("Testing equality of leadInProfiles");
            leadInTest = leadInProfile.approxEqual(other.leadInProfile);
        }

        boolean distTest     = approxEquals(dist,     other.dist);
        boolean distLastTest = approxEquals(distLast, other.distLast);
        boolean PiTest       = approxEquals(Pi,       other.Pi);
        boolean PiLastTest   = approxEquals(PiLast,   other.PiLast);
        boolean ViTest       = approxEquals(Vi,       other.Vi);
        boolean VmaxTest     = approxEquals(Vmax,     other.Vmax);
        boolean TbTest       = approxEquals(Tb,       other.Tb);
        boolean SbTest       = approxEquals(Sb,       other.Sb);
        boolean ViLastTest   = approxEquals(ViLast,   other.ViLast);
        boolean AmaxTest     = approxEquals(Amax,     other.Amax);
        boolean TaTest       = approxEquals(Ta,       other.Ta);
        boolean SaTest       = approxEquals(Sa,       other.Sa);
        boolean VtTest       = approxEquals(Vt,       other.Vt);
        boolean VcTest       = approxEquals(Vc,       other.Vc);
        boolean TcTest       = approxEquals(Tc,       other.Tc);
        boolean ScTest       = approxEquals(Sc,       other.Sc);
        boolean DmaxTest     = approxEquals(Dmax,     other.Dmax);
        boolean TdTest       = approxEquals(Td,       other.Td);
        boolean SdTest       = approxEquals(Sd,       other.Sd);
        boolean TtTest       = approxEquals(Tt,       other.Tt);

        if(!leadInTest)
            System.out.println("Failed leadInTest");
        if(!distTest)
            System.out.println("Failed distTest");
        if(!distLastTest)
            System.out.println("Failed distLastTest");
        if(!PiTest)
            System.out.println("Failed PiTest");
        if(!PiLastTest)
            System.out.println("Failed PiLastTest");
        if(!ViTest)
            System.out.println("Failed ViTest");
        if(!VmaxTest)
            System.out.println("Failed VmaxTest");
        if(!TbTest)
            System.out.println("Failed TbTest");
        if(!SbTest)
            System.out.println("Failed SbTest");
        if(!ViLastTest)
            System.out.println("Failed ViLastTest");
        if(!AmaxTest)
            System.out.println("Failed AmaxTest");
        if(!TaTest)
            System.out.println("Failed TaTest");
        if(!SaTest)
            System.out.println("Failed SaTest");
        if(!VtTest)
            System.out.println("Failed VtTest");
        if(!VcTest)
            System.out.println("Failed VcTest");
        if(!TcTest)
            System.out.println("Failed TcTest");
        if(!ScTest)
            System.out.println("Failed ScTest");
        if(!DmaxTest)
            System.out.println("Failed DmaxTest");
        if(!TdTest)
            System.out.println("Failed TdTest");
        if(!SdTest)
            System.out.println("Failed SdTest");
        if(!TtTest)
            System.out.println("Failed TtTest");

        return  leadInTest && distTest && distLastTest && PiTest     && PiLastTest && ViTest   &&
                VmaxTest   && TbTest   && SbTest       && ViLastTest && AmaxTest   && TaTest   &&
                SaTest     && VtTest   && VcTest       && TcTest     && ScTest     && DmaxTest &&
                TdTest     && SdTest   && TtTest;
    }

    @NonNull
    @Override
    public String toString() {
        String formatString = "TrapezoidalMotionProfile1D%n" +
                "    dist     = %1$.5f%n"  +
                "    distLast = %2$.5f%n"  +
                "    Pi       = %3$.5f%n"  +
                "    Vi       = %4$.5f%n"  +
                "    Vmax     = %5$.5f%n"  +
                "    Tb       = %6$.5f%n"  +
                "    Sb       = %7$.5f%n"  +
                "    ViLast   = %8$.5f%n"  +
                "    Amax     = %9$.5f%n"  +
                "    Ta       = %10$.5f%n" +
                "    Sa       = %11$.5f%n" +
                "    Vt       = %12$.5f%n" +
                "    Tc       = %13$.5f%n" +
                "    Vc       = %14$.5f%n" +
                "    Sc       = %15$.5f%n" +
                "    Dmax     = %16$.5f%n" +
                "    Td       = %17$.5f%n" +
                "    Sd       = %18$.5f%n" +
                "    Tt       = %19$.5f%n";

        String str = leadInProfile == null ?
                String.format(Locale.US, "TrapezoidalMotionProfile1D.leadInProfile%n") :
                leadInProfile.toString();
        str       += String.format(Locale.US, formatString,
                dist,distLast,Pi,Vi,Vmax,Tb,Sb,ViLast,Amax,Ta,Sa,Vt,Tc,Vc,Sc,Dmax,Td,Sd,Tt);

        return str;
    }

    /**
     * use to test example motion profiles
     * @param args not used
     */
    public static void main(String[] args) {
        TrapezoidalMotionProfile1D profile = new TrapezoidalMotionProfile1D();
        TrapezoidalMotionProfile1D test    = new TrapezoidalMotionProfile1D();

        String  testDesc;
        boolean testResult;
        //
        // Profile variables, initialized to
        testDesc               = "Profile 1 - Positive Distance - Trinagular Profile - No Cruise";
        test.dist              =  27.0;     // m
        test.distLast          =  27.0;     // m
        test.Pi                =   0;       // m
        test.Vi                =   0.0;     // m/s
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =   0.0;     // m/s
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   3.0;     // a
        test.Sa                =   9.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   6.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   6.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   6.0;     // s (Vc/Td)
        test.Sd                =  18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   9.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 2 - Negative Distance + Triangular Profile - No Cruise";
        test.dist              = -27.0;     // m
        test.distLast          = -27.0;     // m
        test.Pi                =   0;       // m
        test.Vi                =   0.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =   0.0;     // m/s
        test.Vmax              =  -6.0;     // m/s
        test.Amax              =  -2.0;     // m/s^2
        test.Ta                =   3.0;     // a
        test.Sa                =  -9.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =  -6.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =  -6.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =   1.0;     // m/s^2
        test.Td                =   6.0;     // s (Vc/Td)
        test.Sd                = -18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   9.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 3 - Negative Distance + Cruise";
        test.dist              = -39.0;     // m
        test.distLast          = -39.0;     // m
        test.Pi                =   0;       // m
        test.Vi                =   0.0;     // m/s
        test.Vmax              =  -6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =   0.0;     // m/s
        test.Amax              =  -2.0;     // m/s^2
        test.Ta                =   3.0;     // a
        test.Sa                =  -9.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =  -7.21110; // m/s
        test.Tc                =   2.0;     // s
        test.Vc                =  -6.0;     // m/s
        test.Sc                = -12.0;     // m
        test.Dmax              =   1.0;     // m/s^2
        test.Td                =   6.0;     // s (Vc/Td)
        test.Sd                = -18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =  11.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 4 - Vc < Vmax";
        test.dist              =   7.0;     // m
        test.distLast          =   7.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   2.0;     // m/s
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =   2.0;     // m/s
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   1.0;     // s
        test.Sa                =   3.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   4.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   4.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -2.0;     // m/s^2
        test.Td                =   2.0;     // s
        test.Sd                =   4.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   3.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 5 - No Acceleration";
        test.dist              =   4.0;     // m
        test.distLast          =   4.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   4.0;     // m/s
        test.Vmax              =   5.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =   4.0;     // m/s
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   0.0;     // s
        test.Sa                =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   4.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   4.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -2.0;     // m/s^2
        test.Td                =   2.0;     // s
        test.Sd                =   4.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   2.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 6 - Vi = Vmax - No acceleration";
        test.dist              =  24.0;     // m
        test.distLast          =  24.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   4.0;     // m/s
        test.Vmax              =   4.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =   4.0;     // m/s
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   0.0;     // s
        test.Sa                =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   6.11010; // m/s
        test.Tc                =   4.0;     // s
        test.Vc                =   4.0;     // m/s
        test.Sc                =  16.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   4.0;     // s
        test.Sd                =   8.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   8.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 7 - Vi > Vmax";
        test.dist              =  44.0;     // m
        test.distLast          =  30.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   8.0;     // m/s
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   2.0;     // s
        test.Sb                =  14.0;     // m
        test.ViLast            =   6.0;     // m/s
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   0.0;     // s
        test.Sa                =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   7.21110; // m/s
        test.Tc                =   2.0;     // s
        test.Vc                =   6.0;     // m/s
        test.Sc                =  12.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   6.0;     // s
        test.Sd                =  18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =  10.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 8 - signum(Vi) == -signum(dist) + Cruise";

        TrapezoidalMotionProfile1D leadInTest = new TrapezoidalMotionProfile1D();
        // test.leadInProfile     = leadInTest;

        leadInTest.dist        = -32.0;     // m
        leadInTest.distLast    = -32.0;     // m
        leadInTest.Pi          =   0.0;     // m
        leadInTest.Vi          =  -8.0;     // m/s
        leadInTest.Vmax        =  -6.0;     // m/s
        leadInTest.Tb          =   2.0;     // s
        leadInTest.Sb          = -14.0;     // m
        leadInTest.ViLast      =  -6.0;     // m/s
        leadInTest.Amax        =  -2.0;     // m/s^2
        leadInTest.Ta          =   0.0;     // s
        leadInTest.Sa          =   0.0;     // m (0.5*Amax*Ta*Ta)
        leadInTest.Vt          =   7.21110; // m/s
        leadInTest.Tc          =   0.0;     // s
        leadInTest.Vc          =  -6.0;     // m/s
        leadInTest.Sc          =   0.0;     // m
        leadInTest.Dmax        =   1.0;     // m/s^2
        leadInTest.Td          =   6.0;     // s
        leadInTest.Sd          = -18.0;     // -0.5*Dmax*Td*Td
        leadInTest.Tt          =   8.0;     // s

        test.dist              =  23.0;     // m
        test.distLast          =  23.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =  -8.0;     // m/s
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =  -8.0;     // m/s
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   7.0;     // s
        test.Sa                =  -7.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   7.21110; // m/s
        test.Tc                =   2.0;     // s
        test.Vc                =   6.0;     // m/s
        test.Sc                =  12.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   6.0;     // s
        test.Sd                =  18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =  15.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 9 - signum(Vi) == -signum(dist) + No Cruise";

        test.dist              =  18.5;     // m
        test.distLast          =  18.5;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =  -1.0;     // m/s
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.ViLast            =  -1.0;     // m/s
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   3.0;     // s
        test.Sa                =   6.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   5.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   5.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   5.0;     // s
        test.Sd                =  12.5;     // -0.5*Dmax*Td*Td
        test.Tt                =   8.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");
    }
}
