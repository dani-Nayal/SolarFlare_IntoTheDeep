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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.base.calibration.CalculationException;
import org.firstinspires.ftc.teamcode.base.calibration.ComplexNumberPair;

@SuppressWarnings({"SpellCheckingInspection"})
public class TrapezoidalMotionProfile1D implements MotionProfile {
    /**
     * FTC Dashboard Telemetry
     */
    Telemetry telemetryDash;
    /**
     * Distance to travel
     */
    double    dist;
    /**
     * Initial Position
     */
    double    Pi;
    /**
     * Initial Velocity
     */
    double    Vi;
    /**
     * Maximum Velocity
     */
    double    Vmax;
    /**
     * Maximum Acceleration
     */
    double    Amax;
    /**
     * Acceleration Time
     */
    double    Ta;
    /**
     * Span (distaance) while accelerating
     */
    double    Sa;
    /**
     * Top Velocity for the triangular case
     */
    double    Vt;
    /**
     * Cruise Velocity
     */
    double    Vc;
    /**
     * Cruise Time
     */
    double    Tc;
    /**
     * Span (distance) cruising
     */
    double    Sc;
    /**
     * Max Deceleration
     */
    double    Dmax;
    /**
     * Deceleration Time
     */
    double    Td;
    /**
     * Span (distance) decelerating
     */
    double    Sd;
    /**
     * Total Time
     */
    double    Tt;

    Telemetry getTelemetryDash() {
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
        dist                    = dist_in;
        Pi                      = Pi_in;
        Vi                      = Vi_in;
        Vmax                    = signum(dist)*abs(Vmax_in);
        Amax                    = signum(dist)*abs(Amax_in);
        Dmax                    = -1*signum(dist)*abs(Dmax_in);

        // Solve for Triangular Motion Profile first
        // Vc is a positive quantity here. But it will be assigned the direction (sign) of
        // distance to travel later

        // Vc is a solution to the following quadratic equation aX^2 + bX + c = 0
        double            a     = Dmax - Amax;
        double            b     = 0;
        double            c     = -Dmax*(Vi*Vi + 2*dist*Amax);
        ComplexNumberPair roots = solveQuadraticEquation(a, b, c);
        if(roots.n1.isComplex() || roots.n2.isComplex()) {
            throw new CalculationException("Could not solve for cruise velocity");
        } else if(signum(roots.n1.a) != signum(roots.n2.a)) {
            Vt                  = signum(roots.n1.a) == signum(dist) ? roots.n1.a : roots.n2.a;
        } else if(roots.n1.compareTo(roots.n2) >= 0) {
            Vt                  = roots.n1.a;
        } else {
            Vt                  = roots.n2.a;
        }

        System.out.println("roots: " + roots);

        Vc                      = abs(Vt)<abs(Vmax) ? Vt : Vmax;

        Ta                      = abs((Vc-Vi) / Amax);
        Td                      = abs(Vc/Dmax);
        Sa                      = Vi*Ta + 0.5*Amax*Ta*Ta;
        Sd                      = 0.5*Vc*Td;
        Sc                      = dist - Sa - Sd;
        Tc                      = Sc/Vc;
        Tt                      = Ta + Tc + Td;

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
        if (t < Ta) {
            return (int) round(Pi + Vi * t + 0.5 * Amax*t*t);
        } else if (t < (Ta + Tc)){
            return (int) round(Pi + Sa + Vc * (t - Ta));
        } else if (t < Tt){
            double tDec         = t - Ta - Tc;
            return (int) round(Pi + Sa + Sc + Vc*tDec + 0.5*Dmax*tDec*tDec);
        }
        else{
            return (int) round(Pi + dist);
        }
    }

    public boolean approxEqual(TrapezoidalMotionProfile1D other) {
        boolean distTest = approxEquals(dist, other.dist);
        boolean PiTest   = approxEquals(Pi,   other.Pi);
        boolean ViTest   = approxEquals(Vi,   other.Vi);
        boolean VmaxTest = approxEquals(Vmax, other.Vmax);
        boolean AmaxTest = approxEquals(Amax, other.Amax);
        boolean TaTest   = approxEquals(Ta,   other.Ta);
        boolean SaTest   = approxEquals(Sa,   other.Sa);
        boolean VtTest   = approxEquals(Vt,   other.Vt);
        boolean VcTest   = approxEquals(Vc,   other.Vc);
        boolean TcTest   = approxEquals(Tc,   other.Tc);
        boolean ScTest   = approxEquals(Sc,   other.Sc);
        boolean DmaxTest = approxEquals(Dmax, other.Dmax);
        boolean TdTest   = approxEquals(Td,   other.Td);
        boolean SdTest   = approxEquals(Sd,   other.Sd);
        boolean TtTest   = approxEquals(Tt,   other.Tt);

        if(!distTest)
            System.out.println("Failed distTest");
        if(!PiTest)
            System.out.println("Failed PiTest");
        if(!ViTest)
            System.out.println("Failed ViTest");
        if(!VmaxTest)
            System.out.println("Failed VmaxTest");
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

        return  distTest && PiTest && ViTest && VmaxTest && AmaxTest && TaTest && SaTest &&
                VtTest   && VcTest && TcTest && ScTest   && DmaxTest && TdTest && SdTest && TtTest;
    }

    @NonNull
    @Override
    public String toString() {
        String formatString = "TrapezoidalMotionProfile1D%n" +
                "    dist = %1$.5f%n"  +
                "    Pi   = %2$.5f%n"  +
                "    Vi   = %3$.5f%n"  +
                "    Vmax = %4$.5f%n"  +
                "    Amax = %5$.5f%n"  +
                "    Ta   = %6$.5f%n"  +
                "    Sa   = %7$.5f%n"  +
                "    Vt   = %8$.5f%n"  +
                "    Tc   = %9$.5f%n"  +
                "    Vc   = %10$.5f%n"  +
                "    Sc   = %11$.5f%n" +
                "    Dmax = %12$.5f%n" +
                "    Td   = %13$.5f%n" +
                "    Sd   = %14$.5f%n" +
                "    Tt   = %15$.5f%n";

        return String.format(Locale.US, formatString,
                dist,Pi,Vi,Vmax,Amax,Ta,Sa,Vt,Tc,Vc,Sc,Dmax,Td,Sd,Tt);
    }

    /**
     * use to test example motion profiles
     * @param args not used
     */
    public static void main(String[] args) {
        TrapezoidalMotionProfile1D profile = new TrapezoidalMotionProfile1D();
        TrapezoidalMotionProfile1D test    = new TrapezoidalMotionProfile1D();

        String testDesc  = "";
        //
        // Profile variables, initialized to
        testDesc         = "Profile 1 - Positive Distance - Trinagular Profile - No Cruise";
        test.dist        =  27.0; // m
        test.Pi          =   0;    // m
        test.Vi          =   0.0;  // m/s
        test.Vmax        =   6.0;  // m/s
        test.Amax        =   2.0;  // m/s^2
        test.Ta          =   3.0;  // a
        test.Sa          =   9.0;  // m (0.5*Amax*Ta*Ta)
        test.Vt          =   6.0;  // m/s
        test.Tc          =   0.0;  // s
        test.Vc          =   6.0;  // m/s
        test.Sc          =   0.0;  // m
        test.Dmax        =  -1.0;  // m/s^2
        test.Td          =   6.0;  // s (Vc/Td)
        test.Sd          =  18.0; // -0.5*Dmax*Td*Td
        test.Tt          =   9.0;  // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println(testDesc + " - Passed: " + profile.approxEqual(test));
        // System.out.println(profile + "\n");

        testDesc         = "Pofile 2 - Negative Distance + Triangular Profile - No Cruise";
        test.dist        = -27.0; // m
        test.Pi          =   0;    // m
        test.Vi          =   0.0;  // m/s
        test.Vmax        =  -6.0;  // m/s
        test.Amax        =  -2.0;  // m/s^2
        test.Ta          =   3.0;  // a
        test.Sa          =  -9.0;  // m (0.5*Amax*Ta*Ta)
        test.Vt          =  -6.0;  // m/s
        test.Tc          =   0.0;  // s
        test.Vc          =  -6.0;  // m/s
        test.Sc          =   0.0;  // m
        test.Dmax        =   1.0;  // m/s^2
        test.Td          =   6.0;  // s (Vc/Td)
        test.Sd          = -18.0; // -0.5*Dmax*Td*Td
        test.Tt          =   9.0;  // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println(testDesc + " - Passed: " + profile.approxEqual(test));
        // System.out.println(profile + "\n");

        testDesc         = "Profile 3 - Negative Distance + Cruise";
        test.dist        = -39.0;     // m
        test.Pi          =   0;       // m
        test.Vi          =   0.0;     // m/s
        test.Vmax        =  -6.0;     // m/s
        test.Amax        =  -2.0;     // m/s^2
        test.Ta          =   3.0;     // a
        test.Sa          =  -9.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt          =  -7.21110; // m/s
        test.Tc          =   2.0;  // s
        test.Vc          =  -6.0;  // m/s
        test.Sc          = -12.0;  // m
        test.Dmax        =   1.0;  // m/s^2
        test.Td          =   6.0;  // s (Vc/Td)
        test.Sd          = -18.0; // -0.5*Dmax*Td*Td
        test.Tt          =  11.0;  // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println(testDesc + " - Passed: " + profile.approxEqual(test));
        // System.out.println(profile + "\n");

        testDesc         = "Profile 4 - Vc < Vmax";
        test.dist        =   7.0;  // m
        test.Pi          =   0.0;  // m
        test.Vi          =   2.0;  // m/s
        test.Vmax        =   6.0;  // m/s
        test.Amax        =   2.0;  // m/s^2
        test.Ta          =   1.0;  // s
        test.Sa          =   3.0;  // m (0.5*Amax*Ta*Ta)
        test.Vt          =   4.0;  // m/s
        test.Tc          =   0.0;  // s
        test.Vc          =   4.0;  // m/s
        test.Sc          =   0.0;  // m
        test.Dmax        =  -2.0;  // m/s^2
        test.Td          =   2.0;  // s
        test.Sd          =   4.0;  // -0.5*Dmax*Td*Td
        test.Tt          =   3.0;  // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println(testDesc + " - Passed: " + profile.approxEqual(test));
        // System.out.println(profile + "\n");

        testDesc         = "Profile 5 - No Acceleration";
        test.dist        =   4.0;  // m
        test.Pi          =   0.0;  // m
        test.Vi          =   4.0;  // m/s
        test.Vmax        =   5.0;  // m/s
        test.Amax        =   2.0;  // m/s^2
        test.Ta          =   0.0;  // s
        test.Sa          =   0.0;  // m (0.5*Amax*Ta*Ta)
        test.Vt          =   4.0;  // m/s
        test.Tc          =   0.0;  // s
        test.Vc          =   4.0;  // m/s
        test.Sc          =   0.0;  // m
        test.Dmax        =  -2.0;  // m/s^2
        test.Td          =   2.0;  // s
        test.Sd          =   4.0;  // -0.5*Dmax*Td*Td
        test.Tt          =   2.0;  // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println(testDesc + " - Passed: " + profile.approxEqual(test));
        // System.out.println(profile + "\n");

        testDesc         = "Profile 6 - Vi = Vmax - No acceleration";
        test.dist        =  24.0;     // m
        test.Pi          =   0.0;     // m
        test.Vi          =   4.0;     // m/s
        test.Vmax        =   4.0;     // m/s
        test.Amax        =   2.0;     // m/s^2
        test.Ta          =   0.0;     // s
        test.Sa          =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt          =   6.11010; // m/s
        test.Tc          =   4.0;     // s
        test.Vc          =   4.0;     // m/s
        test.Sc          =  16.0;     // m
        test.Dmax        =  -1.0;     // m/s^2
        test.Td          =   4.0;     // s
        test.Sd          =   8.0;     // -0.5*Dmax*Td*Td
        test.Tt          =   8.0;     // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println(testDesc + " - Passed: " + profile.approxEqual(test));
        // System.out.println(profile + "\n");
    }
}
