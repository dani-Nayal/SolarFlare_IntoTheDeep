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
        double            b     = Dmax*Vi;
        double            c     = -2*Dmax*(Vi*Vi + dist*Amax);
        ComplexNumberPair roots = solveQuadraticEquation(a, b, c);
        if(roots.n1.isComplex() || roots.n2.isComplex()) {
            throw new CalculationException("Could not solve for cruise velocity");
        } else if(roots.n1.compareTo(roots.n2)>=0) {
            Vc                  = roots.n1.a;
        } else {
            Vc                  = roots.n2.a;
        }
        Vc                      = abs(Vc)<abs(Vmax) ? Vc : Vmax;

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
        return approxEquals( dist, other.dist) &&
                approxEquals(Pi,   other.Pi)   &&
                approxEquals(Vi,   other.Vi)   &&
                approxEquals(Vmax, other.Vmax) &&
                approxEquals(Amax, other.Amax) &&
                approxEquals(Ta,   other.Ta)   &&
                approxEquals(Sa,   other.Sa)   &&
                approxEquals(Vc,   other.Vc)   &&
                approxEquals(Tc,   other.Tc)   &&
                approxEquals(Sc,   other.Sc)   &&
                approxEquals(Dmax, other.Dmax) &&
                approxEquals(Td,   other.Td)   &&
                approxEquals(Sd,   other.Sd)   &&
                approxEquals(Tt,   other.Tt);
    }

    @NonNull
    @Override
    public String toString() {
        String formatString = "TrapezoidalMotionProfile1D%n" +
                "    dist = %1$20.5f%n"  +
                "    Pi   = %2$20.5f%n"  +
                "    Vi   = %3$20.5f%n"  +
                "    Vmax = %4$20.5f%n"  +
                "    Amax = %5$20.5f%n"  +
                "    Ta   = %6$20.5f%n"  +
                "    Sa   = %7$20.5f%n"  +
                "    Tc   = %8$20.5f%n"  +
                "    Vc   = %9$20.5f%n"  +
                "    Sc   = %10$20.5f%n" +
                "    Dmax = %11$20.5f%n" +
                "    Td   = %12$20.5f%n" +
                "    Sd   = %13$20.5f%n" +
                "    Tt   = %14$20.5f%n";

        return String.format(Locale.US, formatString,
                dist,Pi,Vi,Vmax,Amax,Ta,Sa,Tc,Vc,Sc,Dmax,Td,Sd,Tt);
    }

    /**
     * use to test example motion profiles
     * @param args not used
     */
    public static void main(String[] args) {
        TrapezoidalMotionProfile1D profile = new TrapezoidalMotionProfile1D();
        TrapezoidalMotionProfile1D test    = new TrapezoidalMotionProfile1D();

        //
        // Profile variables, initialized to
        // profile 1 - Positive Distance - Trinagular Profile - No Cruise
        test.Pi          =  0;    // m
        test.Vi          =  0.0;  // m/s
        test.Amax        =  2.0;  // m/s^2
        test.Ta          =  3.0;  // a
        test.Sa          =  9.0;  // m (0.5*Amax*Ta*Ta)
        test.Vmax        =  6.0;  // m/s
        test.Vc          =  6.0;  // m/s
        test.Sc          =  0.0;  // m
        test.Dmax        = -1.0;  // m/s^2
        test.Td          =  6.0;  // s (Vc/Td)
        test.Sd          =  18.0; // -0.5*Dmax*Td*Td
        test.dist        =  27.0; // m
        test.Tt          =  9.0;  // s

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println("Profile 1 matched: " + profile.approxEqual(test));
        System.out.println(profile + "\n");

        // profile 2 - Negative Distance + Triangular Profile - No Cruise
        test.dist        = -27.0; // m
        test.Vmax        = -6.0;  // m/s
        test.Amax        = -2.0;  // m/s^2
        test.Sa          = -9.0;  // m (0.5*Amax*Ta*Ta)
        test.Vc          = -6.0;  // m/s
        test.Dmax        =  1.0;  // m/s^2
        test.Sd          = -18.0; // -0.5*Dmax*Td*Td

        profile.resetProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        System.out.println("Profile 2 matched: " + profile.approxEqual(test));
        System.out.println(profile + "\n");
    }
}
