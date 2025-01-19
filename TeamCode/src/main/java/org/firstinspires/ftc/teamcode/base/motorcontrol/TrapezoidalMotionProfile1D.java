package org.firstinspires.ftc.teamcode.base.motorcontrol;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.round;

import java.util.Locale;
import java.util.function.Function;

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
     * Max Acceleration
     */
    double    Amax;
    /**
     * Max Deceleration
     */
    double    Dmax;
    /**
     * Max Velocity
     */
    double    Vmax;
    /**
     * Initial Velocity
     */
    double    Vi;
    /**
     * Initial Position
     */
    double    Pi;
    /**
     * Cruise Speed
     */
    double    Vc;
    /**
     * Distance to travel
     */
    double    dist;
    /**
     * Acceleration Time
     */
    double    Ta;
    /**
     * Deceleration Time
     */
    double    Td;
    /**
     * Cruise Time
     */
    double    Tc;
    /**
     * Total Time
     */
    double    Tt;
    /**
     * Span (distaance) while accelerating
     */
    double    Sa;
    /**
     * Span (distance) cruising
     */
    double    Sc;
    /**
     * Span (distance) decelerating
     */
    double    Sd;

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
    public void resetProfile(double Amax_in,
                             double Dmax_in,
                             double Vmax_in,
                             double Vi_in,
                             double dist_in,
                             int    Pi_in) {
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

    @NonNull
    @Override
    public String toString() {
        String formatString = "TrapezoidalMotionProfile1D%n" +
                "    dist = %1$20.5f%n"  +
                "    Pi   = %2$20.5f%n"  +
                "    Vi   = %3$20.5f%n"  +
                "    Amax = %4$20.5f%n"  +
                "    Ta   = %5$20.5f%n"  +
                "    Sa   = %6$20.5f%n"  +
                "    Vmax = %7$20.5f%n"  +
                "    Tc   = %8$20.5f%n"  +
                "    Vc   = %9$20.5f%n"  +
                "    Sc   = %10$20.5f%n" +
                "    Dmax = %11$20.5f%n" +
                "    Td   = %12$20.5f%n" +
                "    Sd   = %13$20.5f%n" +
                "    Tt   = %14$20.5f%n";

        return String.format(Locale.US, formatString,
                dist,Pi,Vi,Amax,Ta,Sa,Vmax,Tc,Vc,Sc,Dmax,Td,Sd,Tt);
    }

    /**
     * use to test example motion profiles
     * @param args not used
     */
    public static void main(String[] args) {
        TrapezoidalMotionProfile1D profile = new TrapezoidalMotionProfile1D();

        //
        // Profile variables, initialized to
        // profile 1 - Trinagular Profile - No cruising
        int    Pi   =  0;    // m
        double Vi   =  0.0;  // m/s
        double Amax =  2.0;  // m/s^2
        double Ta   =  3.0;  // a
        double Sa   =  9.0;  // m (0.5*Amax*Ta*Ta)
        double Vmax =  6.0;  // m/s
        double Vc   =  6.0;  // m/s
        double Sc   =  0.0;  // m
        double Dmax = -1.0;  // m/s^2
        double Td   =  6.0;  // s (Vc/Td)
        double Sd   =  18.0; // -0.5*Dmax*Td*Td
        double dist =  27.0; // m
        double Tt   =  9.0;  // s

        Function<TrapezoidalMotionProfile1D, Boolean> test = (TrapezoidalMotionProfile1D p) ->
        {
            return approxEquals( Ta,   p.Ta)   &&
                    approxEquals(Sa,   p.Sa)   &&
                    approxEquals(Vc,   p.Vc)   &&
                    approxEquals(Sc,   p.Sc)   &&
                    approxEquals(Dmax, p.Dmax) &&
                    approxEquals(Td,   p.Td)   &&
                    approxEquals(Sd,   p.Sd)   &&
                    approxEquals(Tt,   p.Tt);
        };

        profile.resetProfile(Amax, Dmax, Vmax, Vi, dist, Pi);
        System.out.println("Profile 1 matched: " + test.apply(profile));

        System.out.println(profile);
    }
}
