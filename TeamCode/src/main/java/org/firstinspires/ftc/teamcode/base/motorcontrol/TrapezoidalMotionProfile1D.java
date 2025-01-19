package org.firstinspires.ftc.teamcode.base.motorcontrol;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.round;

import static org.firstinspires.ftc.teamcode.base.calibration.Math.solveQuadraticEquation;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.calibration.CalculationException;
import org.firstinspires.ftc.teamcode.base.calibration.ComplexNumberPair;

public class TrapezoidalMotionProfile1D implements MotionProfile {
    /**
     * FTC Dashboard Telemetry
     */
    Telemetry telemetryDash = FtcDashboard.getInstance().getTelemetry();
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
                             double Vmax_in,
                             double Vi_in,
                             double dist_in,
                             int Pi_in) {
        dist                    = dist_in;
        Pi                      = Pi_in;
        Vi                      = Vi_in;
        Vmax                    = signum(dist)*abs(Vmax_in);
        Amax                    = signum(dist)*abs(Amax_in);
        Dmax                    = -1*Amax;
        // Dmax                 = -1*signum(dist)*abs(Dmax_in);

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

        Ta                      = (Vc-Vi) / Amax;
        Td                      = Vc / Dmax;
        Sa                      = Vi*Ta + 0.5*Amax*Ta*Ta;
        Sd                      = 0.5*Vc*Td;
        Sc                      = dist - Sa - Sd;
        Tc                      = Sc / Vc;

        telemetryDash.addData("Pi", Pi);
        telemetryDash.addData("Vi", Vi);
        telemetryDash.addData("distance", dist);
        telemetryDash.addData("Max Velocity", Vmax);
        telemetryDash.addData("Cruise Velocity", Vc);
        telemetryDash.addData("Accel Span", Sa);
        telemetryDash.addData("Cruise Span", Sc);
        telemetryDash.addData("Decel Span", Sd);
        telemetryDash.update();
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
}
