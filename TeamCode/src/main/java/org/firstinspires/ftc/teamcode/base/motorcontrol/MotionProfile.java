package org.firstinspires.ftc.teamcode.base.motorcontrol;

public interface MotionProfile {
    abstract int    runProfile(double time);
    abstract void   resetProfile(double Amax_in,
                                 double Dmax_in,
                                 double Vmax_in,
                                 double Vi_in,
                                 double dist_in,
                                 int Pi_in);
}
