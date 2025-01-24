package org.firstinspires.ftc.teamcode.base.motorcontrol;

public interface MotionProfile {
    abstract int  runProfile(double time);
    abstract void resetProfile(double dist_in,
                               double Pi_in,
                               double Vi_in,
                               double Vmax_in,
                               double Amax_in,
                               double Dmax_in);
}


