package org.firstinspires.ftc.teamcode.base.calibration;

import androidx.annotation.NonNull;

import java.util.Locale;

public class NumberPair {
    public double n1;
    public double n2;

    public NumberPair(double n1, double n2) {
        this.n1 = n1;
        this.n2 = n2;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "(%1$10.3f,%2$10.3f)", n1, n2);
    }
}
