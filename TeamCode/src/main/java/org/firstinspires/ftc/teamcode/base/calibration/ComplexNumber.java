package org.firstinspires.ftc.teamcode.base.calibration;

import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;

import java.util.Locale;

public class ComplexNumber {
    /**
     * a: Real part
     */
    public double a;
    /**
     * b: Imaginary part
     */
    public double b;

    ComplexNumber(double a, double b) {
        this.a = a;
        this.b = b;
    }

    public boolean isReal() {
        return b == 0.0;
    }

    public boolean isComplex() {
        return b != 0.0;
    }

    public double getNorm() {
        return sqrt(a*a + b*b);
    }

    public int compareTo(ComplexNumber other) {
        return Double.compare(this.getNorm(), other.getNorm());
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "(%1$.5f,%2$.5f)", a, b);
    }
}
