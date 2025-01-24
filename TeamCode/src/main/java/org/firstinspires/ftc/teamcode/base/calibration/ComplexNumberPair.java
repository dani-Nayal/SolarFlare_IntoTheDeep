package org.firstinspires.ftc.teamcode.base.calibration;

import androidx.annotation.NonNull;

import java.util.Locale;

public class ComplexNumberPair {
    public ComplexNumber n1;
    public ComplexNumber n2;

    ComplexNumberPair(ComplexNumber n1, ComplexNumber n2) {
        this.n1 = n1;
        this.n2 = n2;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "[%1$s, %2$s]", n1, n2);
    }
}
