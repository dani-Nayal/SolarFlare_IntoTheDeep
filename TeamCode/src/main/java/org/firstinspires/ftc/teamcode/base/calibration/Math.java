package org.firstinspires.ftc.teamcode.base.calibration;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import java.util.Locale;

public class Math {
    public static double NUMERICAL_TOLERANCE_RATIO = 1E-3;

    public static ComplexNumberPair solveQuadraticEquation(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;
        if (discriminant > 0) {
            double root1 = (-b + sqrt(discriminant)) / (2 * a);
            double root2 = (-b - sqrt(discriminant)) / (2 * a);
            // System.out.println("Roots are real and different:");
            // System.out.println("Root 1: " + root1);
            // System.out.println("Root 2: " + root2);
            return new ComplexNumberPair(
                    new ComplexNumber(root1, 0.0),
                    new ComplexNumber(root2, 0.0)
            );
        } else if (discriminant == 0) {
            double root = -b / (2 * a);
            // System.out.println("Roots are real and equal:");
            // System.out.println("Root: " + root);
            return new ComplexNumberPair(
                    new ComplexNumber(root, 0.0),
                    new ComplexNumber(root, 0.0)
            );
        } else {
            // System.out.println("Roots are complex and different:");
            double realPart = -b / (2 * a);
            double imaginaryPart = sqrt(-discriminant) / (2 * a);
            // System.out.println("Root 1: " + realPart + " + " + imaginaryPart + "i");
            // System.out.println("Root 2: " + realPart + " - " + imaginaryPart + "i");
            return new ComplexNumberPair(
                    new ComplexNumber(realPart,  imaginaryPart),
                    new ComplexNumber(realPart, -imaginaryPart)
            );
        }
    }

    public static boolean approxEquals(double n1, double n2) {
        if(n1 == 0.0 && n2 == 0.0)
            return true;

        // System.out.println(String.format(Locale.US, "n1=%1$20.5f n2=%2$20.5f", n1, n2));

        double errorRatio = abs(n1-n2)/(abs(n1)+abs(n2));
        // System.out.println(String.format(Locale.US, "Error Ratio=%1$20.10f", errorRatio));

        return errorRatio < NUMERICAL_TOLERANCE_RATIO;
    }
}
