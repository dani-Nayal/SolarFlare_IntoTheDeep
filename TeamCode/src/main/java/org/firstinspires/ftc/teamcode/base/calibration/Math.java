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
