package org.firstinspires.ftc.teamcode.base.config;

import androidx.annotation.NonNull;

public class RobotDimensions {
    public double length = 15.364;
    public double width  = 14.375;

    public RobotDimensions() {
        this.length = 15.364;
        this.width  = 14.375;
    }
    public RobotDimensions(double length, double width) {
        this.length = length;
        this.width  = width;
    }

    @NonNull
    @Override
    public String toString() {
        return "RobotDimensions(length=" + length + ", width=" + width + ")";
    }
}

