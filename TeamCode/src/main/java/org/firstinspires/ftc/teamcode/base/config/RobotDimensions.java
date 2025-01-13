package org.firstinspires.ftc.teamcode.base.config;

import androidx.annotation.NonNull;

public class RobotDimensions {
    public double length = 0.0;
    public double width  = 0.0;

    public RobotDimensions() {
        this.length = 0.0;
        this.width  = 0.0;
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

