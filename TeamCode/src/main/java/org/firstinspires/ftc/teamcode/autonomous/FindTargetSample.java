package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfig;

public class FindTargetSample {
    static ElapsedTime timer = new ElapsedTime();
    static double scanningTime = 3000;
    static double targetTX = 0;
    static double targetTY = 0;
    static double smallestTargetDeviance;
    // Tune later
    static double minimumTAValue = 0.01;
    static LLResultTypes.DetectorResult closestToTargetDetection = null;
    static HardwareConfig hw = HardwareConfig.getInstance();
    private static boolean isClassNameBeingDetected(String className, String... imageClassNames) {
        for (String name : imageClassNames) {
            if (className.equals(name)) {
                return true;
            }
        }
        return false;
    }
    private static double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    public static LLResultTypes.DetectorResult findTargetSample(String... imageClassNames){

        timer.reset();

        while (timer.milliseconds() < scanningTime){

            // Reset value
            smallestTargetDeviance = 1000000000;

            LLResult result = hw.getLimelightConfig().limelight.getLatestResult();

            if (result != null && result.isValid()) {

                for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
                    if (isClassNameBeingDetected(detection.getClassName(), imageClassNames)
                    && detection.getTargetArea() > minimumTAValue) {
                        double currentTX = detection.getTargetXDegrees();
                        double currentTY = detection.getTargetYDegrees();
                        double targetDeviance = calculateDistance(currentTX, currentTY, targetTX, targetTY);

                        if (targetDeviance < smallestTargetDeviance) {
                            closestToTargetDetection = detection;
                        }
                    }
                }
            }
        }
        return closestToTargetDetection;
    }
}