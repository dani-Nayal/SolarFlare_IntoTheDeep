package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.HardwareConfig;

import java.util.Objects;

public class FindClosestDetection {
    public static LLResultTypes.DetectorResult findClosestDetection(HardwareConfig hw, String... imageClassNames) {

        LLResult result = hw.getLimelightConfig().limelight.getLatestResult();

        LLResultTypes.DetectorResult closestDetection = null;

        if (result != null && result.isValid()) {
            double greatestTA = 0.0;

            for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
                String detectedClassName = detection.getClassName();

                for (String imageClassName : imageClassNames) {

                    if (Objects.equals(detectedClassName, imageClassName)) {
                        double currentTA = detection.getTargetArea();

                        if (currentTA > greatestTA) {
                            greatestTA = currentTA;
                            closestDetection = detection;
                        }

                    }
                }
            }
        }
        return closestDetection;
    }
}
