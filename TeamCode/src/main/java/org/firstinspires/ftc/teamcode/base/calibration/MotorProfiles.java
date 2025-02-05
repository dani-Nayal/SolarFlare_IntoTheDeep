package org.firstinspires.ftc.teamcode.base.calibration;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.teamcode.base.config.MotorConfig;
import org.firstinspires.ftc.teamcode.base.config.Validatable;

public class MotorProfiles implements Validatable {
    private MotorConfig          motorConfig;
    private int                  powerResolution;
    private double               dP;
    private double               Pi;
    private double               Pf;
    private MotorProfileConstP[] motorProfiles;
    public MotorProfiles(MotorConfig motorConfig_in) {
        motorConfig          = motorConfig_in;
        powerResolution      = motorConfig.calibParams.powerResolution;
        dP                   = 2.0/powerResolution;
        motorProfiles        = new MotorProfileConstP[2*powerResolution];
        for(int i=0; i<powerResolution; i+=2) {
            motorProfiles[i] = new MotorProfileConstP(motorConfig);
            motorProfiles[i] = new MotorProfileConstP(motorConfig);
        }
    }

    public void calcProfiles(double Pi_in, double Pf_in) {
        Pi                   = Pi_in;
        Pf                   = Pf_in;
        for(int i=0; i<powerResolution; i++) {
            double power     = -1.0 + i*dP;
        }
    }
    public boolean isValid() {
        return true;
    }
}
