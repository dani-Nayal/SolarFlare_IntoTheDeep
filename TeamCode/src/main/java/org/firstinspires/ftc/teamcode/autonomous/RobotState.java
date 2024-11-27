package org.firstinspires.ftc.teamcode.autonomous;

public class RobotState {
    public double extendoTarget = 0;
    public double extendoPitchTarget = 0;
    public double clawPitchPosition = 15;
    public double clawFingerPosition = 0;
    public double clawWristPosition = 76.5;
    public double bucketSlidesTarget = 0;
    public double bucketPosition = 85;
    public double hangTarget = 0;

    public double getExtendoTarget() {
        return extendoTarget;
    }

    public void setExtendoTarget(double extendoTarget) {
        this.extendoTarget = extendoTarget;
    }

    public double getExtendoPitchTarget() {
        return extendoPitchTarget;
    }

    public void setExtendoPitchTarget(double extendoPitchTarget) {
        this.extendoPitchTarget = extendoPitchTarget;
    }

    public double getClawPitchPosition() {
        return clawPitchPosition;
    }

    public void setClawPitchPosition(double clawPitchPosition) {
        this.clawPitchPosition = clawPitchPosition;
    }

    public double getClawFingerPosition() {
        return clawFingerPosition;
    }

    public void setClawFingerPosition(double clawFingerPosition) {
        this.clawFingerPosition = clawFingerPosition;
    }

    public double getClawWristPosition() {
        return clawWristPosition;
    }

    public void setClawWristPosition(double clawWristPosition) {
        this.clawWristPosition = clawWristPosition;
    }

    public double getBucketSlidesTarget() {
        return bucketSlidesTarget;
    }

    public void setBucketSlidesTarget(double bucketSlidesTarget) {
        this.bucketSlidesTarget = bucketSlidesTarget;
    }

    public double getBucketPosition() {
        return bucketPosition;
    }

    public void setBucketPosition(double bucketPosition) {
        this.bucketPosition = bucketPosition;
    }

    public double getHangTarget() {
        return hangTarget;
    }

    public void setHangTarget(double hangTarget) {
        this.hangTarget = hangTarget;
    }
}
