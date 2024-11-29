package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareConfig;

public class CustomActions {
    RobotState state;
    HardwareConfig hw;
    public CustomActions(RobotState state, HardwareConfig hw) {
        this.state = state;
        this.hw = hw;
    }
    enum MotorName {
        EXTENDO,
        EXTENDO_PITCH,
        BUCKET_SLIDES,
        HANG
    }

    enum ServoName {
        CLAW_PITCH,
        CLAW_FINGER,
        CLAW_WRIST,
        BUCKET
    }

    public class SetMotorTarget implements Action {
        MotorName motorName;
        int target;

        public SetMotorTarget(MotorName motorName, int target) {
            this.motorName = motorName;
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (motorName) {
                case EXTENDO:
                    state.setExtendoTarget(target);
                    break;
                case EXTENDO_PITCH:
                    state.setExtendoPitchTarget(target);
                    break;
                case BUCKET_SLIDES:
                    state.setBucketSlidesTarget(target);
                    break;
                case HANG:
                    state.setHangTarget(target);
                    break;
                default:
                    throw new IllegalArgumentException("Illegal Motor Name " + motorName);
            }
            return false;
        }
    }

    public class SetServoPosition implements Action {
        ServoName servoName;
        double degrees;

        public SetServoPosition(ServoName servoName, double degrees) {
            this.servoName = servoName;
            this.degrees = degrees;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (servoName) {
                case CLAW_PITCH:
                    state.setClawPitchPosition(degrees);
                    break;
                case CLAW_FINGER:
                    state.setClawFingerPosition(degrees);
                    break;
                case CLAW_WRIST:
                    state.setClawWristPosition(degrees);
                    break;
                case BUCKET:
                    state.setBucketPosition(degrees);
                    break;
                default:
                    throw new IllegalArgumentException("Illegal Servo Name " + servoName);
            }
            return false;
        }
    }

    public class GlobalPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hw.extendo.motor.setPower((state.getExtendoTarget() - hw.extendo.motor.getCurrentPosition()) * hw.extendo.kP);
            hw.extendoPitch.motor.setPower((state.getExtendoPitchTarget() - hw.extendoPitch.motor.getCurrentPosition()) * 0.005);
            hw.hang.motor.setPower((state.getHangTarget() - hw.hang.motor.getCurrentPosition()) * hw.extendo.kP);
            hw.bucketSlides.motor.setPower((state.getBucketSlidesTarget() - hw.bucketSlides.motor.getCurrentPosition()) * hw.extendo.kP);

            hw.clawPitchLeft.servo.setPosition(state.getClawPitchPosition() / 270);
            hw.clawPitchRight.servo.setPosition(state.getClawPitchPosition() / 270);
            hw.bucket.servo.setPosition(state.getBucketPosition() / 270);
            hw.clawFingers.servo.setPosition(state.getClawFingerPosition() / 180);
            hw.clawWrist.servo.setPosition(state.getClawWristPosition() / 180);
            return true;
        }
    }
    public class UpdateTelemetry implements Action{
        Telemetry telemetry;
        public UpdateTelemetry(Telemetry telemetry){
            this.telemetry = telemetry;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            telemetry.addData("extendo position", hw.extendo.motor.getCurrentPosition());
            telemetry.addData("extendo target", state.extendoTarget);
            telemetry.addData("extendo pitch position", hw.extendoPitch.motor.getCurrentPosition() );
            telemetry.addData("extendo pitch target", state.extendoPitchTarget);
            telemetry.addData("hang pos", hw.hang.motor.getCurrentPosition());
            telemetry.addData("hang target", state.hangTarget);
            telemetry.addData("bucketSlides pos", hw.bucketSlides.motor.getCurrentPosition());
            telemetry.addData("bucketSlides target", state.bucketSlidesTarget);

            telemetry.addData("left claw pitch position", hw.clawPitchLeft.servo.getPosition());
            telemetry.addData("right claw pitch position", hw.clawPitchRight.servo.getPosition());
            telemetry.addData("claw finger position", hw.clawFingers.servo.getPosition());
            telemetry.addData("claw finger position in degree", state.clawFingerPosition);
            telemetry.addData("claw wrist position", hw.clawWrist.servo.getPosition());
            telemetry.addData("bucket pos", hw.bucket.servo.getPosition());
            telemetry.addData("bucket target", state.bucketPosition);

            telemetry.addData("pinpoint heading", hw.pinpoint.pinpoint.getYawScalar());
            telemetry.addData("Control hub IMU heading", hw.imu.imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("pinpoint x", hw.pinpoint.pinpoint.getPosX());
            telemetry.addData("pinpoint y", hw.pinpoint.pinpoint.getPosY());
            telemetry.update();
            return true;
        }
    }

    public Action updateTelemetry(Telemetry telemetry){return new UpdateTelemetry(telemetry);};
    public Action globalPID() {return new GlobalPID();}
    public Action setExtendoTarget(int target) {return new SetMotorTarget(CustomActions.MotorName.EXTENDO, target);}
    public Action setExtendoPitchTarget(int target) {return new SetMotorTarget(CustomActions.MotorName.EXTENDO_PITCH, target);}
    public Action setBucketSlidesTarget(int target) {return new SetMotorTarget(CustomActions.MotorName.BUCKET_SLIDES, target);}
    public Action setClawPitchPosition(double degrees) {return new SetServoPosition(CustomActions.ServoName.CLAW_PITCH, degrees);}
    public Action setClawFingerPosition(double degrees) {return new SetServoPosition(CustomActions.ServoName.CLAW_FINGER, degrees);}
    public Action setClawWristPosition(double degrees) {return new SetServoPosition(CustomActions.ServoName.CLAW_WRIST, degrees);}
    public Action setBucketPosition(double degrees) {return new SetServoPosition(CustomActions.ServoName.BUCKET, degrees);}
}
