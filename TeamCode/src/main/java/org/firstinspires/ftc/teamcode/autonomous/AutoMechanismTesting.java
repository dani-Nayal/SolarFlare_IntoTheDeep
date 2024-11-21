package org.firstinspires.ftc.teamcode.autonomous;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
        import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PIDCoefficients;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous
public class AutoMechanismTesting extends LinearOpMode {
    public double extendoTarget;
    public double extendoPitchTarget;
    public double clawPitchPosition;
    public double clawFingerPosition;
    public double clawWristPosition;
    public double bucketSlidesTarget;
    public double bucketPosition;
    public double hangTarget;

    public class CustomActions{
        DcMotor extendo = hardwareMap.dcMotor.get("extendo");
        DcMotor extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        DcMotor hang = hardwareMap.dcMotor.get("hang");
        DcMotor bucketSlides = hardwareMap.dcMotor.get("bucketSlides");

        Servo clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        Servo clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        Servo clawFingers = hardwareMap.servo.get("clawFingers");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo bucket = hardwareMap.servo.get("bucket");
        public class Initialize implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                extendo.setDirection(DcMotorSimple.Direction.REVERSE);

                extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                bucketSlides.setDirection(DcMotor.Direction.REVERSE);
                bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                clawPitchLeft.setDirection(Servo.Direction.REVERSE);

                hangTarget = 0;
                extendoPitchTarget = 0;
                extendoTarget = 0;
                bucketSlidesTarget = 0;
                clawWristPosition = 76.5;
                clawFingerPosition = 50;
                clawPitchPosition = 200;
                bucketPosition = 81.51;
                return false;
            }
        }
        public Action initialize(){return new CustomActions.Initialize();}
        public class SetPositions implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket){

                extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * PIDCoefficients.kP);
                extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * PIDCoefficients.kP);
                hang.setPower((hangTarget - hang.getCurrentPosition()) * PIDCoefficients.kP);
                bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * PIDCoefficients.kP);
                clawPitchLeft.setPosition(clawPitchPosition/270);
                clawPitchRight.setPosition(clawPitchPosition/270);
                bucket.setPosition(bucketPosition/270);
                clawFingers.setPosition(clawFingerPosition/270);
                clawWrist.setPosition(clawWristPosition/180);
                return true;
            }
        }
        public Action setPositions(){return new CustomActions.SetPositions();}
        public class SetExtendoTarget implements Action{
            private double target;
            public SetExtendoTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                extendoTarget = target;
                return false;
            }
        }
        public Action setExtendoTarget(double target){return new CustomActions.SetExtendoTarget(target);}
        public class SetExtendoPitchTarget implements Action{
            private double target;
            public SetExtendoPitchTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                extendoPitchTarget = target;
                return false;
            }
        }
        public Action setExtendoPitchTarget(double target){return new CustomActions.SetExtendoPitchTarget(target);}
        public class SetBucketSlidesTarget implements Action{
            private double target;
            public SetBucketSlidesTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                bucketSlidesTarget = target;
                return false;
            }
        }
        public Action setBucketSlidesTarget(double target){return new CustomActions.SetBucketSlidesTarget(target);}
        public class SetHangTarget implements Action{
            private double target;
            public SetHangTarget(double target){
                this.target = target;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                hangTarget = target;
                return false;
            }
        }
        public Action setHangTarget(double target){return new CustomActions.SetHangTarget(target);}
        public class SetClawPitchPosition implements Action{
            private double position;
            public SetClawPitchPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawPitchPosition = position;
                return false;
            }
        }
        public Action setClawPitchPosition(double position){return new CustomActions.SetHangTarget(position);}
        public class SetClawFingerPosition implements Action{
            private double position;
            public SetClawFingerPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawFingerPosition = position;
                return false;
            }
        }
        public Action setClawFingerPosition(double position){return new CustomActions.SetHangTarget(position);}
        public class SetClawWristPosition implements Action{
            private double position;
            public SetClawWristPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                clawWristPosition = position;
                return false;
            }
        }
        public Action setClawWristPosition(double position){return new CustomActions.SetHangTarget(position);}
        public class SetBucketPosition implements Action{
            private double position;
            public SetBucketPosition(double position){
                this.position = position;
            }
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                bucketPosition = position;
                return false;
            }
        }
        public Action setBucketPosition(double position){return new CustomActions.SetHangTarget(position);}
    }
    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-42,-62.5,Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        CustomActions customActions = new CustomActions();

        Actions.runBlocking(
                
        );
    }

}
