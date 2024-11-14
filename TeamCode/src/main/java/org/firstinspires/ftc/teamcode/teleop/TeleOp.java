package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double hangTarget = 0;
        double extendoPitchTarget = 0;
        double extendoTarget = 0;
        double bucketSlidesTarget = 0;
        double clawWristPosition = 76.5;
        double clawFingerPosition = 50;
        double clawPitchPosition = 217;
        double bucketPosition = 81.51;

        boolean isPressingY = false;
        boolean isPressingY2 = false;
        boolean isPressingTrigger1 = false;
        boolean isPressingBumper2=false;
        boolean isXSequenceActive=false;
        boolean isBSequenceActive=false;
        boolean isASequenceActive=false;

        double kP = 0.015;
        double kPpitch = 0.008;
        ElapsedTime Xtimer = new ElapsedTime();
        ElapsedTime Btimer = new ElapsedTime();
        ElapsedTime Atimer = new ElapsedTime();

        DcMotor extendo = hardwareMap.dcMotor.get("extendo");
        DcMotor extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor hang = hardwareMap.dcMotor.get("hang");
        DcMotor bucketSlides = hardwareMap.dcMotor.get("bucketSlides");

        Servo clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        Servo clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        Servo clawFingers = hardwareMap.servo.get("clawFingers");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo bucket = hardwareMap.servo.get("bucket");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketSlides.setDirection(DcMotor.Direction.REVERSE);
        bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawPitchLeft.setDirection(Servo.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        pinpoint.resetPosAndIMU();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // bucket slides retraction sequence
            if (gamepad1.x){
                isXSequenceActive=true;
                Xtimer.reset();
            }
            if (isXSequenceActive) {
                bucketPosition=81.51;

                if (Xtimer.seconds()>0.5){
                    bucketSlidesTarget=0;
                    isXSequenceActive=false;
                }
            }
            // Intake sequence picking up sample in submersible (claw pitch needs to fit over sub)
            if (gamepad1.b){
                isBSequenceActive=true;
                Btimer.reset();

            }
            if (isBSequenceActive) {
                extendoPitchTarget = 1350;
                clawWristPosition = 76.5;
                if (Btimer.seconds() > 0.7) {
                    extendoTarget = 503;
                    clawPitchPosition = 104;
                    isBSequenceActive=false;
                }
            }
            // Transfer sample
            else if (gamepad1.a){
                isASequenceActive=true;
                Atimer.reset();

            }
            if (isASequenceActive) {
                extendoTarget = 0;
                clawPitchPosition=104;
                clawWristPosition = 76.5;
                if (Atimer.seconds() > 1) {
                    extendoPitchTarget = 0;
                    clawPitchPosition = 217;
                    isASequenceActive=false;
                }
            }

            // Extendo retracted 0 ticks
            // Extendo fully extending 360
            // Dynamic extendo control
            //bro why did you comment out our entire dynamic extendo control
            /*
            if (gamepad1.y){
                extendoTarget = 360;
            }
            if (gamepad1.a){
                extendoTarget = 0;
            }
            */
            if (gamepad1.right_bumper){
                extendoTarget += 30;
            }
            else if (gamepad1.left_bumper){
                extendoTarget -= 30;
            }


            // Extendo pitch transfer / default pos 0 ticks
            // Extendo pitch pickup 1350
            if (gamepad1.dpad_down){
               extendoPitchTarget = 1350;
            }
            if (gamepad1.dpad_up){
               extendoPitchTarget = 0;
            }

            //dyanmic extendo movement - for specimen scoring
            if (gamepad2.right_stick_y>0&&extendoPitchTarget>=50){
                extendoPitchTarget-=50;
            }
            else if (gamepad2.right_stick_y<0&&extendoPitchTarget<=1300){
                extendoPitchTarget+=50;
            }

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (hangTarget == 0) {
                        isPressingY2 = true;
                        hangTarget = 9517;}
                    else hangTarget = 5287;
                }
            }
            else isPressingY2=false;


            // Bucket Slides toggle between min and max positions
            if (gamepad1.y){
                if (!isPressingY) {
                    if (bucketSlidesTarget == 0) {
                        bucketSlidesTarget = 1000;}
                    else {
                        bucketSlidesTarget = 0;
                    }
                    isPressingY=true;
                }
            }
            else isPressingY=false;

            // Claw pitch picking up pos 30.5 degrees
            // Claw transfer pos 217 degrees
            // Claw pitch going into sub 104 degrees
            // Claw pitch to position 0 to 1
            if (gamepad2.dpad_left){
                if (!isPressingBumper2) {
                    if (clawPitchPosition == 104) {
                        clawPitchPosition = 30.5;
                    }
                    else if (clawPitchPosition == 217){
                        clawPitchPosition = 104;
                    }
                }
                isPressingBumper2=true;
            }
            else if (gamepad2.dpad_right){
                if (!isPressingBumper2) {
                    if (clawPitchPosition == 30.5) {
                        clawPitchPosition = 104;
                    }
                    else if (clawPitchPosition == 104) {
                        clawPitchPosition = 217;
                    }
                }
                isPressingBumper2=true;
            }
            else{
                isPressingBumper2=false;
            }

            // Claw finger close 0 degrees
            // Claw finger partially open 50 degrees
            // Claw finger fully open 100 degrees
            // Claw fingers toggle between open and closed

            /*
            if (gamepad2.a) {
                if (!isPressingA2) {
                    if (clawFingers.getPosition() == 50) {
                        clawFingerPosition= 0;
                    } else clawFingerPosition= 50;
                    isPressingA2 = true;
                }
            }
            else isPressingA2 = false;

            if (gamepad2.right_bumper && clawFingers.getPosition()<1){
                clawFingerPosition+=5;
            }
            else if (gamepad2.left_bumper && clawFingers.getPosition()>0) {
                clawFingerPosition-=5;
            }
            */
            if (gamepad2.left_bumper){
                if (!isPressingTrigger1) {
                    if (clawFingerPosition == 100) {
                        clawFingerPosition = 50;
                    }
                    else if (clawFingerPosition == 50){
                        clawFingerPosition = 0;
                    }
                }
                isPressingTrigger1=true;
            }
            else if (gamepad2.right_bumper){
                if (!isPressingTrigger1) {
                    if (clawFingerPosition == 0) {
                        clawFingerPosition = 50;
                    }
                    else if (clawFingerPosition == 50) {
                        clawFingerPosition = 100;
                    }
                }
                isPressingTrigger1=true;
            }
            else{
                isPressingTrigger1=false;
            }
            clawFingers.setPosition(clawFingerPosition/180);
            telemetry.addData("claw finger position", clawFingers.getPosition());

            // Default perpendicular pos 76.5 degrees
            // Claw wrist dynamic movement
            if (gamepad2.b){
                clawWristPosition = 76.5;
            }
            if (gamepad2.left_trigger>0 && clawWristPosition >= 10) {
                clawWristPosition -= 10;
            }
            else if (gamepad2.right_trigger>0 && clawWristPosition <= 66.5) {
                clawWristPosition += 10;
            }
            clawWrist.setPosition(clawWristPosition/180);
            telemetry.addData("claw wrist position", clawWrist.getPosition());

            // BucketTransfer / default pos 81.51 degrees
            // Bucket Deposit pos 190 degrees
            // When bucket slides are going up the bucket will move when the slides are 100 ticks away from max position
            if (gamepad2.dpad_down ){
                bucketPosition = 81.51;
            }
            if (gamepad2.dpad_up){
                bucketPosition = 190;
            }



            if (gamepad1.options) {
                pinpoint.resetPosAndIMU();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.left_trigger>0.5) { // Checks for left trigger input, slows all motors by 50%
                frontLeftPower = 0.5 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.5 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.5 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.5 * (rotY + rotX - rx) / denominator;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * kPpitch);
            hang.setPower((hangTarget - hang.getCurrentPosition()) * kP);
            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * kP);
            clawPitchLeft.setPosition(clawPitchPosition/270);
            clawPitchRight.setPosition(clawPitchPosition/270);
            bucket.setPosition(bucketPosition/270);

            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendo target", extendoTarget);
            telemetry.addData("extendo pitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("extendo pitch target", extendoPitchTarget);
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.addData("hang target", hangTarget);
            telemetry.addData("bucket pos", bucketSlides.getCurrentPosition());
            telemetry.addData("bucket target", bucketSlidesTarget);
            telemetry.addData("left claw pitch position", clawPitchLeft.getPosition());
            telemetry.addData("right claw pitch position", clawPitchRight.getPosition());
            telemetry.addData("bucket pos", bucket.getPosition());
            telemetry.addData("bucket target", bucketPosition);
            telemetry.addData("bot heading", botHeading);
            telemetry.update();
        }
    }
}