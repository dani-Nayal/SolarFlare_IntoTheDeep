package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double hangTarget = 0;
        double extendoPitchTarget = 0;
        double extendoTarget = 0;
        double bucketSlidesTarget = 0;
        double clawWristPosition = 0;
        double clawFingerPosition = 0;
        double clawPitchPosition = 0;
        double bucketPosition = 0;

        boolean isPressingA = false;
        boolean isPressingY2 = false;
        boolean isPressingA2 = false;
        boolean isPressingBumper2=false;
        boolean isPressingTrigger1=false;
        double kP = 0.015;

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

        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setDirection(DcMotor.Direction.REVERSE);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketSlides.setDirection(DcMotor.Direction.REVERSE);
        bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawPitchLeft.setDirection(Servo.Direction.REVERSE);
        // Test clawPitchLeft Reverse
        // Test clawPitchRight Reverse
        // Test clawFingers Reverse
        // Test clawWrist Reverse
        // Test bucket Reverse

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Extendo retracted 0 ticks
            // Extendo fully extending 360
            // Dynamic extendo control
            if (gamepad1.y){
                extendoTarget = 360;
            }
            if (gamepad1.a){
                extendoTarget = 0;
            }
            if (gamepad1.right_stick_y > 0){
                extendoTarget += 7;
            }
            else if (gamepad1.right_stick_y < 0){
                extendoTarget -= 7;
            }
            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendo target", extendoTarget);

            // Extendo pitch transfer / default pos 0 ticks
            // Extendo pitch pickup 1421
            if (gamepad1.dpad_down){
               extendoPitchTarget = 1421;
            }
            if (gamepad1.dpad_up){
               extendoPitchTarget = 0;
            }
            extendoPitch.setPower((extendoPitchTarget - extendo.getCurrentPosition()) * kP);
            telemetry.addData("extendo pitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("extendo pitch target", extendoPitchTarget);

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (hangTarget == 5287) {
                        isPressingY2 = true;
                        hangTarget = 9517;}
                    else hangTarget = 5287;
                }
            }
            else isPressingY2=false;

            hang.setPower((hangTarget - hang.getCurrentPosition()) * kP);
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.addData("hang target", hangTarget);

            // Bucket Slides toggle between min and max positions
            if (gamepad1.x){
                if (!isPressingA) {
                    if (bucketSlidesTarget == 0) {
                        isPressingA=true;
                        bucketSlidesTarget = 1200;}
                    else bucketSlidesTarget = 0;
                }
            }
            else isPressingA=false;

            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * kP);
            telemetry.addData("bucket pos", bucketSlides.getCurrentPosition());
            telemetry.addData("bucket target", bucketSlidesTarget);

            // Claw pitch picking up pos 30.5 degrees
            // Claw transfer pos 217 degrees
            // Claw pitch going into sub 104 degrees
            // Claw pitch to position 0 to 1
            if (gamepad2.dpad_left){
                if (!isPressingBumper2) {
                    if (clawPitchPosition == 90) {
                        clawPitchPosition = 0;
                    }
                    else if (clawPitchPosition == 180){
                        clawPitchPosition = 90;
                    }
                }
                isPressingBumper2=true;
            }
            else if (gamepad2.dpad_right){
                if (!isPressingBumper2) {
                    if (clawPitchPosition == 0) {
                        clawPitchPosition = 90;
                    }
                    else if (clawPitchPosition == 90) {
                        clawPitchPosition = 180;
                    }
                }
                isPressingBumper2=true;
            }
            else{
                isPressingBumper2=false;
            }
            clawPitchLeft.setPosition(clawPitchPosition/270);
            clawPitchRight.setPosition(clawPitchPosition/270);
            telemetry.addData("left claw pitch position", clawPitchLeft.getPosition());
            telemetry.addData("right claw pitch position", clawPitchRight.getPosition());


            // Claw finger close 0 degrees
            // Claw finger open 50 degrees
            // Claw fingers toggle between open and closed
            if (gamepad2.a) {
                if (!isPressingA2) {
                    if (clawFingers.getPosition() == 50/270) {
                        clawFingerPosition= 0;
                    } else clawFingerPosition= 0;
                    isPressingA2 = true;
                }
            }
            else isPressingA2 = false;
            // Dynamic claw
            if (gamepad2.right_bumper && clawFingers.getPosition()<1){
                clawFingerPosition+=0.01;
            }
            else if (gamepad2.left_bumper && clawFingers.getPosition()>0) {
                clawFingerPosition-=0.01;
            }
            clawFingers.setPosition(clawFingerPosition);
            telemetry.addData("claw finger position", clawFingers.getPosition());

            // Default perpendicular pos 76.5 degrees
            // Claw wrist dynamic movement
            if (gamepad2.b){
                clawWristPosition = 76.5;
            }
            if (gamepad1.left_trigger>0 && clawWristPosition >= 10) {
                clawWristPosition -= 10;
            }
            else if (gamepad1.right_trigger>0 && clawWristPosition <= 66.5) {
                clawWristPosition += 10;
            }
            clawWrist.setPosition(clawWristPosition/270);
            telemetry.addData("claw wrist position", clawWrist.getPosition());

            // BucketTransfer / default pos 81.51 degrees
            // Bucket Deposit pos 190 degrees
            // When bucket slides are going up the bucket will move when the slides are 100 ticks away from max position
            if (gamepad1.dpad_right){
                bucketPosition = 81.51;
            }
            if (gamepad1.dpad_left){
                bucketPosition = 190;
            }
            /*if (bucketSlidesTarget - bucketSlides.getCurrentPosition() < 100){
                if (bucketSlidesTarget==1891){
                    bucketPosition = 190;
                }
                else{
                    bucketPosition = 81.51;
                }
            }

             */
            bucket.setPosition(bucketPosition/270);
            telemetry.addData("bucket pos", bucket.getPosition());
            telemetry.addData("bucket target", bucketPosition);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.update();
        }
    }
}