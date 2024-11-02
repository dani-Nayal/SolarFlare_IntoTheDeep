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

        boolean isPressingA = false;
        boolean isPressingY2 = false;
        boolean isPressingA2 = false;

        double kP = 0.015;

        DcMotor extendo = hardwareMap.dcMotor.get("extendo");
        DcMotor extendoPitch = hardwareMap.dcMotor.get("extendoPitch");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor hang = hardwareMap.dcMotor.get("hang");
        DcMotor bucketSlides = hardwareMap.dcMotor.get("bucket");

        Servo clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
        Servo clawPitchRight = hardwareMap.servo.get("clawPitchRight");
        Servo clawFingers = hardwareMap.servo.get("clawFingers");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo bucket = hardwareMap.servo.get("bucket");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Test Reverse
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Test Reverse
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

            // Dynamic extendo control
            if (gamepad1.left_bumper){
                extendoTarget += 15;
            }
            else if (gamepad1.right_bumper){
                extendoTarget -= 15;
            }
            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendo target", extendoTarget);

            // Extendo pitch presets
            if (gamepad1.a){
                // extendoPitchTarget = down position
            }
            if (gamepad1.x){
                // extendoPitchTarget = default / intake position
            }
            extendoPitch.setPower((extendoPitchTarget - extendo.getCurrentPosition()) * kP);
            telemetry.addData("extendo pitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("extendo pitch target", extendoPitchTarget);

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (hangTarget == 0) {
                        isPressingY2 = true;
                        hangTarget = 3634;}
                    else hangTarget = 0;
                }
            }
            else isPressingY2=false;

            hang.setPower((hangTarget = hang.getCurrentPosition()) * kP);
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.addData("hang target", hangTarget);

            // Bucket Slides toggle between min and max positions
            if (gamepad1.a){
                if (!isPressingA) {
                    if (bucketSlidesTarget == 0) {
                        isPressingA=true;
                        bucketSlidesTarget = 1891;}
                    else bucketSlidesTarget = 0;
                }
            }
            else isPressingA=false;

            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * kP);
            telemetry.addData("bucket pos", bucketSlides.getCurrentPosition());
            telemetry.addData("bucket target", bucketSlidesTarget);

            // Claw pitch to position 0 to 1
            if (gamepad2.left_bumper){
                clawPitchLeft.setPosition(0);
                clawPitchRight.setPosition(0);
            }
            if (gamepad2.right_bumper){
                clawPitchLeft.setPosition(1);
                clawPitchRight.setPosition(1);
            }
            telemetry.addData("left claw pitch position", clawPitchLeft.getPosition());
            telemetry.addData("right claw pitch position", clawPitchRight.getPosition());

            // Claw fingers toggle between open and closed
            if (gamepad2.a) {
                if (!isPressingA2) {
                    if (clawFingers.getPosition() == 0.8) {
                        isPressingA2 = true;
                        clawFingers.setPosition(1);
                    }
                    else clawFingers.setPosition(0.8);
                }
            }
            else isPressingA2 = false;
            telemetry.addData("claw finger position", clawFingers.getPosition());

            // Claw wrist dynamic movement
            if (gamepad2.left_bumper && clawWrist.getPosition() > 0){
                clawWristPosition -= 0.01;
            }
            if (gamepad2.right_bumper && clawWrist.getPosition() < 1){
                clawWristPosition += 0.01;
            }
            clawWrist.setPosition(clawWristPosition);
            telemetry.addData("claw wrist position", clawWrist.getPosition());

            // When bucket slides are going up the bucket will move when the slides are 100 ticks away from max position
            if (bucketSlidesTarget - bucketSlides.getCurrentPosition()<100){
                if (bucketSlidesTarget==1891){
                    bucket.setPosition(1);
                }
                else{
                    bucket.setPosition(0.2);
                }
            }

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