
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.InitializeMechanisms;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class MecanumTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double hangTarget = 0;
        double extendoPitchTarget = 0;
        double extendoTarget = 0;
        double bucketSlidesTarget = 300;
        double clawWristPosition = 76.5;
        double clawFingerPosition = 50;
        double clawPitchPosition = 195;
        double bucketPosition = 85;
        boolean isXSequenceActive=false;
        boolean isASequenceActive=false;
        boolean isBSequenceActive=false;
        boolean isB2SequenceActive=false;
        boolean isPressingY = false;
        boolean isPressingY2 = false;
        boolean isPressingA2 = false;
        boolean isPressingX2 = false;
        boolean isPressingBumper2=false;
        boolean isPressingTrigger1=false;
        boolean isPressingDpad=false;
        double kP = 0.015;
        double kPpitch = 0.005;

        ElapsedTime Xtimer = new ElapsedTime();
        ElapsedTime Btimer = new ElapsedTime();
        ElapsedTime B2timer = new ElapsedTime();
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
        GoBildaPinpointDriverRR pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");

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

        pinpoint.recalibrateIMU();

        pinpoint.resetPosAndIMU();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            pinpoint.update();

            // bucket slides retraction sequence
            if (gamepad1.x){
                isXSequenceActive=true;
                Xtimer.reset();
            }
            if (isXSequenceActive) {
                bucketPosition=85;

                if (Xtimer.seconds()>0.3){
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
                if (clawWristPosition != 76.5){
                    clawWristPosition = 76.5;
                    bucketPosition=85;
                    if (Atimer.seconds() > 0.3){
                        clawPitchPosition = 104;

                    }
                    if (Atimer.seconds() > 0.5){
                        extendoTarget=0;
                    }

                    if (Atimer.seconds() > 0.8) {
                        if (!(extendo.getCurrentPosition()>150)) {
                            extendoPitchTarget = 0;
                        }
                        clawPitchPosition = 195;
                        isASequenceActive=false;
                    }
                }
                else{
                    bucketPosition=85;
                    clawPitchPosition = 104;


                    if (Atimer.seconds() > 0.2){
                        extendoTarget=0;
                    }

                    if (Atimer.seconds() > 0.5) {
                        if (!(extendo.getCurrentPosition()>150)) {
                            extendoPitchTarget = 0;
                        }
                        clawPitchPosition = 195;
                        isASequenceActive=false;
                    }
                }

            }
            //specimen setup sequence (including retraction of extendo after specimen pickup)
            else if (gamepad2.b){
                isB2SequenceActive=true;
                B2timer.reset();
            }
            if (isB2SequenceActive) {
                clawWristPosition =76.5;

                if (B2timer.seconds()>0.3){
                    clawPitchPosition = 104;
                    bucketPosition=205;
                    extendoTarget=0;
                }

                if (B2timer.seconds() > 0.6) {
                    extendoPitchTarget = 450;
                }
                if (B2timer.seconds() > 1) {
                    extendoTarget=500;
                    isB2SequenceActive=false;
                }
            }
            // Extendo retracted 0 ticks
            // Extendo fully extending 36050
            // Dynamic extendo control

            if ((gamepad1.right_bumper) && (extendoTarget <= 473)){
                extendoTarget += 30;
            }
            else if ((gamepad1.left_bumper) && (extendoTarget >= 30)){
                extendoTarget -= 30;
            }

            // Extendo pitch transfer / default pos 0 ticks
            // Extendo pitch pickup 1350
            if (gamepad1.dpad_down && !(extendo.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (extendoPitchTarget == 0) {
                        extendoPitchTarget = 450;
                    }
                    else if (extendoPitchTarget == 450 || extendoPitchTarget == 760){
                        extendoPitchTarget = 1421;
                    }
                }
                isPressingDpad=true;
            }
            else if (gamepad1.dpad_up && !(extendo.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (extendoPitchTarget == 1421 || extendoPitchTarget == 760) {
                        extendoPitchTarget = 450;
                    }
                    else if (extendoPitchTarget == 450) {
                        extendoPitchTarget = 0;
                    }
                }
                isPressingDpad=true;
            }
            else{
                isPressingDpad=false;
            }

            /*
            //dynamic extendo movement - for specimen scoring
            if (-gamepad2.right_stick_y>0&&extendoPitchTarget>=50){
                extendoPitchTarget-=50;
            }
            else if (-gamepad2.right_stick_y<0&&extendoPitchTarget<=1300){
                extendoPitchTarget+=50;
            }
            */
            if (gamepad2.x&&extendoPitchTarget>=210){
                if (!isPressingX2) {
                    extendoPitchTarget=760;
                    isPressingX2=true;
                }
            }
            else isPressingX2=false;

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (hangTarget == 0) {
                        isPressingY2 = true;
                        hangTarget = 9517;}
                    else if (hangTarget == 9517) {
                        hangTarget = 5400;
                        isPressingY2 = true;
                    }
                    else if (hangTarget == 5400){
                        hangTarget = 0;
                        isPressingY2 = true;
                    }
                }
            }
            else isPressingY2=false;

            // Bucket Slides toggle between min and max positions
            if (gamepad1.y){
                if (!isPressingY) {
                    if (bucketSlidesTarget == 0 || bucketSlidesTarget==300) {
                        bucketSlidesTarget = 1100;}
                    else {
                        bucketSlidesTarget = 0;
                    }
                    isPressingY=true;
                }
            }
            else isPressingY=false;

            // Claw pitch picking up pos 30.5 degrees
            // Claw transfer pos 195 degrees
            // Claw pitch going into sub 104 degrees
            // Claw pitch to position 0 to 1
            if (gamepad2.dpad_left){
                if (!isPressingBumper2) {
                    if(clawWristPosition==76.5){
                        if (clawPitchPosition == 195){
                            clawPitchPosition = 104;
                        }
                        else if (clawPitchPosition == 104 || clawPitchPosition == 67.25) {
                            clawPitchPosition = 30.5;
                        }
                    }

                }
                isPressingBumper2=true;
            }
            else if (gamepad2.dpad_right){
                if (!isPressingBumper2) {
                    if (clawWristPosition==76.5){
                        if (clawPitchPosition == 30.5 || clawPitchPosition == 67.25) {
                            clawPitchPosition = 104;
                        }
                        else if (clawPitchPosition == 104) {
                            clawPitchPosition = 195;
                        }
                    }
                }
                isPressingBumper2=true;
            }
            else{
                isPressingBumper2=false;
            }

            //specimen scoring pitch position
            if ((gamepad2.options) && (clawWristPosition==76.5)){
                clawPitchPosition = 67.25;
            }

            // Claw finger close 0 degrees
            // Claw finger open 50 degrees
            // Claw fingers toggle between open and closed
            // Claw fingers fully open 80 degrees
            if (gamepad2.left_bumper){
                if (!isPressingTrigger1) {
                    if (clawFingerPosition == 80) {
                        clawFingerPosition = 50;
                    }
                    else if (clawFingerPosition == 50){
                        clawFingerPosition = 2;
                    }
                }
                isPressingTrigger1=true;
            }
            else if (gamepad2.right_bumper){
                if (!isPressingTrigger1) {
                    if (clawFingerPosition == 2) {
                        clawFingerPosition = 50;
                    }
                    else if (clawFingerPosition == 50) {
                        clawFingerPosition = 80;
                    }
                }
                isPressingTrigger1=true;
            }
            else{
                isPressingTrigger1=false;
            }

            // Default perpendicular claw pos 76.5 degrees
            if (gamepad2.left_trigger>0) {
                clawWristPosition = 76.5;
            }
            else if (gamepad2.right_trigger>0 && clawWristPosition <= 143) {
                clawWristPosition += 10;
            }

            // BucketTransfer / default pos 85 degrees
            // Bucket Deposit pos 205 degrees
            // When bucket slides are going up the bucket will move when the slides are 100 ticks away from max position
            /*
            if (gamepad2.dpad_down){
                bucketPosition = 85;
            }
            if (gamepad2.dpad_up){
                bucketPosition = 205;
            }
            */
            if (gamepad2.a){
                if (!isPressingA2){
                    isPressingA2=true;
                    if (bucketPosition==85) {bucketPosition=205;} else {bucketPosition=85;}
                }
            }
            else isPressingA2=false;

            if (gamepad1.options) {
                imu.resetYaw();
                pinpoint.resetPosAndIMU();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //double botHeading = pinpoint.getYawScalar();
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

            extendo.setPower((extendoTarget - extendo.getCurrentPosition()) * kP);
            extendoPitch.setPower((extendoPitchTarget - extendoPitch.getCurrentPosition()) * kPpitch);
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            hang.setPower((hangTarget - hang.getCurrentPosition()) * kP);
            bucketSlides.setPower((bucketSlidesTarget - bucketSlides.getCurrentPosition()) * kP);

            clawPitchLeft.setPosition(clawPitchPosition/270);
            clawPitchRight.setPosition(clawPitchPosition/270);
            clawFingers.setPosition(clawFingerPosition/180);
            clawWrist.setPosition(clawWristPosition/180);
            bucket.setPosition(bucketPosition/270);

            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendo target", extendoTarget);
            telemetry.addData("extendo pitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("extendo pitch target", extendoPitchTarget);
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.addData("hang target", hangTarget);
            telemetry.addData("bucketSlides pos", bucketSlides.getCurrentPosition());
            telemetry.addData("bucketSlides target", bucketSlidesTarget);

            telemetry.addData("left claw pitch position", clawPitchLeft.getPosition());
            telemetry.addData("right claw pitch position", clawPitchRight.getPosition());
            telemetry.addData("claw finger position", clawFingers.getPosition());
            telemetry.addData("claw finger position in degree", clawFingerPosition);
            telemetry.addData("claw wrist position", clawWrist.getPosition());
            telemetry.addData("bucket pos", bucket.getPosition());
            telemetry.addData("bucket target", bucketPosition);

            telemetry.addData("bot heading", botHeading);
            telemetry.addData("pinpoint heading", pinpoint.getYawScalar());
            telemetry.addData("Control hub IMU heading", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("pinpoint x",pinpoint.getPosX());
            telemetry.addData("pinpoint y", pinpoint.getPosY());

            telemetry.update();
        }
    }
}
