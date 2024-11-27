package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.autonomous.RobotState;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class InProgressTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareConfig hw = new HardwareConfig(hardwareMap);
        RobotState state = new RobotState();
        
        boolean isXSequenceActive = false;
        boolean isASequenceActive = false;
        boolean isBSequenceActive = false;
        boolean isB2SequenceActive = false;
        boolean isPressingY = false;
        boolean isPressingY2 = false;
        boolean isPressingA2 = false;
        boolean isPressingX2 = false;
        boolean isPressingBumper2 = false;
        boolean isPressingTrigger1 = false;
        boolean isPressingDpad = false;

        ElapsedTime Xtimer = new ElapsedTime();
        ElapsedTime Btimer = new ElapsedTime();
        ElapsedTime B2timer = new ElapsedTime();
        ElapsedTime Atimer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad2.back){
                hw.bucketSlides.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // bucket slides retraction sequence
            if (gamepad1.x){
                isXSequenceActive=true;
                Xtimer.reset();
            }
            if (isXSequenceActive) {
                state.bucketPosition=85;

                if (Xtimer.seconds()>0.3){
                    state.bucketSlidesTarget=0;
                    isXSequenceActive=false;
                }
            }
            // Intake sequence picking up sample in submersible (claw pitch needs to fit over sub)
            if (gamepad1.b){
                isBSequenceActive=true;
                Btimer.reset();

            }
            if (isBSequenceActive) {
                state.extendoPitchTarget = 1350;
                state.clawWristPosition = 76.5;
                if (Btimer.seconds() > 0.7) {
                    state.extendoTarget = 503;
                    state.clawPitchPosition = 104;
                    isBSequenceActive=false;
                }
            }
            // Transfer sample
            else if (gamepad1.a){
                isASequenceActive=true;
                Atimer.reset();

            }
            if (isASequenceActive) {
                if (state.clawWristPosition != 76.5){
                    state.clawWristPosition = 76.5;
                    state.bucketPosition=85;
                    if (Atimer.seconds() > 0.3){
                        state.clawPitchPosition = 104;

                    }
                    if (Atimer.seconds() > 0.5){
                        state.extendoTarget=0;
                    }

                    if (Atimer.seconds() > 0.8) {
                        if (!(hw.extendo.motor.getCurrentPosition()>150)) {
                            state.extendoPitchTarget = 0;
                        }
                        state.clawPitchPosition = 195;
                        isASequenceActive=false;
                    }
                }
                else{
                    state.bucketPosition=85;
                    state.clawPitchPosition = 104;


                    if (Atimer.seconds() > 0.2){
                        state.extendoTarget=0;
                    }

                    if (Atimer.seconds() > 0.5) {
                        if (!(hw.extendo.motor.getCurrentPosition()>150)) {
                            state.extendoPitchTarget = 0;
                        }
                        state.clawPitchPosition = 195;
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
                state.clawWristPosition =76.5;

                if (B2timer.seconds()>0.3){
                    state.clawPitchPosition = 104;
                    state.bucketPosition=205;
                    state.extendoTarget=0;
                }

                if (B2timer.seconds() > 0.6) {
                    state.extendoPitchTarget = 450;
                }
                if (B2timer.seconds() > 1) {
                    state.extendoTarget=500;
                    isB2SequenceActive=false;
                }
            }
            // Extendo retracted 0 ticks
            // Extendo fully extending 36050
            // Dynamic extendo control

            if ((gamepad1.right_bumper) && (state.extendoTarget <= 473)){
                state.extendoTarget += 30;
            }
            else if ((gamepad1.left_bumper) && (state.extendoTarget >= 30)){
                state.extendoTarget -= 30;
            }

            // Extendo pitch transfer / default pos 0 ticks
            // Extendo pitch pickup 1350
            if (gamepad1.dpad_down && !(hw.extendo.motor.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (state.extendoPitchTarget == 0) {
                        state.extendoPitchTarget = 450;
                    }
                    else if (state.extendoPitchTarget == 450 || state.extendoPitchTarget == 760){
                        state.extendoPitchTarget = 1421;
                    }
                }
                isPressingDpad=true;
            }
            else if (gamepad1.dpad_up && !(hw.extendo.motor.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (state.extendoPitchTarget == 1421 || state.extendoPitchTarget == 760) {
                        state.extendoPitchTarget = 450;
                    }
                    else if (state.extendoPitchTarget == 450) {
                        state.extendoPitchTarget = 0;
                    }
                }
                isPressingDpad=true;
            }
            else{
                isPressingDpad=false;
            }

            /*
            //dynamic extendo movement - for specimen scoring
            if (-gamepad2.right_stick_y>0&&state.extendoPitchTarget>=50){
                state.extendoPitchTarget-=50;
            }
            else if (-gamepad2.right_stick_y<0&&state.extendoPitchTarget<=1300){
                state.extendoPitchTarget+=50;
            }
            */
            if (gamepad2.x&&state.extendoPitchTarget>=210){
                if (!isPressingX2) {
                    state.extendoPitchTarget=760;
                    isPressingX2=true;
                }
            }
            else isPressingX2=false;

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (state.hangTarget == 0) {
                        isPressingY2 = true;
                        state.hangTarget = 9000;}
                    else if (state.hangTarget == 9000) {
                        state.hangTarget = 5400;
                        isPressingY2 = true;
                    }
                    else if (state.hangTarget == 5400){
                        state.hangTarget = 0;
                        isPressingY2 = true;
                    }
                }
            }
            else isPressingY2=false;

            // Bucket Slides toggle between min and max positions
            if (gamepad1.y){
                if (!isPressingY) {
                    if (state.bucketSlidesTarget == 0) {
                        state.bucketSlidesTarget = 1100;}
                    else {
                        state.bucketSlidesTarget = 0;
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
                    if(state.clawWristPosition==76.5){
                        if (state.clawPitchPosition == 195){
                            state.clawPitchPosition = 104;
                        }
                        else if (state.clawPitchPosition == 104 || state.clawPitchPosition == 67.25) {
                            state.clawPitchPosition = 30.5;
                        }
                    }

                }
                isPressingBumper2=true;
            }
            else if (gamepad2.dpad_right){
                if (!isPressingBumper2) {
                    if (state.clawWristPosition==76.5){
                        if (state.clawPitchPosition == 30.5 || state.clawPitchPosition == 67.25) {
                            state.clawPitchPosition = 104;
                        }
                        else if (state.clawPitchPosition == 104) {
                            state.clawPitchPosition = 195;
                        }
                    }
                }
                isPressingBumper2=true;
            }
            else{
                isPressingBumper2=false;
            }

            //specimen scoring pitch position
            if ((gamepad2.options) && (state.clawWristPosition==76.5)){
                state.clawPitchPosition = 67.25;
            }

            // Claw finger close 0 degrees
            // Claw finger open 50 degrees
            // Claw fingers toggle between open and closed
            // Claw fingers fully open 80 degrees
            if (gamepad2.left_bumper){
                if (!isPressingTrigger1) {
                    if (state.clawFingerPosition == 80) {
                        state.clawFingerPosition = 50;
                    }
                    else if (state.clawFingerPosition == 50){
                        state.clawFingerPosition = 2;
                    }
                }
                isPressingTrigger1=true;
            }
            else if (gamepad2.right_bumper){
                if (!isPressingTrigger1) {
                    if (state.clawFingerPosition == 2) {
                        state.clawFingerPosition = 50;
                    }
                    else if (state.clawFingerPosition == 50) {
                        state.clawFingerPosition = 80;
                    }
                }
                isPressingTrigger1=true;
            }
            else{
                isPressingTrigger1=false;
            }

            // Default perpendicular claw pos 76.5 degrees
            if (gamepad2.left_trigger>0) {
                state.clawWristPosition = 76.5;
            }
            else if (gamepad2.right_trigger>0 && state.clawWristPosition <= 143) {
                state.clawWristPosition += 10;
            }

            // BucketTransfer / default pos 85 degrees
            // Bucket Deposit pos 205 degrees
            // When bucket slides are going up the bucket will move when the slides are 100 ticks away from max position
            /*
            if (gamepad2.dpad_down){
                state.bucketPosition = 85;
            }
            if (gamepad2.dpad_up){
                state.bucketPosition = 205;
            }
            */
            if (gamepad2.a){
                if (!isPressingA2){
                    isPressingA2=true;
                    if (state.bucketPosition==85) {state.bucketPosition=205;} else {state.bucketPosition=85;}
                }
            }
            else isPressingA2=false;

            if (gamepad1.options) {
                hw.imu.imu.resetYaw();
                hw.pinpoint.pinpoint.resetPosAndIMU();
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

            hw.extendo.motor.setPower((state.extendoTarget - hw.extendo.motor.getCurrentPosition()) * hw.extendo.kP);
            hw.extendoPitch.motor.setPower((state.extendoPitchTarget - hw.extendoPitch.motor.getCurrentPosition()) * hw.extendo.kP);
            hw.frontLeftMotor.motor.setPower(frontLeftPower);
            hw.backLeftMotor.motor.setPower(backLeftPower);
            hw.frontRightMotor.motor.setPower(frontRightPower);
            hw.backRightMotor.motor.setPower(backRightPower);
            hw.hang.motor.setPower((state.hangTarget - hw.hang.motor.getCurrentPosition()) * hw.extendo.kP);
            hw.bucketSlides.motor.setPower((state.bucketSlidesTarget - hw.bucketSlides.motor.getCurrentPosition()) * hw.extendo.kP);

            hw.clawPitchLeft.servo.setPosition(state.clawPitchPosition/270);
            hw.clawPitchRight.servo.setPosition(state.clawPitchPosition/270);
            hw.clawFingers.servo.setPosition(state.clawFingerPosition/180);
            hw.clawWrist.servo.setPosition(state.clawWristPosition/180);
            hw.bucket.servo.setPosition(state.bucketPosition/270);

            telemetry.addData("extendo position", hw.extendo.motor.getCurrentPosition());
            telemetry.addData("extendo target", state.extendoTarget);
            telemetry.addData("extendo pitch position", hw.extendoPitch.motor.getCurrentPosition());
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

            telemetry.addData("bot heading", botHeading);
            telemetry.addData("pinpoint heading", hw.pinpoint.pinpoint.getYawScalar());
            telemetry.addData("Control hub IMU heading", hw.imu.imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("pinpoint x",hw.pinpoint.pinpoint.getPosX());
            telemetry.addData("pinpoint y", hw.pinpoint.pinpoint.getPosY());

            telemetry.update();
        }
    }
}
