package org.firstinspires.ftc.teamcode.base.teleop;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.lang.Math;
import java.lang.Double;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class UpgradedMecanumTeleop extends LinearOpMode {

    double hangDistance = 0;
    double extendoPitchDistance = 0;
    double extendoDistance = 0;
    double bucketSlidesDistance = 0;
    double hangTarget = 0;
    double extendoPitchTarget = 0;
    double extendoTarget = 0;
    double bucketSlidesTarget = 0;
    double oldHangPos = 0;
    double oldExtendoPitchPos = 0;
    double oldExtendoPos = 0;
    double oldBucketSlidesPos = 0;
    double clawWristPosition = 79.5;
    double clawFingerPosition = 120;
    double clawPitchPosition = 205;
    double bucketPosition = 85;
    DcMotor extendo;
    DcMotor extendoPitch;
    DcMotor hang;
    DcMotor bucketSlides;

    ElapsedTime hangTimer = new ElapsedTime();
    ElapsedTime extendoTimer = new ElapsedTime();
    ElapsedTime extendoPitchTimer = new ElapsedTime();
    ElapsedTime bucketSlidesTimer = new ElapsedTime();
    ElapsedTime PIDTimer = new ElapsedTime();
    HashMap<String,Double> integralSums;

    double extendokP = 0.015;
    double extendoPitchkP = 0.005;
    double bucketSlideskP = 0.015;
    double hangkP = 0.015;
    double extendokI = 0.001;
    double extendoPitchkI = 0.001;
    double bucketSlideskI = 0.001;
    double hangkI = 0.001;
    double extendokD = 0.001;
    double extendoPitchkD = 0.001;
    double bucketSlideskD = 0.001;
    double hangkD = 0.001;
    double extendoPrevPos = 0.001;
    double extendoPitchPrevPos = 0.001;
    double bucketSlidesPrevPos = 0.001;
    double hangPrevPos = 0.001;



    public void setHangTarget(double target){
        oldHangPos=hang.getCurrentPosition();
        hangTarget=target;
        hangDistance=hangTarget-oldHangPos;
        hangTimer.reset();
        integralSums.put("hang",0.0);
    }
    public void setBucketSlidesTarget(double target){
        oldBucketSlidesPos=hang.getCurrentPosition();
        bucketSlidesTarget=target;
        bucketSlidesDistance=bucketSlidesTarget-oldBucketSlidesPos;
        bucketSlidesTimer.reset();
        integralSums.put("bucketSlides",0.0);
    }
    public void setExtendoTarget(double target){
        oldExtendoPos=hang.getCurrentPosition();
        extendoTarget=target;
        extendoDistance=extendoTarget-oldExtendoPos;
        extendoTimer.reset();
        integralSums.put("extendo",0.0);
    }
    public void setExtendoPitchTarget(double target){
        oldExtendoPitchPos=hang.getCurrentPosition();
        extendoPitchTarget=target;
        extendoPitchDistance=extendoPitchTarget-oldExtendoPitchPos;
        extendoPitchTimer.reset();
        integralSums.put("extendoPitch",0.0);
    }

    public double motionProfile(double max_acceleration, double max_velocity, double distance, String motor) {
        HashMap map=new HashMap<String,ElapsedTime>(4);
        map.put("hang",hangTimer);
        map.put("bucketSlides",bucketSlidesTimer);
        map.put("extendo",extendoTimer);
        map.put("extendoPitch",extendoPitchTimer);
        ElapsedTime timer= (ElapsedTime)map.get(motor);
        double elapsed_time=timer.seconds();
        HashMap map2=new HashMap<String,Double>(4);
        map.put("hang",hangDistance);
        map.put("bucketSlides",bucketSlidesDistance);
        map.put("extendo",extendoDistance);
        map.put("extendoPitch",extendoPitchDistance);
        distance= (double)map.get(motor);
        /*
        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        */
        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            timer.reset();
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time,2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time,2);
        }
    }
    public double calculatePID(double kP, double kI, double kD, double elapsedTime, double currentpos, double previouspos, double target, String motor){
        double error=target-currentpos;
        double previouserror=target-previouspos;
        integralSums.put(motor,integralSums.get(motor)+error*elapsedTime);
        return Math.min(1,Math.max(-1,kP*error+kI*integralSums.get(motor)+kD*(error-previouserror)/elapsedTime));
    }
    @Override
    public void runOpMode() throws InterruptedException {
        integralSums.put("hang",0.0); integralSums.put("extendo",0.0); integralSums.put("extendoPitch",0.0); integralSums.put("bucketSlides",0.0);
        double maxExtendoPosition = 350;
        boolean isXSequenceActive = false;
        boolean isASequenceActive = false;
        boolean isBSequenceActive = false;
        boolean isB2SequenceActive = false;
        boolean isOp2SequenceActive = false;
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
        ElapsedTime Op2timer = new ElapsedTime();

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
            if (gamepad1.back && bucketSlidesTarget>=15){
                setBucketSlidesTarget(bucketSlidesTarget-15);
            }
            else if (gamepad1.back && bucketSlidesTarget<15){
                setBucketSlidesTarget(0);
            }
            if (gamepad2.back){
                bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            pinpoint.update();

            // bucket slides retraction sequence
            if (gamepad1.x){
                isXSequenceActive=true;
                Xtimer.reset();
            }
            if (isXSequenceActive) {
                bucketPosition=85;
                if (Xtimer.seconds()>0.3){
                    setBucketSlidesTarget(0);
                    isXSequenceActive=false;
                }
            }
            // Intake sequence picking up sample in submersible (claw pitch needs to fit over sub)
            if (gamepad1.b){
                isBSequenceActive=true;
                Btimer.reset();

            }
            if (isBSequenceActive) {
                setExtendoPitchTarget(1425);
                clawWristPosition = 79.5;
                if (Btimer.seconds() > 0.7) {
                    setExtendoTarget(maxExtendoPosition);
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

                clawWristPosition = 79.5;
                bucketPosition=85;
                if (Atimer.seconds() > 0.5){
                    clawPitchPosition = 104;

                }
                if (Atimer.seconds() > 0.5){
                    setExtendoTarget(0);
                }

                if (Atimer.seconds() > 0.8) {
                    if (!(extendo.getCurrentPosition()>150)) {
                        setExtendoPitchTarget(0);
                    }
                }

                if (extendoPitch.getCurrentPosition()<100) {
                    clawPitchPosition = 205;
                    clawWristPosition = 90;
                    isASequenceActive=false;
                }

            }


            //specimen setup sequence (including retraction of extendo after specimen pickup)
            else if (gamepad2.b){
                isB2SequenceActive=true;
                B2timer.reset();
            }
            if (isB2SequenceActive) {
                clawWristPosition =79.5;
                clawPitchPosition=205;

                if (B2timer.seconds()>0.3){
                    clawPitchPosition = 104;
                    bucketPosition=205;
                    setExtendoTarget(0);
                }

                if (B2timer.seconds() > 0.6) {
                    setExtendoPitchTarget(450);
                }
                if (B2timer.seconds() > 1) {
                    setExtendoTarget(500);
                    isB2SequenceActive=false;
                }
            }
            //wall specimen pickup sequence

            if (gamepad2.options){
                isOp2SequenceActive=true;
                Op2timer.reset();
            }
            if (isOp2SequenceActive==true){
                clawWristPosition = 79.5;

                if (B2timer.seconds()>0.3){
                    clawPitchPosition = 84;
                    bucketPosition=205;
                    setExtendoTarget(0);
                }

                if (B2timer.seconds() > 0.6) {
                    setExtendoPitchTarget(1100);
                }
                if (B2timer.seconds() > 1) {
                    setExtendoTarget(maxExtendoPosition);
                    clawFingerPosition=120;
                    isOp2SequenceActive=false;
                }
            }

            // Extendo retracted 0 ticks
            // Extendo fully extending 36050
            // Dynamic extendo control
            /*
            if ((gamepad1.right_bumper) && (extendoTarget <= maxExtendoPosition-30)){
                extendoTarget += 30;
            }
            else if ((gamepad1.right_bumper) && (extendoTarget >= maxExtendoPosition-30)){
                extendoTarget = maxExtendoPosition;
            }
            else if ((gamepad1.left_bumper) && (extendoTarget>=30)){
                extendoTarget -= 30;
            }
            else if ((gamepad1.left_bumper) && (extendoTarget <= 30)){
                extendoTarget = 0;
            }
            */

            // Extendo pitch transfer / default pos 0 ticks
            // Extendo pitch pickup 1425
            if (gamepad1.dpad_down && !(extendo.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (extendoPitchTarget >= 0 && extendoPitchTarget<450){
                        setExtendoPitchTarget(450);
                    }
                    else if (extendoPitchTarget >= 450 && extendoPitchTarget<1421){
                        setExtendoPitchTarget(1421);
                    }
                }
                isPressingDpad=true;
            }
            else if (gamepad1.dpad_up && !(extendo.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (extendoPitchTarget <= 1421 && extendoPitchTarget > 450) {
                        setExtendoPitchTarget(450);
                    }
                    else if (extendoPitchTarget <= 450 && extendoPitchTarget>0) {
                        setExtendoPitchTarget(0);
                    }
                }
                isPressingDpad=true;
            }
            else{
                isPressingDpad=false;
            }

            //dynamic extendo pitch movement - for specimen scoring
            if (-gamepad2.right_stick_y>0&&extendoPitchTarget>=50&&!(extendo.getCurrentPosition()>150)){
                setExtendoPitchTarget(extendoPitchTarget-20);
            }
            else if (-gamepad2.right_stick_y<0&&extendoPitchTarget<=1300&&!(extendo.getCurrentPosition()>150)){
                setExtendoPitchTarget(extendoPitchTarget+20);
            }
            if (gamepad2.x&&extendoPitchTarget>=215){
                if (!isPressingX2) {
                    setExtendoPitchTarget(760);
                    isPressingX2=true;
                }
            }
            else isPressingX2=false;

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (hangTarget == 0) {
                        isPressingY2 = true;
                        setHangTarget(9000);}
                    else if (hangTarget == 9000) {
                        setHangTarget(5400);
                        isPressingY2 = true;
                    }
                    else if (hangTarget == 5400){
                        setHangTarget(0);
                        isPressingY2 = true;
                    }
                }
            }
            else isPressingY2=false;

            // Bucket Slides toggle between min and max positions
            if (gamepad1.y){
                if (!isPressingY) {
                    if (bucketSlidesTarget == 0) {
                        setBucketSlidesTarget(1100);}
                    else {
                        setBucketSlidesTarget(0);
                    }
                    isPressingY=true;
                }
            }
            else isPressingY=false;

            // Claw pitch picking up pos 30.5 degrees
            // Claw transfer pos 215 degrees
            // Claw pitch going into sub 104 degrees
            // Claw pitch to position 0 to 1
            if (gamepad2.dpad_left){
                if (!isPressingBumper2) {
                    if(clawWristPosition==79.5){
                        if (clawPitchPosition > 104){
                            clawPitchPosition = 104;
                        }
                        else if (clawPitchPosition <= 104) {
                            clawPitchPosition = 30.5;
                        }
                    }

                }
                isPressingBumper2=true;
            }
            else if (gamepad2.dpad_right){
                if (!isPressingBumper2) {
                    if (clawWristPosition==79.5){
                        if (clawPitchPosition < 104) {
                            clawPitchPosition = 104;
                        }
                        else if (clawPitchPosition >= 104) {
                            clawPitchPosition = 205;
                        }
                    }
                }
                isPressingBumper2=true;
            }
            else{
                isPressingBumper2=false;
            }


            // Claw finger close 0 degrees
            // Claw finger open 50 degrees
            // Claw fingers toggle between open and closed
            // Claw fingers fully open 80 degrees
            if (gamepad2.left_bumper){
                if (!isPressingTrigger1) {
                    if (clawFingerPosition == 120) {
                        clawFingerPosition = 90;
                    }
                    else if (clawFingerPosition == 90){
                        clawFingerPosition = 37;
                    }
                }
                isPressingTrigger1=true;
            }
            else if (gamepad2.right_bumper){
                if (!isPressingTrigger1) {
                    if (clawFingerPosition == 37) {
                        clawFingerPosition = 90;
                    }
                    else if (clawFingerPosition == 90) {
                        clawFingerPosition = 120;
                    }
                }
                isPressingTrigger1=true;
            }
            else{
                isPressingTrigger1=false;
            }

            // Default perpendicular claw pos 79.5 degrees
            if (gamepad2.left_trigger>0) {
                clawWristPosition = 79.5;
            }
            else if (gamepad2.right_trigger>0 && clawWristPosition <= 143) {
                clawWristPosition += 10;
            }

            // BucketTransfer / default pos 85 degrees
            // Bucket Deposit pos 205 degrees
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
            double extendoMP=motionProfile(2781*Math.signum(extendoDistance),2781*Math.signum(extendoDistance),extendoDistance,"extendo")+oldExtendoPos;
            double extendoPitchMP=motionProfile(2787*Math.signum(extendoPitchDistance),2787*Math.signum(extendoPitchDistance),extendoPitchDistance,"extendoPitch")+oldExtendoPitchPos;
            double bucketSlidesMP=motionProfile(2781*Math.signum(bucketSlidesDistance),2781*Math.signum(bucketSlidesDistance),bucketSlidesDistance,"bucketSlides")+oldBucketSlidesPos;
            double hangMP=motionProfile(2787*Math.signum(hangDistance),2787*Math.signum(hangDistance),hangDistance,"hang")+oldHangPos;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            extendo.setPower(calculatePID(extendokP, extendokI, extendokD, PIDTimer.time(), extendo.getCurrentPosition(), extendoPrevPos,extendoMP, "extendo"));
            extendoPitch.setPower(calculatePID(extendoPitchkP, extendoPitchkI, extendoPitchkD, PIDTimer.time(), extendoPitch.getCurrentPosition(), extendoPitchPrevPos,extendoPitchMP, "extendoPitch"));
            hang.setPower(calculatePID(hangkP, hangkI, hangkD, PIDTimer.time(), hang.getCurrentPosition(), hangPrevPos,hangMP, "hang"));
            bucketSlides.setPower(calculatePID(bucketSlideskP, bucketSlideskI, bucketSlideskD, PIDTimer.time(), bucketSlides.getCurrentPosition(), bucketSlidesPrevPos,bucketSlidesMP, "bucketSlides"));
            PIDTimer.reset();

            extendoPrevPos=extendo.getCurrentPosition();
            extendoPitchPrevPos=extendo.getCurrentPosition();
            bucketSlidesPrevPos=extendo.getCurrentPosition();
            hangPrevPos=extendo.getCurrentPosition();


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
            telemetry.addData("pinpoint heading", pinpoint.getHeading());
            telemetry.addData("Control hub IMU heading", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("pinpoint x",pinpoint.getPosX());
            telemetry.addData("pinpoint y", pinpoint.getPosY());

            telemetry.update();
        }
    }
}
