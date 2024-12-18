package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.ServoEnum;

@TeleOp
public class OptimizedTeleOp extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
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

    int maxExtendoPosition = 350;

    ElapsedTime xTimer = new ElapsedTime();
    ElapsedTime bTimer = new ElapsedTime();
    ElapsedTime b2Timer = new ElapsedTime();
    ElapsedTime aTimer = new ElapsedTime();
    ElapsedTime Op2timer = new ElapsedTime();
    @Override
    public void runOpMode(){
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();

        waitForStart();

        while (opModeIsActive()){

            // Drivetrain control

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double botHeading = 0;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double leftFrontPower = (rotY + rotX + rx) / denominator;
            double leftBackPower = (rotY - rotX + rx) / denominator;
            double rightFrontPower = (rotY - rotX - rx) / denominator;
            double rightBackPower = (rotY + rotX - rx) / denominator;
            // Slow mode, 50% speed
            if (gamepad1.left_trigger>0.5) {
                leftFrontPower = 0.5 * (rotY + rotX + rx) / denominator;
                leftBackPower = 0.5 * (rotY - rotX + rx) / denominator;
                rightFrontPower = 0.5 * (rotY - rotX - rx) / denominator;
                rightBackPower = 0.5 * (rotY + rotX - rx) / denominator;
            }

            hw.getMotorConfig(MotorEnum.LEFT_FRONT).motor.setPower(leftFrontPower);
            hw.getMotorConfig(MotorEnum.LEFT_BACK).motor.setPower(leftBackPower);
            hw.getMotorConfig(MotorEnum.RIGHT_FRONT).motor.setPower(rightFrontPower);
            hw.getMotorConfig(MotorEnum.RIGHT_BACK).motor.setPower(rightBackPower);

            if (gamepad1.back && state.getMotorTarget(MotorEnum.BUCKET_SLIDES)>=15){
                state.setMotorTarget(MotorEnum.BUCKET_SLIDES, state.getMotorTarget(MotorEnum.BUCKET_SLIDES) - 15);
            }
            else if (gamepad1.back && state.getMotorTarget(MotorEnum.BUCKET_SLIDES)<15){
                state.setMotorTarget(MotorEnum.BUCKET_SLIDES, 0);
            }
            if (gamepad2.back){
                hw.getMotorConfig(MotorEnum.BUCKET_SLIDES).motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.getMotorConfig(MotorEnum.EXTENDO).motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            hw.getPinpointConfig().pinpoint.update();

            // bucket slides retraction sequence
            if (gamepad1.x){
                isXSequenceActive=true;
                xTimer.reset();
            }
            if (isXSequenceActive) {
                state.setServoPosition(ServoEnum.BUCKET, 85);

                if (xTimer.seconds()>0.3){
                    state.setMotorTarget(MotorEnum.BUCKET_SLIDES, 0);
                    isXSequenceActive=false;
                }
            }
            // Intake sequence picking up sample in submersible (claw pitch needs to fit over sub)
            if (gamepad1.b){
                isBSequenceActive=true;
                bTimer.reset();

            }
            if (isBSequenceActive) {
                state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 1425);
                state.setServoPosition(ServoEnum.CLAW_WRIST, 79.5);
                if (bTimer.seconds() > 0.7) {
                    state.setMotorTarget(MotorEnum.EXTENDO, maxExtendoPosition);
                    state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,104);
                    isBSequenceActive=false;
                }
            }
            // Transfer sample
            else if (gamepad1.a){
                isASequenceActive=true;
                aTimer.reset();

            }
            if (isASequenceActive) {

                state.setServoPosition(ServoEnum.CLAW_WRIST, 79.5);
                state.setServoPosition(ServoEnum.BUCKET, 85);
                if (aTimer.seconds() > 0.5){
                    state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,104);

                }
                if (aTimer.seconds() > 0.5){
                    state.setMotorTarget(MotorEnum.EXTENDO,0);
                }

                if (aTimer.seconds() > 0.8) {
                    if (!(hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition()>150)) {
                        state.setMotorTarget(MotorEnum.EXTENDO_PITCH,0);
                    }
                }

                if (hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.getCurrentPosition()<100) {
                    state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,205);
                    state.setServoPosition(ServoEnum.CLAW_WRIST,90);
                    isASequenceActive=false;
                }

            }


            //specimen setup sequence (including retraction of extendo after specimen pickup)
            else if (gamepad2.b){
                isB2SequenceActive=true;
                b2Timer.reset();
            }
            if (isB2SequenceActive) {
                state.setServoPosition(ServoEnum.CLAW_WRIST, 79.5);
                state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,205);

                if (b2Timer.seconds()>0.3){
                    state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT,104);
                    state.setServoPosition(ServoEnum.BUCKET, 205);
                    state.setMotorTarget(MotorEnum.EXTENDO,0);
                }

                if (b2Timer.seconds() > 0.6) {
                    state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 450);
                }
                if (b2Timer.seconds() > 1) {
                    state.setMotorTarget(MotorEnum.EXTENDO,500);
                    isB2SequenceActive=false;
                }
            }
            //wall specimen pickup sequence
            /*
            if (gamepad2.options){
                isOp2SequenceActive=true;
                Op2timer.reset();
            }
            if (isOp2SequenceActive==true){
                clawWristPosition = 79.5;

                if (b2Timer.seconds()>0.3){
                    clawPitchPosition = 84;
                    bucketPosition=205;
                    extendoTarget=0;
                }

                if (b2Timer.seconds() > 0.6) {
                    extendoPitchTarget = 1100;
                }
                if (b2Timer.seconds() > 1) {
                    extendoTarget=maxExtendoPosition;
                    clawFingerPosition=120;
                    isOp2SequenceActive=false;
                }
            }
            */

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
            if (gamepad1.dpad_down && !(hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (state.getMotorTarget(MotorEnum.EXTENDO_PITCH) >= 0 && state.getMotorTarget(MotorEnum.EXTENDO_PITCH)<450){
                        state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 450);
                    }
                    else if (state.getMotorTarget(MotorEnum.EXTENDO_PITCH) >= 450 && state.getMotorTarget(MotorEnum.EXTENDO_PITCH)<1421){
                        state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 1421);
                    }
                }
                isPressingDpad=true;
            }
            else if (gamepad1.dpad_up && !(hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition()>150)){
                if (!isPressingDpad) {
                    if (state.getMotorTarget(MotorEnum.EXTENDO_PITCH) <= 1421 && state.getMotorTarget(MotorEnum.EXTENDO_PITCH) > 450) {
                        state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 450);
                    }
                    else if (state.getMotorTarget(MotorEnum.EXTENDO_PITCH) <= 450 && state.getMotorTarget(MotorEnum.EXTENDO_PITCH)>0) {
                        state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 0);
                    }
                }
                isPressingDpad=true;
            }
            else{
                isPressingDpad=false;
            }

            //dynamic extendo pitch movement - for specimen scoring
            if (-gamepad2.right_stick_y > 0 && state.getMotorTarget(MotorEnum.EXTENDO_PITCH) >= 50 &&!(hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition() > 150)){
                state.setMotorTarget(MotorEnum.EXTENDO_PITCH, hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.getCurrentPosition() - 20);
            }
            else if (-gamepad2.right_stick_y<0&&state.getMotorTarget(MotorEnum.EXTENDO_PITCH)<=1300&&!(hw.getMotorConfig(MotorEnum.EXTENDO).motor.getCurrentPosition()>150)){
                state.setMotorTarget(MotorEnum.EXTENDO_PITCH, hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.getCurrentPosition() + 20);
            }
            if (gamepad2.x&&state.getMotorTarget(MotorEnum.EXTENDO_PITCH)>=215){
                if (!isPressingX2) {
                    state.setMotorTarget(MotorEnum.EXTENDO_PITCH, 760);
                    isPressingX2=true;
                }
            }
            else isPressingX2=false;

            // Hang toggle between min and max positions
            if (gamepad2.y){
                if (!isPressingY2) {
                    if (state.getMotorTarget(MotorEnum.HANG) == 0) {
                        isPressingY2 = true;
                        state.setMotorTarget(MotorEnum.HANG,9000);}
                    else if (state.getMotorTarget(MotorEnum.HANG) == 9000) {
                        state.setMotorTarget(MotorEnum.HANG,5400);
                        isPressingY2 = true;
                    }
                    else if (state.getMotorTarget(MotorEnum.HANG) == 5400){
                        state.setMotorTarget(MotorEnum.HANG,0);
                        isPressingY2 = true;
                    }
                }
            }
            else isPressingY2=false;

            // Bucket Slides toggle between min and max positions
            if (gamepad1.y){
                if (!isPressingY) {
                    if (state.getMotorTarget(MotorEnum.BUCKET_SLIDES) == 0) {
                        state.setMotorTarget(MotorEnum.BUCKET_SLIDES, 1100);}
                    else {
                        state.setMotorTarget(MotorEnum.BUCKET_SLIDES, 0);
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
                    if(state.getServoPosition(ServoEnum.CLAW_WRIST) == 79.5){
                        if (state.getServoPosition(ServoEnum.CLAW_PITCH_LEFT) > 104 && state.getServoPosition(ServoEnum.CLAW_PITCH_RIGHT) > 104){
                            state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT, 104);
                            state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT, 104);
                        }
                        else if (state.getServoPosition(ServoEnum.CLAW_PITCH_LEFT) <= 104 && state.getServoPosition(ServoEnum.CLAW_PITCH_RIGHT) <= 104) {
                            state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT, 30.5);
                            state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT, 30.5);
                        }
                    }

                }
                isPressingBumper2=true;
            }
            else if (gamepad2.dpad_right){
                if (!isPressingBumper2) {
                    if (state.getServoPosition(ServoEnum.CLAW_WRIST) == 79.5){
                        if (state.getServoPosition(ServoEnum.CLAW_PITCH_LEFT) < 104 && state.getServoPosition(ServoEnum.CLAW_PITCH_RIGHT) < 104){
                            state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT, 104);
                            state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT, 104);
                        }
                        else if (state.getServoPosition(ServoEnum.CLAW_PITCH_LEFT) >= 104 && state.getServoPosition(ServoEnum.CLAW_PITCH_RIGHT) >= 104) {
                            state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT, 205);
                            state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT, 205);
                        }
                    }
                }
                isPressingBumper2=true;
            }
            else{
                isPressingBumper2=false;
            }

            //specimen scoring pitch position
            if ((gamepad2.options) && (state.getServoPosition(ServoEnum.CLAW_WRIST) == 79.5)){
                state.setServoPosition(ServoEnum.CLAW_PITCH_LEFT, 67.5);
                state.setServoPosition(ServoEnum.CLAW_PITCH_RIGHT, 67.5);
            }

            // Claw finger close 0 degrees
            // Claw finger open 50 degrees
            // Claw fingers toggle between open and closed
            // Claw fingers fully open 80 degrees) 
            if (gamepad2.left_bumper){
                if (!isPressingTrigger1) {
                    if (state.getServoPosition(ServoEnum.CLAW_FINGERS) == 120) {
                        state.setServoPosition(ServoEnum.CLAW_FINGERS, 90);
                    }
                    else if (state.getServoPosition(ServoEnum.CLAW_FINGERS) == 90){
                        state.setServoPosition(ServoEnum.CLAW_FINGERS, 37);
                    }
                }
                isPressingTrigger1=true;
            }
            else if (gamepad2.right_bumper){
                if (!isPressingTrigger1) {
                    if (state.getServoPosition(ServoEnum.CLAW_FINGERS) == 37) {
                        state.setServoPosition(ServoEnum.CLAW_FINGERS, 90);
                    }
                    else if (state.getServoPosition(ServoEnum.CLAW_FINGERS) == 90) {
                        state.setServoPosition(ServoEnum.CLAW_FINGERS, 120);
                    }
                }
                isPressingTrigger1=true;
            }
            else{
                isPressingTrigger1=false;
            }

            // Default perpendicular claw pos 79.5 degrees
            if (gamepad2.left_trigger>0) {
                state.setServoPosition(ServoEnum.CLAW_WRIST, 79.5);
            }
            else if (gamepad2.right_trigger>0 && state.getServoPosition(ServoEnum.CLAW_WRIST) <= 143) {
                state.setServoPosition(ServoEnum.CLAW_WRIST, state.getServoPosition(ServoEnum.CLAW_WRIST) + 10);
            }

            // BucketTransfer / default pos 85 degrees
            // Bucket Deposit pos 205 degrees
            if (gamepad2.a){
                if (!isPressingA2){
                    isPressingA2=true;

                    if (state.getServoPosition(ServoEnum.BUCKET)==85) {
                        state.setServoPosition(ServoEnum.BUCKET, 205);
                    }
                    else {
                        state.setServoPosition(ServoEnum.BUCKET, 85);
                    }
                }
            }
            else isPressingA2=false;

            if (gamepad1.options) {
                hw.getImuConfig().imu.resetYaw();
                hw.getPinpointConfig().pinpoint.resetPosAndIMU();
            }

        }
    }
}
