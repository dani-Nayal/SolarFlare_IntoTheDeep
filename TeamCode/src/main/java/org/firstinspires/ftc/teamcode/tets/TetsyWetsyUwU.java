package org.firstinspires.ftc.teamcode.tets;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TetsyWetsyUwU extends LinearOpMode {
    int armtarget=0;
    int buckettarget=0;
    int hangtarget=0;
    int extendotarget=0;
    double pitchtarget=0;
    double rolltarget=135;
    double fingertarget=0;
    double bucketswervotarget=135;
    double armdynamicKp=0.015;
    double extendodynamicKp=0.015;
    double bucketdynamicKp=0.015;
    double hangdynamicKp=0.015;
    boolean isPressingA=false;
    boolean isPressingB=false;
    boolean isPressingY2 = false;
    Servo clawfingers;
    Servo clawroll;
    DcMotorEx extendo;
    DcMotorEx hang;
    DcMotorEx arm;
    DcMotorEx bucket;
    Servo pitch1;
    Servo pitch2;
    Servo bucketswervo;
    int clawrollmax=180;
    int pitchmax=270;
    int bucketpitchmax=270;
    int fingermax=180;
    @Override
    public void runOpMode() throws InterruptedException {
        //clawfingers = hardwareMap.get(Servo.class, "clawFingers");
        //clawroll = hardwareMap.get(Servo.class, "clawWrist");
        //extendo=hardwareMap.get(DcMotorEx.class, "extendo");
        //hang = hardwareMap.get(DcMotorEx.class, "hang");
        //arm = hardwareMap.get(DcMotorEx.class, "extendoPitch");
        //pitch1 = hardwareMap.get(Servo.class, "clawPitchLeft");
        //pitch2 = hardwareMap.get(Servo.class, "clawPitchRight");
        //bucketswervo = hardwareMap.get(Servo.class, "bucket");
        //arm = hardwareMap.get(DcMotorEx.class, "extendoPitch");
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       // bucketswervo = hardwareMap.get(Servo.class, "bucket");
        //bucket=hardwareMap.get(DcMotorEx.class, "bucketSlides");

        waitForStart();
        while (opModeIsActive()){
            if (isStopRequested()) return;
            clawPitchTets();
            armTets();
            telemetry.update();
        }
    }
    public void armTets(){
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


       /*
       arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad1.left_trigger>0){
            armtarget-=1;
        }
        else if (gamepad1.right_trigger>0){
            armtarget+=1;
        }
        arm.setPower(armdynamicKp*(armtarget-arm.getCurrentPosition()));

        */
        telemetry.addData("armpos",arm.getCurrentPosition());
        telemetry.addData("armtarget",armtarget);


    }
    public void bucketTets(){

        bucket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucket.setDirection(DcMotor.Direction.REVERSE);
        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad1.left_bumper){
            buckettarget-=15;
        }
        else if (gamepad1.right_bumper){
            buckettarget+=15;
        }
        bucket.setPower(bucketdynamicKp*(buckettarget-bucket.getCurrentPosition()));
        telemetry.addData("bucketpos",bucket.getCurrentPosition());
        telemetry.addData("buckettarget",buckettarget);
    }
    public void hangTets(){

        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.left_bumper && hangtarget>100){
            hangtarget-=100;
        }
        else if (gamepad2.right_bumper && hangtarget<3500){
            hangtarget+=100;
        }
        hang.setPower(hangdynamicKp*(hangtarget-hang.getCurrentPosition()));
        telemetry.addData("hangpos",hang.getCurrentPosition());
        telemetry.addData("hangtarget",hangtarget);
    }
    public void hangTets2(){
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.y){
            if (!isPressingY2) {
                if (hangtarget == 0) {hangtarget = 9517;}
                else if (hangtarget == 9517) {hangtarget = 5287;}
                else {hangtarget = 0;}
                isPressingY2 = true;
            }
        }
        else isPressingY2=false;
        hang.setPower(hangdynamicKp*(hangtarget-hang.getCurrentPosition()));
        telemetry.addData("hangpos",hang.getCurrentPosition());
        telemetry.addData("hangtarget",hangtarget);
    }
    public void extendoTets(){
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        if (gamepad2.left_trigger>0){
            extendotarget+=15;
        }
        else if (gamepad2.right_trigger>0 && extendotarget>15){
            extendotarget-=15;
        }
        extendo.setPower(extendodynamicKp*(extendotarget-extendo.getCurrentPosition()));

         */
        telemetry.addData("extendopos",extendo.getCurrentPosition());
        telemetry.addData("extendotarget",extendotarget);
    }
    public void clawFingerTets(){
        if (gamepad2.right_stick_y>0&&fingertarget<270){
            fingertarget+=0.5;
        }
        else if (gamepad2.right_stick_y<0&&fingertarget>0){
            fingertarget-=0.5;
        }
        clawfingers.setPosition(fingertarget/fingermax);
        telemetry.addData("clawfingerpos",clawfingers.getPosition());
        telemetry.addData("clawfinger",fingertarget);
    }
    public void clawRollTets(){
        if (gamepad1.right_stick_y>0&&rolltarget<225) {
            rolltarget+=0.5;
        }
        else if (gamepad1.right_stick_y<0&&rolltarget>45){
            rolltarget-=0.5;
        }
        clawroll.setPosition(rolltarget/clawrollmax);
        telemetry.addData("clawroll",rolltarget);
        telemetry.addData("clawrollpos",clawroll.getPosition());
    }
    public void clawPitchTets(){
        pitch1.setDirection(Servo.Direction.REVERSE);
        if (gamepad1.left_stick_y<0&&pitchtarget>0) {
            pitchtarget-=0.5;
        }
        else if (gamepad1.left_stick_y>0&&pitchtarget<270){
            pitchtarget+=0.5;
        }
        pitch1.setPosition(pitchtarget/pitchmax);
        pitch2.setPosition(pitchtarget/pitchmax);
        telemetry.addData("clawpitch",pitchtarget);
        telemetry.addData("clawpitchpos",pitch1.getPosition());
    }
    public void bucketSwervoTets(){
        if (gamepad2.left_stick_y>0&&bucketswervotarget<270) {
            bucketswervotarget+=0.5;
        }
        else if (gamepad2.left_stick_y<0&&bucketswervotarget>0){
            bucketswervotarget-=0.5;
        }
        bucketswervo.setPosition(bucketswervotarget/bucketpitchmax);
        telemetry.addData("bucketpitch",bucketswervotarget);
        telemetry.addData("bucketpitchpos",bucketswervo.getPosition());
    }
}
