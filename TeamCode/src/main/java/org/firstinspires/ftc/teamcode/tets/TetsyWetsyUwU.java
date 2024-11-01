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
    double pitchtarget=135;
    double rolltarget=135;
    double bucketswervotarget=0;
    double dynamicKp=0.015;
    boolean isPressingA=false;
    boolean isPressingB=false;
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
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        clawfingers = hardwareMap.get(Servo.class, "clawfingers");
        clawroll = hardwareMap.get(Servo.class, "clawroll");
        extendo=hardwareMap.get(DcMotorEx.class, "extendo");
        hang = hardwareMap.get(DcMotorEx.class, "hang");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        pitch1 = hardwareMap.get(Servo.class, "clawpitch");
        pitch1.setDirection(Servo.Direction.REVERSE);
        pitch2 = hardwareMap.get(Servo.class, "clawpitch");
        bucketswervo = hardwareMap.get(Servo.class, "bucketservo");
        */
        bucket=hardwareMap.get(DcMotorEx.class, "bucket");

        waitForStart();
        while (opModeIsActive()){
            if (isStopRequested()) return;
            if (dynamicKp==0.015){
                dynamicKp=0.016;
            }
            else if (dynamicKp==0.016){
                dynamicKp=0.014;
            }
            else if (dynamicKp==0.014){
                dynamicKp=0.015;
            }
            bucketTets();
            telemetry.update();
        }
    }
    public void armTets(){

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad1.left_trigger>0){
            armtarget-=15;
        }
        else if (gamepad1.right_trigger>0){
            armtarget+=15;
        }
        arm.setPower(dynamicKp*(armtarget-arm.getCurrentPosition()));
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
        bucket.setPower(dynamicKp*(buckettarget-bucket.getCurrentPosition()));
        telemetry.addData("bucketpos",bucket.getCurrentPosition());
        telemetry.addData("buckettarget",buckettarget);
    }
    public void hangTets(){

        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.left_bumper){
            hangtarget-=100;
        }
        else if (gamepad2.right_bumper){
            hangtarget+=100;
        }
        hang.setPower(dynamicKp*-(hangtarget-hang.getCurrentPosition()));
        telemetry.addData("hangpos",hang.getCurrentPosition());
        telemetry.addData("hangtarget",hangtarget);
    }
    public void extendoTets(){

        if (gamepad2.left_trigger>0){
            extendotarget+=15;
        }
        else if (gamepad2.right_trigger>0 && extendotarget>15){
            extendotarget-=15;
        }
        extendo.setPower(dynamicKp*(extendotarget-extendo.getCurrentPosition()));
        telemetry.addData("extendopos",extendo.getCurrentPosition());
        telemetry.addData("extendotarget",extendotarget);
    }
    public void clawFingerTets(){

        if (gamepad1.a) {
            if (isPressingA) {
                clawfingers.setPosition(1 - clawfingers.getPosition());
                isPressingA=true;
            }
        }
        else{
            isPressingA=false;
        }
    }
    public void clawRollTets(){
        if (gamepad1.right_stick_y>0&&rolltarget<210) {
            rolltarget+=15;
        }
        else if (gamepad1.left_stick_y<0&&rolltarget>60){
            rolltarget-=15;
        }
        pitch1.setPosition(rolltarget/clawrollmax);
        pitch2.setPosition(rolltarget/clawrollmax);
        telemetry.addData("clawroll",rolltarget);
    }
    public void clawPitchTets(){
        if (gamepad1.left_stick_y>0&&pitchtarget<210) {
            pitchtarget+=15;
        }
        else if (gamepad1.left_stick_y<0&&pitchtarget>60){
            pitchtarget-=15;
        }
        pitch1.setPosition(pitchtarget/pitchmax);
        pitch2.setPosition(pitchtarget/pitchmax);
        telemetry.addData("clawpitch",pitchtarget);
    }
    public void bucketSwervoTets(){
        if (gamepad2.left_stick_y>0) {
            bucketswervotarget+=15;
        }
        else if (gamepad2.left_stick_y<0){
            bucketswervotarget-=15;
        }
        bucketswervo.setPosition(bucketswervotarget/bucketpitchmax);
        telemetry.addData("bucketpitch",bucketswervotarget);
    }
}
