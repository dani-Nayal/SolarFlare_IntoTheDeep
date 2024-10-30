package org.firstinspires.ftc.teamcode.tets;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class TetsyWetsyUwU extends LinearOpMode {
    int armtarget=0;
    int buckettarget=0;
    int hangtarget=0;
    int extendotarget=0;
    double pitchtarget=135;
    double dynamicKp=0.015;
    boolean isPressingA=false;
    boolean isPressingB=false;
    @Override
    public void runOpMode() throws InterruptedException {
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
            armTets();
        }
    }
    public void armTets(){
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
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
        DcMotorEx bucket = hardwareMap.get(DcMotorEx.class, "bucket");
        bucket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad1.left_bumper){
            buckettarget-=15;
        }
        else if (gamepad1.right_bumper){
            buckettarget+=15;
        }
        bucket.setPower(dynamicKp*(armtarget-bucket.getCurrentPosition()));
        telemetry.addData("bucketpos",bucket.getCurrentPosition());
        telemetry.addData("buckettarget",buckettarget);
    }
    public void hangTets(){
        DcMotorEx hang = hardwareMap.get(DcMotorEx.class, "hang");
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.left_bumper){
            hangtarget-=100;
        }
        else if (gamepad2.right_bumper){
            hangtarget+=100;
        }
        hang.setPower(dynamicKp*(hangtarget-hang.getCurrentPosition()));
        telemetry.addData("hangpos",hang.getCurrentPosition());
        telemetry.addData("hangtarget",hangtarget);
    }
    public void extendoTets(){
        DcMotorEx extendo=hardwareMap.get(DcMotorEx.class, "extendo");
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
        Servo clawfingers = hardwareMap.get(Servo.class, "clawfingers");
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
        Servo clawroll = hardwareMap.get(Servo.class, "clawroll");
        if (gamepad1.b) {
            if (isPressingB) {
                if(clawroll.getPosition()==0){clawroll.setPosition(0.5);}else{clawroll.setPosition(0);}
                isPressingB=true;
            }
        }
        else{
            isPressingB=false;
        }
    }
    public void clawPitchTets(){
        Servo pitch1 = hardwareMap.get(Servo.class, "clawpitch");
        Servo pitch2 = hardwareMap.get(Servo.class, "clawpitch");
        if (gamepad1.left_stick_y>0&&pitchtarget<210) {
            pitchtarget+=15;
        }
        else if (gamepad1.left_stick_y<0&&pitchtarget>60){
            pitchtarget-=15;
        }
        pitch1.setPosition(pitchtarget/270);
        pitch2.setPosition(pitchtarget/270);
        telemetry.addData("clawpitch",pitchtarget);
    }
}
