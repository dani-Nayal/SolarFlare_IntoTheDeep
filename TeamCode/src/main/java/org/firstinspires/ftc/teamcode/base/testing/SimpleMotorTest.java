package org.firstinspires.ftc.teamcode.base.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SimpleMotorTest extends LinearOpMode {
    public void runOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get("motor-1");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                motor.setPower(0);
            }
            else if (gamepad1.b){
                motor.setPower(0.25);
            }
            else if (gamepad1.y){
                motor.setPower(0.75);
            }
            else if (gamepad1.x){
                motor.setPower(1.0);
            }
        }
    }
}
