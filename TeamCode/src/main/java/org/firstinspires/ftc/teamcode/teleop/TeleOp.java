package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double hangTarget = 0;
        double kP = 0.015;


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        DcMotor hang = hardwareMap.dcMotor.get("hang");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // If target is less than 15 immediately set target to 0 when going down
            // Prevents negative target
            if (gamepad2.left_bumper) {
                if (hangTarget >= 15) {
                    hangTarget -= 15;
                }
                else{
                    hangTarget=0;
                }
            }
            // If target is greater than 3619 set target to max when going down
            // Prevents target from going over
            else if (gamepad2.right_bumper){
                if (hangTarget <= 3619){
                    hangTarget += 15;
                }
                else {
                    hangTarget = 3634 ;
                }

            }
            //if (gamepad2.dpad_down){
          //      hang.setPower(-1);
           // }
            hang.setPower( (hangTarget - hang.getCurrentPosition()) * kP );
            telemetry.addData("hang pos", hang.getCurrentPosition());
            telemetry.addData("heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            telemetry.addData("hang target", hangTarget);
            telemetry.update();

            if (gamepad1.a){

            }
            if (gamepad1.b){

            }
            if (gamepad1.x){

            }
            if (gamepad1.y){

            }
            if (gamepad1.right_bumper){

            }
            if (gamepad1.left_bumper){

            }
            if (gamepad1.left_trigger > 0.5){

            }
            if (gamepad1.right_trigger > 0.5){

            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

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
        }
    }
}