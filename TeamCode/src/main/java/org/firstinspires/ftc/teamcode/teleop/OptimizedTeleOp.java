package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.RobotState;

public class OptimizedTeleOp extends LinearOpMode {
    HardwareConfig hw;
    RobotState state;
    @Override
    public void runOpMode(){
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();
        state = new RobotState();

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

        }
    }
}
