package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareConfig;
@TeleOp
public class ExtendoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            hw.extendo.motor.setPower(gamepad1.left_stick_y); // Test extendo with joystick
            telemetry.addData("Extendo Position", hw.extendo.motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
