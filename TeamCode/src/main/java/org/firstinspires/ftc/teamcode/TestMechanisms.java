package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestMechanisms extends LinearOpMode {
    HardwareConfig hw;
    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-70+(hw.ROBOT_LENGTH/2), -hw.ROBOT_WIDTH/2, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        HardwareConfig.makeHardwareConfig(hardwareMap);
        hw = HardwareConfig.getHardwareConfig();

        waitForStart();


        hw.getMotorConfig(MotorEnum.EXTENDO).motor.setPower(0.2);
        hw.getMotorConfig(MotorEnum.EXTENDO_PITCH).motor.setPower(0.2);
        hw.getMotorConfig(MotorEnum.HANG).motor.setPower(0.2);
        hw.getMotorConfig(MotorEnum.LEFT_FRONT).motor.setPower(0.2);
        hw.getMotorConfig(MotorEnum.RIGHT_FRONT).motor.setPower(0.2);
        hw.getMotorConfig(MotorEnum.RIGHT_BACK).motor.setPower(0.2);
        hw.getMotorConfig(MotorEnum.LEFT_BACK).motor.setPower(0.2);
        hw.getMotorConfig(MotorEnum.BUCKET_SLIDES).motor.setPower(0.2);

        hw.getServoConfig(ServoEnum.CLAW_WRIST);


    }
}
