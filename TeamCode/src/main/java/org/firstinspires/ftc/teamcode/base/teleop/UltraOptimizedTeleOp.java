package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitchLeft;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawWrist;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.BotMotor;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.leftBack;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.leftFront;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.rightBack;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.rightFront;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.RobotCentricMecanumAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSequentialAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpParallelAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UninterruptiblePressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;

@TeleOp
public class UltraOptimizedTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry);
        waitForStart();
        clawFingers.setPosition(clawFingers.get("openPosition"));
        clawWrist.setPosition(clawWrist.get("normalPosition"));
        clawPitchLeft.setPosition(clawPitchLeft.get("transferPosition"));
        innerClawPitch.setPosition(innerClawPitch.get("transferPosition"));
        TeleOpActions.runLoop(
                this::opModeIsActive,
                this::isStopRequested,
                new UpdateTelemetryAction(),
                new RobotCentricMecanumAction(new BotMotor[]{leftFront,leftBack,rightFront,rightBack},()->(gamepad1.left_stick_x),()->(gamepad1.left_stick_y),()->(gamepad1.right_stick_x),()->(gamepad1.left_trigger>0.2))
        );
    }
}
