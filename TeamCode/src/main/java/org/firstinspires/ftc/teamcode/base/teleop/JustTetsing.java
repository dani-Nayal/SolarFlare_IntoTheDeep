package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitchRight;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSequentialAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpParallelAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UninterruptiblePressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSleepAction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;

@TeleOp
public class JustTetsing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry,new Pose2d(0,0,Math.toRadians(90)));
        UninterruptiblePressTrigger sequence = new UninterruptiblePressTrigger(
                new Condition[]{()->(gamepad1.y)},
                new TeleOpAction[]{
                        new TeleOpSequentialAction(
                                new TeleOpParallelAction(
                                        clawPitch.setPositionAction(clawPitch.getPos("hoverPosition")),
                                        new TeleOpSequentialAction(
                                                new TeleOpSleepAction(0.1),
                                                clawPitch.setPositionAction(clawPitch.getPos("pickUpPosition"))
                                        )
                                ),
                                clawFingers.setPositionAction(0)
                        )
                }
        );
        waitForStart();
        clawFingers.setPosition(clawFingers.getPos("openPosition"));
        clawPitch.setPosition(clawPitch.getPos("pickUpPosition"));
        innerClawPitch.setPosition(innerClawPitch.getPos("pickUpPosition"));
        TeleOpActions.runLoop(
                this::opModeIsActive,
                sequence,
                new UpdateTelemetryAction()
        );
    }
}
