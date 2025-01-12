package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitchLeft;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSequentialAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpParallelAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UninterruptiblePressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;

@TeleOp
public class JustTetsing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry);
        TeleOpSequentialAction sequence = new TeleOpSequentialAction(
                new TeleOpParallelAction(
                        clawFingers.setPositionAction(clawFingers.KEY_POSITIONS.get("openPosition")),
                        clawPitchLeft.setPositionAction(clawPitchLeft.KEY_POSITIONS.get("pickUpPosition")),
                        innerClawPitch.setPositionAction(innerClawPitch.KEY_POSITIONS.get("pickUpPosition"))
                ),
                clawFingers.setPositionAction(clawFingers.KEY_POSITIONS.get("closedPosition")),
                new TeleOpParallelAction(
                    clawPitchLeft.setPositionAction(clawPitchLeft.KEY_POSITIONS.get("hoverPosition")),
                    innerClawPitch.setPositionAction(innerClawPitch.KEY_POSITIONS.get("hoverPosition"))
                )

        );
        UninterruptiblePressTrigger trigger = new UninterruptiblePressTrigger(new Condition[]{()->(gamepad1.y)},new TeleOpAction[]{sequence});

        waitForStart();
        clawFingers.setPosition(92);
        clawPitchLeft.setPosition(clawPitchLeft.KEY_POSITIONS.get("hoverPosition"));
        innerClawPitch.setPosition(innerClawPitch.KEY_POSITIONS.get("hoverPosition"));
        TeleOpActions.runLoop(
                this::opModeIsActive,
                this::isStopRequested,
                new UpdateTelemetryAction(),
                trigger,
                clawPitchLeft.triggeredFSMAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),clawPitchLeft.KEY_POSITIONS.get("pickUpPosition"),clawPitchLeft.KEY_POSITIONS.get("hoverPosition"),130),
                extendo.triggeredDynamicAction(()->(gamepad1.right_trigger>0),()->(gamepad1.left_trigger>0),15,6000,3000)
        );
    }
}
