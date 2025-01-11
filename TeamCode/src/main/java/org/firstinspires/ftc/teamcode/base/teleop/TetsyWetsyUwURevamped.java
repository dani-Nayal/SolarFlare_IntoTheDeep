package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucket;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitchLeft;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitchRight;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawWrist;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSequentialAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpParallelAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.PressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UninterruptiblePressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;

@TeleOp
public class TetsyWetsyUwURevamped extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry);
        TeleOpSequentialAction sequence = new TeleOpSequentialAction(
                new TeleOpParallelAction(
                        clawFingers.setPositionAction(clawFingers.KEY_POSITIONS.get("openPosition")),
                        clawPitchLeft.setPositionAction(clawPitchLeft.KEY_POSITIONS.get("pickUpPosition")),
                        clawPitchRight.setPositionAction(clawPitchRight.KEY_POSITIONS.get("pickUpPosition")),
                        innerClawPitch.setPositionAction(innerClawPitch.KEY_POSITIONS.get("pickUpPosition"))
                ),
                clawFingers.setPositionAction(clawFingers.KEY_POSITIONS.get("closedPosition")),
                new TeleOpParallelAction(
                    clawPitchLeft.setPositionAction(clawPitchLeft.KEY_POSITIONS.get("hoverPosition")),
                    clawPitchRight.setPositionAction(clawPitchRight.KEY_POSITIONS.get("hoverPosition")),
                    innerClawPitch.setPositionAction(innerClawPitch.KEY_POSITIONS.get("hoverPosition"))
                )

        );
        UninterruptiblePressTrigger trigger = new UninterruptiblePressTrigger(new Condition[]{()->(gamepad1.y)},new TeleOpAction[]{sequence});

        waitForStart();

        clawFingers.setPosition(92);
        //clawWrist.setPosition(0);
        clawPitchLeft.setPosition(130);
        clawPitchRight.setPosition(130);
        innerClawPitch.setPosition(innerClawPitch.KEY_POSITIONS.get("hoverPosition"));
        //bucket.setPosition(0);
        TeleOpActions.runLoop(
                this::opModeIsActive,
                this::isStopRequested,
                //clawFingers.triggeredDynamicAction(()->(gamepad1.dpad_up),()->(gamepad1.dpad_down),0.25),
                //clawWrist.triggeredDynamicAction(()->(gamepad1.dpad_right),()->(gamepad1.dpad_left),1),
                //clawPitchLeft.triggeredDynamicAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),0.1),
                //clawPitchRight.triggeredDynamicAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),0.1),
                //innerClawPitch.triggeredDynamicAction(()->(gamepad1.a),()->(gamepad1.b),1),
                //bucket.triggeredDynamicAction(()->(gamepad1.right_stick_y>0),()->(gamepad1.right_stick_y<0),1),
                //new UpdateTelemetryAction()
                trigger,
                clawPitchLeft.triggeredFSMAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),clawPitchLeft.KEY_POSITIONS.get("pickUpPosition"),clawPitchLeft.KEY_POSITIONS.get("hoverPosition"),130),
                clawPitchRight.triggeredFSMAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),clawPitchLeft.KEY_POSITIONS.get("hoverPosition"),clawPitchLeft.KEY_POSITIONS.get("pickUpPosition"),130)
        );
    }
}
