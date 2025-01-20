package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitchRight;
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
        waitForStart();
        TeleOpActions.runLoop(
                this::opModeIsActive,
                new UpdateTelemetryAction(),
                extendo.triggeredMoveToTargetAction(()->(gamepad1.a),400),
                extendo.triggeredMoveToTargetAction(()->(gamepad1.b),800),
                extendo.triggeredDynamicAction(()->(gamepad1.right_trigger>0),()->(gamepad1.left_trigger>0),20)
        );
    }
}
