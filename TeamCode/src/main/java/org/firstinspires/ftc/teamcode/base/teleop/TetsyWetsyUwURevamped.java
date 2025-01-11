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

@TeleOp
public abstract class TetsyWetsyUwURevamped extends LinearOpMode {
    @Override
    public void runOpMode(){
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry);
        waitForStart();
        TeleOpActions.runLoop(
                this::opModeIsActive,
                this::isStopRequested,
                clawFingers.triggeredDynamicAction(()->(gamepad1.dpad_up),()->(gamepad1.dpad_down),1),
                clawWrist.triggeredDynamicAction(()->(gamepad1.dpad_right),()->(gamepad1.dpad_left),1),
                clawPitchLeft.triggeredDynamicAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),1),
                clawPitchRight.triggeredDynamicAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),1),
                innerClawPitch.triggeredDynamicAction(()->(gamepad1.a),()->(gamepad1.b),1),
                bucket.triggeredDynamicAction(()->(gamepad1.right_stick_y>0),()->(gamepad1.right_stick_y<0),1),
                new UpdateTelemetryAction()
        );
    }
}
