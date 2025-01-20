package org.firstinspires.ftc.teamcode.base.teleop;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.isRRActive;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucket;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.bucketSlides;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawFingers;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.clawWrist;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendo;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.extendoPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.innerClawPitch;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.BotMotor;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.leftBack;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.leftFront;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.rightBack;
import static org.firstinspires.ftc.teamcode.base.teleop.TeleOpComponents.rightFront;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UpdateTelemetryAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpSequentialAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpParallelAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.UninterruptiblePressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.ConditionalAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.RobotCentricMecanumAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;

@TeleOp
public class UltraOptimizedTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpComponents.initializeMechanisms(hardwareMap,telemetry);
        waitForStart();
        clawFingers.setPosition(clawFingers.getPos("openPosition"));
        clawWrist.setPosition(clawWrist.getPos("normalPosition"));
        clawPitch.setPosition(clawPitch.getPos("transferPosition"));
        innerClawPitch.setPosition(innerClawPitch.getPos("transferPosition"));
        bucket.setPosition(bucket.getPos("transferPosition"));

        UninterruptiblePressTrigger lowerIntakeSequence = new UninterruptiblePressTrigger(new Condition[]{()->(gamepad1.a)},
                new TeleOpAction[]{
                        new TeleOpParallelAction(
                                extendoPitch.moveToPositionAction(extendoPitch.getPos("pickUpPosition")),
                                clawFingers.setPositionAction(clawFingers.getPos("openPosition")),
                                clawPitch.setPositionAction(clawPitch.getPos("hoverPosition")),
                                innerClawPitch.setPositionAction(innerClawPitch.getPos("hoverPosition")),
                                extendo.moveToPositionAction(extendo.MAX_POSITION)
                        )
                });
        UninterruptiblePressTrigger pickUpSequence = new UninterruptiblePressTrigger(new Condition[]{()->(gamepad1.b)},
                new TeleOpAction[]{
                        new TeleOpSequentialAction(
                            new TeleOpParallelAction(
                                    innerClawPitch.setPositionAction(innerClawPitch.getPos("pickUpPosition")),
                                    clawPitch.setPositionAction(clawPitch.getPos("pickUpPosition"))
                            ),
                            clawFingers.setPositionAction(clawFingers.getPos("closedPosition")),
                            new TeleOpParallelAction(
                                    innerClawPitch.setPositionAction(innerClawPitch.getPos("hoverPosition")),
                                    clawPitch.setPositionAction(clawPitch.getPos("hoverPosition"))
                            )
                        )
                });
        UninterruptiblePressTrigger transferSequence = new UninterruptiblePressTrigger(new Condition[]{()->(gamepad1.x)},
                new TeleOpAction[]{
                        new TeleOpSequentialAction(
                                new TeleOpParallelAction(
                                        clawWrist.setPositionAction(clawWrist.getPos("normalPosition")),
                                        extendo.moveToPositionAction(extendo.MIN_POSITION),
                                        clawPitch.setPositionAction(clawPitch.getPos("hoverPosition")),
                                        innerClawPitch.setPositionAction(innerClawPitch.getPos("hoverPosition"))
                                ),
                                new TeleOpParallelAction(
                                        extendoPitch.moveToPositionAction(extendoPitch.getPos("transferPosition")),
                                        clawPitch.setPositionAction(clawPitch.getPos("transferPosition")),
                                        innerClawPitch.setPositionAction(innerClawPitch.getPos("transferPosition"))
                                ),
                                clawFingers.setPositionAction(clawFingers.getPos("openPosition")),
                                new TeleOpParallelAction(
                                        clawPitch.setPositionAction(clawPitch.getPos("backOffPosition")),
                                        innerClawPitch.setPositionAction(innerClawPitch.getPos("backOffPosition"))
                                )
                        )
                });
        UninterruptiblePressTrigger specimenGrabSequence = new UninterruptiblePressTrigger(new Condition[]{()->(gamepad1.y)},
                new TeleOpAction[]{
                            new TeleOpParallelAction(
                                    extendoPitch.moveToPositionAction(extendoPitch.getPos("specimenGrabPosition")),
                                    clawWrist.setPositionAction(clawWrist.getPos("normalPosition")),
                                    extendo.moveToPositionAction(extendo.MAX_POSITION),
                                    clawPitch.setPositionAction(clawPitch.getPos("specimenGrabPosition")),
                                    innerClawPitch.setPositionAction(clawPitch.getPos("specimenGrabPosition"))
                            )
                });
        UninterruptiblePressTrigger setUpSpecimenDepositSequence = new UninterruptiblePressTrigger(new Condition[]{()->(gamepad1.options)},
                new TeleOpAction[]{
                        new TeleOpSequentialAction(
                            clawFingers.setPositionAction(clawFingers.getPos("closedPosition")),
                            extendoPitch.moveToPositionAction(extendoPitch.getPos("specimenGrabPosition")+50),
                            new TeleOpParallelAction(
                                    extendo.moveToPositionAction(extendo.MIN_POSITION),
                                    clawPitch.setPositionAction(clawPitch.getPos("specimenDepositPosition")),
                                    innerClawPitch.setPositionAction(clawPitch.getPos("specimenDepositPosition"))
                            ),
                            extendoPitch.moveToPositionAction(extendoPitch.getPos("specimenDepositPosition")),
                            extendo.moveToPositionAction(extendo.MAX_POSITION)
                        )
                });
        TeleOpActions.runLoop(
                this::opModeIsActive,
                clawFingers.triggeredToggleAction(()->(gamepad1.right_trigger>0),clawFingers.getPos("openPosition"),clawFingers.getPos("closedPosition")),
                clawWrist.triggeredDynamicAction(()->(gamepad1.right_bumper),()->(gamepad1.left_bumper),1),
                clawPitch.triggeredFSMAction(()->(gamepad1.dpad_left),()->(gamepad1.dpad_right),clawPitch.getPos("transferPosition"),clawPitch.getPos("hoverPosition"),clawPitch.getPos("pickUpPosition")),
                bucket.triggeredToggleAction(()->(gamepad2.a),bucket.getPos("transferPosition"),bucket.getPos("depositPosition")),
                extendoPitch.triggeredFSMAction(()->(gamepad1.dpad_up),()->(gamepad1.dpad_down),extendoPitch.getPos("transferPosition"),extendoPitch.getPos("pickUpPosition")),
                extendoPitch.triggeredDynamicAction(()->(gamepad2.right_stick_y>0),()->(gamepad2.right_stick_y<0),15),
                bucketSlides.triggeredToggleAction(()->(gamepad2.y),bucketSlides.getPos("depositPosition"),bucketSlides.getPos("transferPosition")),
                lowerIntakeSequence,
                pickUpSequence,
                transferSequence,
                specimenGrabSequence,
                setUpSpecimenDepositSequence,
                new ConditionalAction(new Condition[]{()->(!isRRActive)}, new TeleOpAction[]{
                        new RobotCentricMecanumAction(new BotMotor[]{leftFront,leftBack,rightFront,rightBack},()->(gamepad1.left_stick_x),()->(gamepad1.left_stick_y),()->(gamepad1.right_stick_x),()->(gamepad1.left_trigger>0.2))
                }),
                new UpdateTelemetryAction()
        );
    }
}
