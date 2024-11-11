package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.max;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class MidCycleAutoTetsByTristan extends LinearOpMode{
    public void runOpMode(){
        Action onePlusFourSpecimen = myBot.getDrive().actionBuilder(new Pose2d(24,-62, Math.toRadians(90)))
                // Score preload
                .strafeToLinearHeading(new Vector2d(9,-58), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 1
                .strafeToLinearHeading(new Vector2d(47.8,-44), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 2
                .strafeToLinearHeading(new Vector2d(58,-44), Math.toRadians(90))
                .waitSeconds(1)
                // Go to sample zone 3
                .strafeToLinearHeading(new Vector2d(62,-44), Math.toRadians(76))
                .waitSeconds(1)
                // Go to OCP
                .strafeToLinearHeading(new Vector2d(9,-59.5), Math.toRadians(0))
                .waitSeconds(1)
                // Score
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)

                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                // Park
                .strafeToLinearHeading(new Vector2d(30,-62), Math.toRadians(90))
                .build();
    }
}
