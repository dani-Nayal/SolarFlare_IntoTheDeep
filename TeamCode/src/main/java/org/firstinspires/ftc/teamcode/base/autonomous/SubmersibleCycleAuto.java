package org.firstinspires.ftc.teamcode.base.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous
public class SubmersibleCycleAuto extends OpMode {
    public double[][] values = new double[3][3];
    public int selectedRow = 0;
    public int selectedColumn = 0;
    public boolean dpadPressed = false;
    public boolean firstLoop=true;
    public String[][] labelArray = new String[][]{new String[]{"Samp 1 X","Samp 1 Y","Samp 1 Deg"}, new String[]{"Samp 2 X","Samp 2 Y","Samp 2 Deg"}, new String[]{"Samp 3 X","Samp 3 Y","Samp 3 Deg"}};

    PinpointDrive drive;
    public double robotLength = 15.364;
    public double robotWidth  = 14.375;

    public final int SERVO_SPEED=555;
    public final int EXTENDO_RETRACTED = 0;
    public final int EXTENDO_SCORE_SPECIMEN_UP = 480;
    public final int EXTENDO_SCORE_SPECIMEN_DOWN = 300;
    public final int EXTENDO_PITCH_TRANSFER = 0;
    public final int EXTENDO_PITCH_SCORE_SPECIMEN = 0;
    public final int EXTENDO_PITCH_PICK_UP = -1030;
    public final int EXTENDO_PITCH_GRAB_SPECIMEN = -960;
    public final int BUCKET_SLIDES_HIGH_BUCKET = 1030;
    public final int BUCKET_SLIDES_TRANSFER = 0;
    public final int BUCKET_SLIDES_SCORING_SPECIMEN = 350;
    public final int CLAW_FINGERS_OPEN = 86;
    public final int CLAW_FINGERS_CLOSED = 20;
    public final int CLAW_WRIST_DEFAULT = 95;
    public final int CLAW_PITCH_PICK_UP = 13;
    public final int CLAW_PITCH_HOVER = 68;
    public final int CLAW_PITCH_TRANSFER = 100;
    public final int CLAW_PITCH_BACK_OFF = 72;
    public final int CLAW_PITCH_GRAB_SPECIMEN = 145;
    public final int CLAW_PITCH_SCORE_SPECIMEN = 13;
    public final int INNER_CLAW_PITCH_PICK_UP = 82;
    public final int INNER_CLAW_PITCH_HOVER = 20;
    public final int INNER_CLAW_PITCH_TRANSFER = 200;
    public final int INNER_CLAW_PITCH_BACK_OFF = 100;
    public final int INNER_CLAW_PITCH_GRAB_SPECIMEN = 78;
    public final int INNER_CLAW_PITCH_SCORE_SPECIMEN = 82;
    public final int BUCKET_TRANSFER = 46;
    public final int BUCKET_DEPOSIT = 158;
    DcMotorEx extendo;
    DcMotorEx extendoPitch;
    DcMotorEx bucketSlides;
    Servo clawPitchLeft;
    Servo clawPitchRight;
    Servo innerClawPitch;
    Servo clawFingers;
    Servo bucket;
    Servo clawWrist;
    int extendoTarget = EXTENDO_RETRACTED;
    int extendoPitchTarget = EXTENDO_PITCH_TRANSFER;
    int bucketSlidesTarget = BUCKET_SLIDES_TRANSFER;

    public class MotorPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            double extendoPower = (extendoTarget - extendo.getCurrentPosition()) * 0.014;
            double extendoPitchPower = (extendoPitchTarget - extendoPitch.getCurrentPosition()) * 0.01;
            double bucketSlidesPower = (bucketSlidesTarget - bucketSlides.getCurrentPosition()) * 0.015;

            extendo.setPower(extendoPower);
            extendoPitch.setPower(extendoPitchPower);
            bucketSlides.setPower(bucketSlidesPower);

            telemetry.addData("extendo target", extendoTarget);
            telemetry.addData("extendoPitch target", extendoPitchTarget);
            telemetry.addData("bucketSlidesTarget target", bucketSlidesTarget);

            telemetry.addData("extendo position", extendo.getCurrentPosition());
            telemetry.addData("extendoPitch position", extendoPitch.getCurrentPosition());
            telemetry.addData("bucketSlides position", bucketSlides.getCurrentPosition());
            telemetry.update();

            return true;
        }
    }

    public class SetExtendoTargetAction implements Action {
        int target;
        public SetExtendoTargetAction(int target){
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            extendoTarget = target;
            return !(Math.abs(extendoTarget - extendo.getCurrentPosition()) < 30);
        }
    }

    public class SetExtendoPitchTargetAction implements Action{
        int target;
        public SetExtendoPitchTargetAction(int target){
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            extendoPitchTarget = target;
            return !(Math.abs(extendoPitchTarget - extendoPitch.getCurrentPosition()) < 30);
        }
    }

    public class SetBucketSlidesTargetAction implements Action{
        int target;
        public SetBucketSlidesTargetAction(int target){
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            bucketSlidesTarget = target;
            return !(Math.abs(bucketSlidesTarget - bucketSlides.getCurrentPosition()) < 30);
        }
    }

    public class SetClawPitchPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetClawPitchPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                clawPitchLeft.setPosition(position / 270);
                clawPitchRight.setPosition(position / 270);
                time=Math.abs(position-clawPitchLeft.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetInnerClawPitchPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetInnerClawPitchPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                innerClawPitch.setPosition(position / 270);
                time=Math.abs(position-innerClawPitch.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetClawFingersPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetClawFingersPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                clawFingers.setPosition(position / 180);
                time=Math.abs(position-clawFingers.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetBucketPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetBucketPositionAction(double position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart) {
                isStart=false;
                bucket.setPosition(position / 270);
                time=Math.abs(position-bucket.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds() > time);
        }
    }

    public class SetClawWristPositionAction implements Action {
        double position;
        ElapsedTime timer = new ElapsedTime();
        boolean isStart = true;
        double time;
        public SetClawWristPositionAction(double position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if (isStart){
                isStart=false;
                clawWrist.setPosition(position / 270);
                time=Math.abs(position-clawWrist.getPosition())/SERVO_SPEED;
                timer.reset();
            }
            return !(timer.seconds()> time);
        }
    }

    @Override
    public void init() {}

    @Override
    public void init_loop() {

        if (gamepad1.dpad_up && selectedRow>0&&!dpadPressed){
            selectedRow-=1;
        }
        else if (gamepad1.dpad_down && selectedRow<2&&!dpadPressed){
            selectedRow+=1;
        }
        else if (gamepad1.dpad_left && selectedColumn>0&&!dpadPressed){
            selectedColumn-=1;
        }
        else if (gamepad1.dpad_right && selectedColumn<2&&!dpadPressed){
            selectedColumn+=1;
        }
        dpadPressed = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;

        if (!labelArray[selectedRow][selectedColumn].startsWith("[")){
            labelArray[selectedRow][selectedColumn]="["+labelArray[selectedRow][selectedColumn]+"]";
        }
        for (int x = 0;x<3;x++){
            for (int y = 0;y<3;y++){
                if (x!=selectedRow||y!=selectedColumn){
                    if (labelArray[x][y].startsWith("[")){
                        labelArray[x][y]= (String) labelArray[x][y].subSequence(1,labelArray[x][y].length()-1);
                    }
                }
            }
        }

        if (gamepad1.right_trigger>0){
            if (selectedColumn!=2){
                values[selectedRow][selectedColumn]+=0.01;
            }
            else{
                if (values[selectedRow][selectedColumn]<89.95) {
                    values[selectedRow][selectedColumn] += 0.05;
                }
                else{
                    values[selectedRow][selectedColumn] = 90;
                }
            }
        }
        else if (gamepad1.left_trigger>0){
            if (selectedColumn!=2){
                values[selectedRow][selectedColumn]-=0.01;
            }
            else{
                if (values[selectedRow][selectedColumn]>-89.95) {
                    values[selectedRow][selectedColumn] -= 0.05;
                }
                else{
                    values[selectedRow][selectedColumn] = -90;
                }
            }
        }
        else if (gamepad1.right_bumper){
            if (selectedColumn!=2){
                values[selectedRow][selectedColumn]+=0.001;
            }
            else{
                if (values[selectedRow][selectedColumn]<89.995) {
                    values[selectedRow][selectedColumn] += 0.005;
                }
                else{
                    values[selectedRow][selectedColumn] = 90;
                }
            }
        }
        else if (gamepad1.left_bumper){
            if (selectedColumn!=2){
                values[selectedRow][selectedColumn]-=0.001;
            }
            else{
                if (values[selectedRow][selectedColumn]>-89.995) {
                    values[selectedRow][selectedColumn] -= 0.005;
                }
                else{
                    values[selectedRow][selectedColumn] = -90;
                }
            }
        }

        telemetry.addData(labelArray[0][0]+": " + values[0][0]+"    "+labelArray[0][1]+": " + values[0][1]+", "+labelArray[0][2]+": " + values[0][2],"");
        telemetry.addData(labelArray[1][0]+": " + values[1][0]+", "+labelArray[1][1]+": " + values[1][1]+", "+labelArray[1][2]+": " + values[1][2],"");
        telemetry.addData(labelArray[2][0]+": " + values[2][0]+", "+labelArray[2][1]+": " + values[2][1]+", "+labelArray[2][2]+": " + values[2][2],"");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (firstLoop){
            extendo = hardwareMap.get(DcMotorEx.class, "extendo");
            extendoPitch = hardwareMap.get(DcMotorEx.class, "extendoPitch");
            bucketSlides = hardwareMap.get(DcMotorEx.class, "bucketSlides");

            clawPitchLeft = hardwareMap.servo.get("clawPitchLeft");
            clawPitchRight = hardwareMap.servo.get("clawPitchRight");
            innerClawPitch = hardwareMap.servo.get("innerClawPitch");
            clawFingers = hardwareMap.servo.get("clawFingers");
            bucket = hardwareMap.servo.get("bucket");
            clawWrist = hardwareMap.servo.get("clawWrist");

            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendoPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bucketSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendoPitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bucketSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            extendo.setDirection(DcMotorSimple.Direction.REVERSE);
            bucketSlides.setDirection(DcMotorSimple.Direction.REVERSE);

            clawPitchRight.setDirection(Servo.Direction.REVERSE);
            innerClawPitch.setDirection(Servo.Direction.REVERSE);

            drive = new PinpointDrive(hardwareMap, new Pose2d((robotWidth / 2), -70 + (robotLength / 2), Math.toRadians(90)));


        }
    }
}
