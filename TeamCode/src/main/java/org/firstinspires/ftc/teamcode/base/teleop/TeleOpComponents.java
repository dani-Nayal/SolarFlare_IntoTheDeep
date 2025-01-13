package org.firstinspires.ftc.teamcode.base.teleop;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.DoubleFunction;
import org.firstinspires.ftc.teamcode.base.teleop.LambdaInterfaces.Condition;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.TeleOpAction;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.PressTrigger;
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.ConditionalAction;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;


public abstract class TeleOpComponents {
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;
    public static PinpointDrive drive;
    public static ArrayList<BotMotor> motors = new ArrayList<>();
    public static ArrayList<BotServo> servos = new ArrayList<>();

    //create mechanism variables here
    public static BotServo clawFingers;
    public static BotServo clawPitch;
    public static BotServo clawPitchRight;
    public static BotServo innerClawPitch;
    public static BotServo clawWrist;
    public static BotServo bucket;
    public static BotMotor extendo;
    public static BotMotor extendoPitch;
    public static BotMotor bucketSlides;
    public static BotMotor rightFront; public static BotMotor rightBack; public static BotMotor leftFront; public static BotMotor leftBack;



    public static class BotMotor extends DcMotorImplEx {
        public ArrayList<BotMotor> synchronizedMotors = new ArrayList<>();
        public double kP; public double kI; public double kD;
        public HashMap<String,Double> KEY_POSITIONS;
        public RunMode RUN_MODE;
        public double MAX_POSITION; public double MIN_POSITION;
        public double MAX_ACCELERATION; public double MAX_VELOCITY;
        public double currentMaxAcceleration = 0; public double currentMaxDeceleration = 0; public double currentMaxVelocity = 0;

        public double target = 0;

        public double accelDT = 0; public double decelDT = 0; public double cruiseDT = 0;
        public double accelDistance = 0; public double decelDistance = 0; public double cruiseDistance = 0;
        public double profileStartPos = 0;
        public double startVelocity = 0;
        final ElapsedTime MOVEMENT_TIMER = new ElapsedTime(); final ElapsedTime LOOP_TIMER = new ElapsedTime();
        double integralSum = 0;
        double previousError = 0;
        double previousVoltage = 0;
        boolean isStallResetting = false;
        String MOVEMENT_MODE;

        public class UpwardFSMAction implements TeleOpAction{
            private final double maxAcceleration;
            private final double maxVelocity;
            private final double[] positions;
            boolean isStart=true;

            public UpwardFSMAction(double maxAcceleration, double maxVelocity, double...positions) {
                this.maxAcceleration = maxAcceleration;
                this.maxVelocity = maxVelocity;
                this.positions = positions;
                Arrays.sort(this.positions);
            }
            public UpwardFSMAction(double...positions) {
                this(MAX_ACCELERATION,MAX_VELOCITY,positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                setMotorTarget(getCurrentPosition());
                setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=target;
                    for (double position : positions){
                        if (position>target){
                            pos=position;
                            break;
                        }
                    }
                    setMotorTarget(pos,maxAcceleration,maxVelocity);
                    isStart=false;
                }
                return Math.abs(target-getCurrentPosition())!=0;
            }
        }
        public class DownwardFSMAction implements TeleOpAction{
            private final double maxAcceleration;
            private final double maxVelocity;
            private final List<Double> positions;
            boolean isStart=true;

            public DownwardFSMAction(double maxAcceleration, double maxVelocity, double...positions) {
                this.maxAcceleration = maxAcceleration;
                this.maxVelocity = maxVelocity;
                Arrays.sort(positions);
                Double[] newPositions = new Double[positions.length];
                for (int i=0;i<positions.length;i++){
                    newPositions[i]= positions[i];
                }
                this.positions=Arrays.asList(newPositions);
                Collections.reverse(this.positions);
            }
            public DownwardFSMAction(double...positions) {
                this(MAX_ACCELERATION,MAX_VELOCITY,positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                setMotorTarget(getCurrentPosition());
                setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=target;
                    for (double position : positions){
                        if (position<target){
                            pos=position;
                            break;
                        }
                    }
                    setMotorTarget(pos,maxAcceleration,maxVelocity);
                    isStart=false;
                }
                return Math.abs(target-getCurrentPosition())!=0;
            }
        }

        public PressTrigger triggeredFSMAction( double maxAcceleration, double maxVelocity, Condition upCondition, Condition downCondition, double...positions){
            return new PressTrigger(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new UpwardFSMAction(maxAcceleration,maxVelocity,positions),new DownwardFSMAction(maxAcceleration,maxVelocity,positions)});
        }
        public PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition,double...positions){
            return new PressTrigger(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new UpwardFSMAction(positions),new DownwardFSMAction(positions)});
        }
        public class SetPowerAction implements TeleOpAction{
            DoubleFunction powerFun;
            public SetPowerAction(double power){
                this.powerFun=() -> (Math.max(-1,Math.min(1,power)));
            }
            public SetPowerAction(DoubleFunction powerFun){
                this.powerFun=powerFun;
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                return run(packet);
            }

            @Override
            public void stop() {
                setPower(0);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setPower(powerFun.call());
                return false;
            }
        }
        public SetPowerAction setPowerAction(double power){
            return new SetPowerAction(power);
        }
        public SetPowerAction setPowerAction(DoubleFunction powerFun){
            return new SetPowerAction(powerFun);
        }
        public class SetTargetAction implements TeleOpAction{
            DoubleFunction targetFun;
            double maxAcceleration;
            double maxVelocity;
            boolean isStart=true;
            public SetTargetAction(double target, double maxAcceleration, double maxVelocity){
                this.targetFun = () -> (target);
                this.maxAcceleration=maxAcceleration;
                this.maxVelocity=maxVelocity;
            }
            public SetTargetAction(DoubleFunction targetFun, double maxAcceleration, double maxVelocity){
                this.targetFun = targetFun;
                this.maxAcceleration=maxAcceleration;
                this.maxVelocity=maxVelocity;
            }
            public SetTargetAction(double target){
                this(target,MAX_ACCELERATION,MAX_VELOCITY);
            }
            public SetTargetAction(DoubleFunction targetFun){
                this(targetFun,MAX_ACCELERATION,MAX_VELOCITY);
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    setMotorTarget(targetFun.call(),maxAcceleration,maxVelocity);
                    isStart=false;
                }
                return Math.abs(target-getCurrentPosition())!=0;
            }

            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                setMotorTarget(getCurrentPosition());
            }

        }
        public SetTargetAction moveToPositionAction(double target, double maxAcceleration, double maxVelocity){
            return new SetTargetAction(target,maxAcceleration,maxVelocity);
        }
        public SetTargetAction moveToPositionAction(DoubleFunction targetFun, double maxAcceleration, double maxVelocity){
            return new SetTargetAction(targetFun,maxAcceleration,maxVelocity);
        }
        public SetTargetAction moveToPositionAction(double target){
            return new SetTargetAction(target,MAX_ACCELERATION,MAX_VELOCITY);
        }
        public SetTargetAction moveToPositionAction(DoubleFunction targetFun){
            return new SetTargetAction(targetFun,MAX_ACCELERATION,MAX_VELOCITY);
        }
        public class StallResetAction implements TeleOpAction{
            public boolean isStart = true;
            public boolean run(@NonNull TelemetryPacket packet){
                if (isStart) {
                    initiateStallReset();
                    isStart=false;
                }
                checkStallResetOnce();
                return isStallResetting;
            }

            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {
                isStallResetting=false;
                setPower(0);
            }
        }
        public StallResetAction stallResetAction(){
            return new StallResetAction();
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change,double maxAcceleration, double maxVelocity){
            return new ConditionalAction(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new SetTargetAction(()->(target+change),maxAcceleration,maxVelocity),new SetTargetAction(()->(target-change),maxAcceleration,maxVelocity)});
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new SetTargetAction(()->(target+change)),new SetTargetAction(()->(target-change))});
        }
        public PressTrigger triggeredMoveToTargetAction(Condition condition, double target, double maxAcceleration, double maxVelocity){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{new SetTargetAction(target,maxAcceleration,maxVelocity)});
        }
        public PressTrigger triggeredMoveToTargetAction(Condition condition, double target){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{new SetTargetAction(target)});
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2, double maxAcceleration, double maxVelocity){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{
                    new ConditionalAction(new Condition[]{()->(target==target1),()->(target==target2)},new TeleOpAction[]{
                            new SetTargetAction(target2,maxAcceleration,maxVelocity),
                            new SetTargetAction(target1,maxAcceleration,maxVelocity)
                    })

            });
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2){
            return triggeredToggleAction(condition,target1,target2,MAX_ACCELERATION,MAX_VELOCITY);
        }
        public BotMotor(String deviceName,
                        DcMotorController controller,
                        int portNumber,
                        @NonNull MotorConfigurationType motorType,

                        double kP,double kI,double kD,
                        String[] keyPositionKeys,
                        double[] keyPositionValues,
                        double maxPosition, double minPosition,
                        double maxAcceleration, double maxVelocity,
                        RunMode runMode, Direction direction, ZeroPowerBehavior zeroPowerBehaviour,
                        String movementMode)
        {
            super(controller, portNumber, DcMotor.Direction.FORWARD,motorType);

            this.kP=kP; this.kI=kI; this.kD=kD;
            this.KEY_POSITIONS = new HashMap<>();
            for (int i=0;i<keyPositionKeys.length;i++){
                this.KEY_POSITIONS.put(keyPositionKeys[i],keyPositionValues[i]);
            }
            this.MAX_POSITION=maxPosition; this.MIN_POSITION = minPosition;
            this.MAX_ACCELERATION=maxAcceleration; this.MAX_VELOCITY=maxVelocity;
            this.RUN_MODE = runMode;
            this.MOVEMENT_MODE=movementMode;

            setMode(RunMode.STOP_AND_RESET_ENCODER);
            setMode(runMode);
            setDirection(direction);
            setZeroPowerBehavior(zeroPowerBehaviour);

            hardwareMap.put(deviceName,this);
            motors.add(this);
        }
        private void createMotionProfile(double max_velocity, double max_acceleration) {
            profileStartPos=getCurrentPosition();
            double distance=target-profileStartPos;
            startVelocity = getVelocity();
            currentMaxVelocity = max_velocity*Math.signum(distance);
            currentMaxAcceleration = max_acceleration*Math.signum(currentMaxVelocity - startVelocity);
            currentMaxDeceleration = -max_acceleration*Math.signum(distance);

            accelDT = (currentMaxVelocity - startVelocity) / currentMaxAcceleration;
            decelDT = (0-currentMaxVelocity) / currentMaxDeceleration;
            accelDistance = startVelocity*accelDT + 0.5 * currentMaxAcceleration * Math.pow(accelDT, 2);
            decelDistance = currentMaxVelocity * decelDT + 0.5 * currentMaxDeceleration * Math.pow(decelDT, 2);

            if (Math.abs(accelDistance+decelDistance) > Math.abs(distance)){
                double halfExceededDistance = (distance-accelDistance-decelDistance)/2;
                accelDistance = accelDistance+halfExceededDistance;
                accelDT = Math.max(
                        (-startVelocity + Math.sqrt(Math.abs(Math.pow(startVelocity,2) + 2*currentMaxAcceleration*accelDistance)))/(currentMaxAcceleration),
                        (-startVelocity - Math.sqrt(Math.abs(Math.pow(startVelocity,2) + 2*currentMaxAcceleration*accelDistance)))/(currentMaxAcceleration)
                );
                currentMaxVelocity = currentMaxAcceleration * accelDT + startVelocity;
                decelDistance = decelDistance+halfExceededDistance;
                decelDT = Math.max(
                        (-currentMaxVelocity + Math.sqrt(Math.abs(Math.pow(currentMaxVelocity,2) + 2*currentMaxDeceleration*decelDistance)))/(currentMaxDeceleration),
                        (-currentMaxVelocity - Math.sqrt(Math.abs(Math.pow(currentMaxVelocity,2) + 2*currentMaxDeceleration*decelDistance)))/(currentMaxDeceleration)
                );
            }
            cruiseDistance = distance - accelDistance - decelDistance;
            cruiseDT = cruiseDistance / currentMaxVelocity;
        }
        public void runMotionProfileOnce(){
            double instantTargetPosition;
            double elapsedTime = MOVEMENT_TIMER.time();
            if (elapsedTime > accelDT+decelDT+cruiseDT){
                instantTargetPosition=target;
            }

            if (elapsedTime < accelDT){
                instantTargetPosition=profileStartPos + startVelocity * elapsedTime + 0.5 * currentMaxAcceleration * Math.pow(elapsedTime, 2);


            }
            else if (elapsedTime < accelDT+cruiseDT){
                double cruiseCurrentDT = elapsedTime - accelDT;
                instantTargetPosition=profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            }

            else {
                double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
                instantTargetPosition = profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentMaxDeceleration * Math.pow(decelCurrentDT, 2);
            }

            double error=instantTargetPosition-getCurrentPosition();
            double kpPower = kP*error;
            integralSum += LOOP_TIMER.time()*error;
            double kiPower = kI*integralSum;
            double kdPower = kD*(error-previousError)/ LOOP_TIMER.time();
            LOOP_TIMER.reset();
            previousError=error;
            setPower(Math.min(1,Math.max(-1,kpPower+kiPower+kdPower)));
        }
        public double getPos(String key){
            return KEY_POSITIONS.get(key);
        }
        public void setMotorTarget(double target, double maxVelocity, double maxAcceleration){
            if (target!=this.target || maxVelocity != currentMaxVelocity || maxAcceleration != currentMaxAcceleration) {
                this.target = Math.min(MAX_POSITION, Math.max(MIN_POSITION, target));
                integralSum = 0;
                previousError = 0;
                MOVEMENT_TIMER.reset();
                LOOP_TIMER.reset();
                createMotionProfile(maxVelocity, maxAcceleration);
                for (BotMotor motor : synchronizedMotors){
                    motor.setMotorTarget(target,maxVelocity,maxAcceleration);
                }
            }
        }
        public void setMotorTarget(double target){
            this.setMotorTarget(target, MAX_VELOCITY, MAX_ACCELERATION);
        }
        public void initiateStallReset(){
            isStallResetting=true;
            setPower(-1);
            previousVoltage = getCurrent(CurrentUnit.AMPS);
        }
        public void checkStallResetOnce(){
            double voltage = getCurrent(CurrentUnit.AMPS);
            if (voltage-previousVoltage>2){
                setPower(0);
                setMode(RunMode.STOP_AND_RESET_ENCODER);
                setMode(RUN_MODE);
                isStallResetting=false;
            }
            else {
                previousVoltage=voltage;
            }
        }
    }

    public static class BotServo extends ServoImpl {
        public ArrayList<BotServo> synchronizedServos = new ArrayList<>();
        public HashMap<String,Double> KEY_POSITIONS;
        public double MAXIMUM_POSITION; public double MINIMUM_POSITION;
        public double RANGE;
        public double SERVO_SPEED;
        public ElapsedTime MOVEMENT_TIMER = null;
        public double startPos = 0;
        public double time;
        public BotServo(String deviceName,
                        ServoController controller,
                        int portNumber,
                        String[] keyPositionKeys,
                        double[] keyPositionValues,
                        double maxPosition,
                        double minPosition,
                        double range,
                        double servoSpeed,
                        Direction direction)
        {
            super(controller, portNumber);

            this.KEY_POSITIONS = new HashMap<>();
            for (int i=0;i<keyPositionKeys.length;i++){
                this.KEY_POSITIONS.put(keyPositionKeys[i],keyPositionValues[i]);
            }
            this.MAXIMUM_POSITION = maxPosition; this.MINIMUM_POSITION =minPosition;
            this.RANGE = range;
            this.SERVO_SPEED = servoSpeed;

            setDirection(direction);

            //hardwareMap.put(deviceName,this);
            servos.add(this);
        }
        @Override
        public void setPosition(double position){
            if (MOVEMENT_TIMER == null){
                MOVEMENT_TIMER = new ElapsedTime();
                startPos=0;
            }
            else {
                startPos = Math.signum(getPosition() - startPos) * SERVO_SPEED * Math.min(time, MOVEMENT_TIMER.time()) + startPos;
            }
            super.setPosition(Math.max(MINIMUM_POSITION,Math.min(MAXIMUM_POSITION,position)) / RANGE);
            for (BotServo servo : synchronizedServos){
                servo.setPosition(position);
            }
            time=Math.abs(getPosition()-startPos)/SERVO_SPEED+0.07;
            MOVEMENT_TIMER.reset();
        }
        @Override
        public double getPosition(){
            return super.getPosition() * RANGE;
        }
        public double getPos(String key){
            return KEY_POSITIONS.get(key);
        }
        public class SetPositionAction implements TeleOpAction {
            boolean isStart = true;
            DoubleFunction posFun;
            public SetPositionAction(double pos){
                this.posFun = () -> (pos);
            }
            public SetPositionAction(DoubleFunction posFun) {
                this.posFun = posFun;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    isStart=false;
                    setPosition(posFun.call());

                }
                return MOVEMENT_TIMER.time() < time;
            }

            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {}
        }
        public SetPositionAction setPositionAction(double pos){
            return new SetPositionAction(pos);
        }
        public SetPositionAction setPositionAction(DoubleFunction posFun){
            return new SetPositionAction(posFun);
        }
        public class UpwardFSMAction implements TeleOpAction{
            private final double[] positions;
            boolean isStart=true;

            public UpwardFSMAction(double...positions) {
                this.positions = positions;
                Arrays.sort(this.positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {}

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=getPosition();
                    isStart=false;
                    for (double position : positions){
                        if (position>pos){
                            pos=position;
                            break;
                        }
                    }
                    setPosition(pos);
                }
                return MOVEMENT_TIMER.time() < time;
            }
        }
        public class DownwardFSMAction implements TeleOpAction{
            private final List<Double> positions;
            boolean isStart=true;
            public DownwardFSMAction(double...positions) {
                Arrays.sort(positions);
                Double[] newPositions = new Double[positions.length];
                for (int i=0;i<positions.length;i++){
                    newPositions[i]= positions[i];
                }
                this.positions=Arrays.asList(newPositions);
                Collections.reverse(this.positions);
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {}

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    double pos=getPosition();
                    isStart=false;
                    for (double position : positions){
                        if (position<pos){
                            pos=position;
                            break;
                        }
                    }
                    setPosition(pos);
                }
                return MOVEMENT_TIMER.time() < time;
            }
        }
        public PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition,double...positions){
            return new PressTrigger(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new UpwardFSMAction(positions),new DownwardFSMAction(positions)});
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new SetPositionAction(()->(getPosition()+change)),new SetPositionAction(()->(getPosition()-change))});
        }
        public PressTrigger triggeredMoveToPositionAction(Condition condition, double target){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{new SetPositionAction(target)});
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2){
            return new PressTrigger(new Condition[]{condition},new TeleOpAction[]{
                    new ConditionalAction(new Condition[]{()->(getPosition()==target1),()->(getPosition()==target2)},new TeleOpAction[]{
                            new SetPositionAction(target2),
                            new SetPositionAction(target1)
                    })

            });
        }
    }
    public static void initializeMechanisms(HardwareMap hardwareMap, Telemetry telemetry){
        TeleOpComponents.hardwareMap=hardwareMap;
        TeleOpComponents.telemetry=telemetry;
        //TeleOpComponents.drive = new PinpointDrive(hardwareMap,new Pose2d(0,0,Math.toRadians(90)));
        //initialize mechanism variables here
        extendo = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "extendo").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "extendo").getController(),
                hardwareMap.get(DcMotorEx.class, "extendo").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "extendo").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                800,0,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.ZeroPowerBehavior.FLOAT,
                "MOTION_PROFILE"
        );
        extendoPitch = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "extendoPitch").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "extendoPitch").getController(),
                hardwareMap.get(DcMotorEx.class, "extendoPitch").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "extendoPitch").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                0,-991,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.ZeroPowerBehavior.FLOAT,
                "MOTION_PROFILE"
        );
        bucketSlides = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "bucketSlides").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "bucketSlides").getController(),
                hardwareMap.get(DcMotorEx.class, "bucketSlides").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "bucketSlides").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                1030,0,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.ZeroPowerBehavior.FLOAT,
                "MOTION_PROFILE"
        );
        /*
        rightFront = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "rightFront").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getController(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                Double.POSITIVE_INFINITY,0,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "soogma"
        );
        rightBack = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "rightFront").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getController(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                Double.POSITIVE_INFINITY,0,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "soogma"
        );
        leftFront = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "rightFront").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getController(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                Double.POSITIVE_INFINITY,0,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "soogma"
        );
        leftBack = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "rightFront").getDeviceName(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getController(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "rightFront").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                Double.POSITIVE_INFINITY,0,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "soogma"
        );
        */
        clawFingers = new BotServo(
                hardwareMap.get(Servo.class, "clawFingers").getDeviceName(),
                hardwareMap.get(Servo.class, "clawFingers").getController(),
                hardwareMap.get(Servo.class, "clawFingers").getPortNumber(),
                new String[]{"closedPosition","openPosition"},
                new double[]{20,92},
                92,
                20,
                180,
                422,
                Servo.Direction.FORWARD
        );
        clawWrist = new BotServo(
                hardwareMap.get(Servo.class, "clawWrist").getDeviceName(),
                hardwareMap.get(Servo.class, "clawWrist").getController(),
                hardwareMap.get(Servo.class, "clawWrist").getPortNumber(),
                new String[]{"normalPosition"},
                new double[]{95},
                185,
                95,
                270,
                422,
                Servo.Direction.FORWARD
        );
        clawPitch = new BotServo(
                hardwareMap.get(Servo.class, "clawPitchLeft").getDeviceName(),
                hardwareMap.get(Servo.class, "clawPitchLeft").getController(),
                hardwareMap.get(Servo.class, "clawPitchLeft").getPortNumber(),
                new String[]{"pickUpPosition", "hoverPosition","transferPosition","backOffPosition"},
                new double[]{22,73,115,77.4},
                270,
                0,
                270,
                422,
                Servo.Direction.FORWARD
        );
        clawPitchRight = new BotServo(
                hardwareMap.get(Servo.class, "clawPitchRight").getDeviceName(),
                hardwareMap.get(Servo.class, "clawPitchRight").getController(),
                hardwareMap.get(Servo.class, "clawPitchRight").getPortNumber(),
                new String[]{"pickUpPosition", "hoverPosition","transferPosition","backOffPosition"},
                new double[]{22,73,115,77.4},
                270,
                0,
                270,
                422,
                Servo.Direction.REVERSE
        );
        innerClawPitch = new BotServo(
                hardwareMap.get(Servo.class, "innerClawPitch").getDeviceName(),
                hardwareMap.get(Servo.class, "innerClawPitch").getController(),
                hardwareMap.get(Servo.class, "innerClawPitch").getPortNumber(),
                new String[]{"pickUpPosition", "hoverPosition","transferPosition","backOffPosition"},
                new double[]{65,0,186,170.5},
                270,
                0,
                270,
                422,
                Servo.Direction.REVERSE
        );
        bucket = new BotServo(
                hardwareMap.get(Servo.class, "bucket").getDeviceName(),
                hardwareMap.get(Servo.class, "bucket").getController(),
                hardwareMap.get(Servo.class, "bucket").getPortNumber(),
                new String[]{"transferPosition","depositPosition"},
                new double[]{46,158},
                270,
                0,
                270,
                422,
                Servo.Direction.FORWARD
        );
        synchronizeServos(clawPitch,clawPitchRight);
    }
    public static void synchronizeServos(BotServo servo1, BotServo servo2){
        servo1.synchronizedServos.add(servo2);
    }
    public static void synchronizeMotors(BotMotor motor1, BotMotor motor2){
        motor1.synchronizedMotors.add(motor2);
    }
}

