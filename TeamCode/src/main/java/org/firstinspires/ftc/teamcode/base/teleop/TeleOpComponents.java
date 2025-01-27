package org.firstinspires.ftc.teamcode.base.teleop;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImpl;
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
import org.firstinspires.ftc.teamcode.base.teleop.TeleOpActions.SemiUninterruptibleConditionalAction;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;


public abstract class  TeleOpComponents {
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;
    public static PinpointDrive drive;
    public static ElapsedTime LOOP_TIMER = null;
    public static ArrayList<BotMotor> motors = new ArrayList<>();
    public static ArrayList<BotMotor> motionProfileMotors = new ArrayList<>();
    public static ArrayList<BotServo> servos = new ArrayList<>();
    public static ArrayList<CRBotServo> CRServos = new ArrayList<>();

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

    public static CRBotServo hang;
    public static CRBotServo hangRight;
    public static BotMotor ryanNemesis;

    public static class BotMotor extends DcMotorImplEx {
        boolean isProfilePending = false; int profileDelayCounter = 1; int profileDelayFactor = 10;
        double maxVelocityParam;
        double maxAccelerationParam;
        double instantTargetPosition = 0;
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
        public ElapsedTime MOVEMENT_TIMER = null;
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
                return Math.abs(target-getCurrentPosition())>15;
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
                return Math.abs(target-getCurrentPosition())>15;
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
                return Math.abs(target-getCurrentPosition())>15;
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
                    new SemiUninterruptibleConditionalAction(new Condition[]{()->(target==target1),()->(target==target2)},new TeleOpAction[]{
                            new SetTargetAction(target2,maxAcceleration,maxVelocity),
                            new SetTargetAction(target1,maxAcceleration,maxVelocity)
                    })

            });
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2){
            return triggeredToggleAction(condition,target1,target2,MAX_ACCELERATION,MAX_VELOCITY);
        }
        public SemiUninterruptibleConditionalAction toggleAction(double target1, double target2, double maxAcceleration, double maxVelocity){
            return new SemiUninterruptibleConditionalAction(new Condition[]{()->(target==target1),()->(target==target2)},new TeleOpAction[]{
                    new SetTargetAction(target2,maxAcceleration,maxVelocity),
                    new SetTargetAction(target1,maxAcceleration,maxVelocity)
            });
        }
        public SemiUninterruptibleConditionalAction toggleAction(double target1, double target2){
            return toggleAction(target1,target2,MAX_ACCELERATION,MAX_VELOCITY);
        }
        public UpwardFSMAction upwardFSMAction(double maxAcceleration, double maxVelocity,double...positions){
            return new UpwardFSMAction(maxAcceleration, maxVelocity,positions);
        }
        public DownwardFSMAction downwardFSMAction(double maxAcceleration, double maxVelocity,double...positions){
            return new DownwardFSMAction(maxAcceleration, maxVelocity,positions);
        }
        public UpwardFSMAction upwardFSMAction(double...positions){
            return new UpwardFSMAction(positions);
        }
        public DownwardFSMAction downwardFSMAction(double...positions){
            return new DownwardFSMAction(positions);
        }
        public BotMotor(DcMotorController controller,
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
            super(controller, portNumber, Direction.FORWARD,motorType);

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
            if (Objects.equals(movementMode, "MOTION_PROFILE")){
                MOVEMENT_TIMER = new ElapsedTime();
            }

            hardwareMap.put(getDeviceName(),this);
            motors.add(this);
            if (Objects.equals(movementMode, "MOTION_PROFILE")) {
                motionProfileMotors.add(this);
            }
        }
        public void createMotionProfile(double max_velocity, double max_acceleration) {
            profileStartPos=getCurrentPosition();
            double distance=target-profileStartPos;
            if (distance!=0) {
                startVelocity = getVelocity();
                currentMaxVelocity = max_velocity * Math.signum(distance);
                currentMaxAcceleration = max_acceleration * Math.signum(currentMaxVelocity - startVelocity);
                currentMaxDeceleration = -max_acceleration * Math.signum(distance);

                accelDT = (currentMaxVelocity - startVelocity) / currentMaxAcceleration;
                decelDT = (0 - currentMaxVelocity) / currentMaxDeceleration;
                accelDistance = startVelocity * accelDT + 0.5 * currentMaxAcceleration * accelDT*accelDT;
                decelDistance = currentMaxVelocity * decelDT + 0.5 * currentMaxDeceleration * decelDT*decelDT;
                cruiseDistance = Math.abs(distance - accelDistance - decelDistance)*Math.signum(currentMaxVelocity);
                if (Math.abs(accelDistance + cruiseDistance+ decelDistance) > Math.abs(distance)) {
                    double halfExceededDistance = (distance - accelDistance - decelDistance) / 2;
                    accelDistance = accelDistance + halfExceededDistance;
                    accelDT = Math.max(
                            (-startVelocity + Math.sqrt(Math.abs(startVelocity*startVelocity + 2 * currentMaxAcceleration * accelDistance))) / (currentMaxAcceleration),
                            (-startVelocity - Math.sqrt(Math.abs(startVelocity*startVelocity + 2 * currentMaxAcceleration * accelDistance))) / (currentMaxAcceleration)
                    );
                    currentMaxVelocity = currentMaxAcceleration * accelDT + startVelocity;
                    decelDistance = decelDistance + halfExceededDistance;
                    decelDT = Math.max(
                            (-currentMaxVelocity + Math.sqrt(Math.abs(currentMaxVelocity*currentMaxVelocity + 2 * currentMaxDeceleration * decelDistance))) / (currentMaxDeceleration),
                            (-currentMaxVelocity - Math.sqrt(Math.abs(currentMaxVelocity*currentMaxVelocity + 2 * currentMaxDeceleration * decelDistance))) / (currentMaxDeceleration)
                    );
                }
                cruiseDistance = Math.abs(distance - accelDistance - decelDistance)*Math.signum(currentMaxVelocity);
                cruiseDT = cruiseDistance / currentMaxVelocity;
                if (Double.isNaN(accelDT) || Double.isNaN(accelDistance) || Double.isNaN(decelDT) || Double.isNaN(decelDistance) || Double.isNaN(cruiseDT) || Double.isNaN(cruiseDistance) || accelDT<0 || decelDT < 0 || cruiseDT<0){
                    accelDT=0;
                    cruiseDT=0;
                    decelDT=0;
                    accelDistance=0;
                    cruiseDistance=0;
                    decelDistance=0;
                    telemetry.addData("e","e");
                    telemetry.update();
                }
            }
            else{
                accelDT=0;
                cruiseDT=0;
                decelDT=0;
                accelDistance=0;
                cruiseDistance=0;
                decelDistance=0;
            }
        }
        public void runMotionProfileOnce(){
            double elapsedTime = MOVEMENT_TIMER.time();
            if (elapsedTime < accelDT){
                instantTargetPosition=profileStartPos + startVelocity * elapsedTime + 0.5 * currentMaxAcceleration * elapsedTime*elapsedTime;
            }
            else if (elapsedTime < accelDT+cruiseDT){
                double cruiseCurrentDT = elapsedTime - accelDT;
                instantTargetPosition = profileStartPos + accelDistance + currentMaxVelocity * cruiseCurrentDT;
            }

            else if (elapsedTime < accelDT+cruiseDT+decelDT){
                double decelCurrentDT = elapsedTime - accelDT - cruiseDT;
                instantTargetPosition = profileStartPos + accelDistance + cruiseDistance + currentMaxVelocity * decelCurrentDT + 0.5 * currentMaxDeceleration * decelCurrentDT*decelCurrentDT;
            }
            else if (elapsedTime >= accelDT+decelDT+cruiseDT){
                instantTargetPosition=target;
            }
            double error=instantTargetPosition-getCurrentPosition();
            double kpPower = kP*error;
            integralSum += LOOP_TIMER.time()*error;
            double kiPower = kI*integralSum;
            double kdPower = kD*(error-previousError)/LOOP_TIMER.time();
            double outPower = Math.min(1,Math.max(-1,kpPower+kiPower+kdPower));
            setPower(outPower);
            previousError=error;
        }
        public double getPos(String key){
            return KEY_POSITIONS.get(key);
        }

        public void setMotorTarget(double target, double maxVelocity, double maxAcceleration){
            target = Math.min(MAX_POSITION, Math.max(MIN_POSITION, target));
            if (target!=this.target || maxVelocity != currentMaxVelocity || maxAcceleration != currentMaxAcceleration) {
                MOVEMENT_TIMER.reset();
                this.target = target;
                integralSum = 0;
                previousError = 0;
                isProfilePending=true; maxAccelerationParam=maxAcceleration; maxVelocityParam=maxVelocity;
                for (BotMotor motor : synchronizedMotors){
                    motor.setMotorTarget(target,maxVelocity,maxAcceleration);
                }
            }
        }
        public void createPendingMotionProfiles(){
            if (isProfilePending) {
                if (profileDelayCounter==1){
                    createMotionProfile(maxVelocityParam, maxAccelerationParam);
                    isProfilePending=false;
                }
                if (profileDelayCounter<profileDelayFactor) profileDelayCounter++; else profileDelayCounter=1;
            }
            else profileDelayCounter=1;
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
        public double startPos = -1;
        public double time;
        public double currPos;
        public double offset = 0;
        public BotServo(ServoController controller,
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

            hardwareMap.put(getDeviceName(),this);
            servos.add(this);
        }
        public void setPositionWithDelay(double position){
            if (MOVEMENT_TIMER == null){
                MOVEMENT_TIMER = new ElapsedTime();
                startPos=0;
            }
            else {
                if (startPos == -1){
                    startPos=getPosition();
                }
                else{
                    startPos = Math.signum(getPosition() - startPos) * SERVO_SPEED * Math.min(time, MOVEMENT_TIMER.time()) + startPos;
                }
            }
            setPosition(position);
            time=Math.abs(getPosition()-startPos)/SERVO_SPEED+0.07;
            MOVEMENT_TIMER.reset();
        }
        public void changeOffset(double amount){
            double position = getPosition();
            if (MOVEMENT_TIMER == null){
                MOVEMENT_TIMER = new ElapsedTime();
                startPos=0;
            }
            else {
                if (startPos == -1){
                    startPos=getPosition();
                }
                else{
                    startPos = Math.signum(getPosition() - startPos) * SERVO_SPEED * Math.min(time, MOVEMENT_TIMER.time()) + startPos;
                }
            }
            offset+=amount;
            for (BotServo servo : synchronizedServos){
                servo.offset+=amount;
            }
            setPosition(position);
            time=Math.abs(getPosition()-startPos)/SERVO_SPEED+0.07;
            MOVEMENT_TIMER.reset();
        }
        @Override
        public void setPosition(double position){
            currPos=Math.max(MINIMUM_POSITION,Math.min(MAXIMUM_POSITION,position+offset))-offset;
            super.setPosition((currPos+offset)/RANGE);
            for (BotServo servo : synchronizedServos){
                servo.setPosition(position);
            }
        }
        @Override
        public double getPosition(){
            return currPos+offset;
        }
        public double getPos(String key){
            return KEY_POSITIONS.get(key);
        }
        public class ChangeOffsetAction implements TeleOpAction {
            boolean isStart = true;
            double amount;
            public ChangeOffsetAction(double amount){
                this.amount = amount;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isStart) {
                    isStart=false;
                    changeOffset(offset);

                }
                if (MOVEMENT_TIMER.time() < time){
                    return true;
                }
                else{
                    startPos=-1;
                    return false;
                }
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                isStart=true;
                return run(packet);
            }

            @Override
            public void stop() {}
        }
        public ChangeOffsetAction changeOffsetAction(double amount){
            return new ChangeOffsetAction(amount);
        }
        public ConditionalAction triggeredChangeOffsetAction(Condition upCondition, Condition downCondition, double amount){
            return new ConditionalAction(new Condition[]{upCondition,downCondition}, new TeleOpAction[]{new ChangeOffsetAction(amount),new ChangeOffsetAction(-amount)});
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
                    setPositionWithDelay(posFun.call());

                }
                if (MOVEMENT_TIMER.time() < time){
                    return true;
                }
                else{
                    startPos=-1;
                    return false;
                }
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
                    setPositionWithDelay(pos);
                }
                if (MOVEMENT_TIMER.time() < time){
                    return true;
                }
                else{
                    startPos=-1;
                    return false;
                }
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
                    setPositionWithDelay(pos);
                }
                if (MOVEMENT_TIMER.time() < time){
                    return true;
                }
                else{
                    startPos=-1;
                    return false;
                }
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
                    new SemiUninterruptibleConditionalAction(new Condition[]{()->(getPosition()==target1),()->(getPosition()==target2)},new TeleOpAction[]{
                            new SetPositionAction(target2),
                            new SetPositionAction(target1)
                    })

            });
        }
        public SemiUninterruptibleConditionalAction toggleAction(double target1, double target2){
            return new SemiUninterruptibleConditionalAction(new Condition[]{()->(getPosition()==target1),()->(getPosition()==target2)},new TeleOpAction[]{
                    new SetPositionAction(target2),
                    new SetPositionAction(target1)
            });
        }
        public UpwardFSMAction upwardFSMAction(double...positions){
            return new UpwardFSMAction(positions);
        }
        public DownwardFSMAction downwardFSMAction(double...positions){
            return new DownwardFSMAction(positions);
        }
    }
    public static class CRBotServo extends CRServoImpl {
        public ArrayList<CRBotServo> synchronizedServos = new ArrayList<>();
        public double SERVO_SPEED;
        public CRBotServo(ServoController controller,
                        int portNumber,
                        double servoSpeed,
                        Direction direction)
        {
            super(controller, portNumber);
            this.SERVO_SPEED = servoSpeed;
            setDirection(direction);
            hardwareMap.put(getDeviceName(),this);
            CRServos.add(this);
        }
        @Override
        public void setPower(double power){
            super.setPower(power);
            for (CRBotServo servo : synchronizedServos){
                servo.setPower(power);
            }
        }
        public class SetPowerAction implements TeleOpAction{
            public DoubleFunction powerFun;
            public SetPowerAction(double power){
                this.powerFun =()->(power);
            }
            public SetPowerAction(DoubleFunction powerFun){
                this.powerFun = powerFun;
            }
            @Override
            public boolean repeatFromStart(@NonNull TelemetryPacket packet) {
                return run(packet);
            }

            @Override
            public void stop() {}

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setPower(powerFun.call());
                return false;
            }
        }
        public SetPowerAction setPowerAction(double power){
            return new SetPowerAction(power);
        }

    }
    public static void initializeMechanisms(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialDrivePose){
        TeleOpComponents.hardwareMap=hardwareMap;
        TeleOpComponents.telemetry=telemetry;
        /*
        ryanNemesis = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "motor-1").getController(),
                hardwareMap.get(DcMotorEx.class, "motor-1").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "motor-1").getMotorType(),
                0.015,0,0.00002,
                new String[]{},new double[]{},
                Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY,
                250000,3500,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "MOTION_PROFILE"
        );
        */
        TeleOpComponents.drive = new PinpointDrive(hardwareMap,initialDrivePose);
        //initialize mechanism variables here
        extendo = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "extendo").getController(),
                hardwareMap.get(DcMotorEx.class, "extendo").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "extendo").getMotorType(),
                0.015,0,0.0,
                new String[]{},new double[]{},
                793,0,
                250000,3500,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "MOTION_PROFILE"
        );
        extendoPitch = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "extendoPitch").getController(),
                hardwareMap.get(DcMotorEx.class, "extendoPitch").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "extendoPitch").getMotorType(),
                0.005,0,0.0,
                new String[]{"transferPosition","pickUpPosition","specimenGrabPosition","specimenDepositPosition"},
                new double[]{0,-960,-960,0},
                0,-960,
                250000,3500,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "MOTION_PROFILE"
        );
        bucketSlides = new BotMotor(
                hardwareMap.get(DcMotorEx.class, "bucketSlides").getController(),
                hardwareMap.get(DcMotorEx.class, "bucketSlides").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "bucketSlides").getMotorType(),
                0.015,0,0.0,
                new String[]{"depositPosition","transferPosition"},new double[]{1060,0},
                1060,0,
                250000,3500,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "MOTION_PROFILE"
        );
        rightFront = new BotMotor(
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
                hardwareMap.get(DcMotorEx.class, "rightBack").getController(),
                hardwareMap.get(DcMotorEx.class, "rightBack").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "rightBack").getMotorType(),
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
                hardwareMap.get(DcMotorEx.class, "leftFront").getController(),
                hardwareMap.get(DcMotorEx.class, "leftFront").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "leftFront").getMotorType(),
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
                hardwareMap.get(DcMotorEx.class, "leftBack").getController(),
                hardwareMap.get(DcMotorEx.class, "leftBack").getPortNumber(),
                hardwareMap.get(DcMotorEx.class, "leftBack").getMotorType(),
                0.015,0,0,
                new String[]{},new double[]{},
                Double.POSITIVE_INFINITY,0,
                200000,3000,
                DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.ZeroPowerBehavior.BRAKE,
                "soogma"
        );
        clawFingers = new BotServo(
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
                hardwareMap.get(Servo.class, "clawWrist").getController(),
                hardwareMap.get(Servo.class, "clawWrist").getPortNumber(),
                new String[]{"normalPosition"},
                new double[]{95},
                185,
                5,
                270,
                422,
                Servo.Direction.FORWARD
        );
        clawPitch = new BotServo(
                hardwareMap.get(Servo.class, "clawPitchLeft").getController(),
                hardwareMap.get(Servo.class, "clawPitchLeft").getPortNumber(),
                new String[]{"pickUpPosition", "hoverPosition","transferPosition","backOffPosition","specimenGrabPosition","specimenDepositPosition"},
                new double[]{13,68,100,72.4,151.011,13},
                270,
                0,
                270,
                422,
                Servo.Direction.FORWARD
        );
        clawPitchRight = new BotServo(
                hardwareMap.get(Servo.class, "clawPitchRight").getController(),
                hardwareMap.get(Servo.class, "clawPitchRight").getPortNumber(),
                new String[]{"pickUpPosition", "hoverPosition","transferPosition","backOffPosition","specimenGrabPosition","specimenDepositPosition"},
                new double[]{13,68,100,72.4,151.011,13},
                270,
                0,
                270,
                422,
                Servo.Direction.REVERSE
        );
        innerClawPitch = new BotServo(
                hardwareMap.get(Servo.class, "innerClawPitch").getController(),
                hardwareMap.get(Servo.class, "innerClawPitch").getPortNumber(),
                new String[]{"pickUpPosition", "hoverPosition","transferPosition","backOffPosition","specimenGrabPosition","specimenDepositPosition"},
                new double[]{82,20,200,160.5,73,82},
                270,
                0,
                270,
                422,
                Servo.Direction.REVERSE
        );
        bucket = new BotServo(
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
    public static void synchronizeServos(CRBotServo servo1, CRBotServo servo2){
        servo1.synchronizedServos.add(servo2);
    }
    public static void synchronizeMotors(BotMotor motor1, BotMotor motor2){
        motor1.synchronizedMotors.add(motor2);
    }
}

