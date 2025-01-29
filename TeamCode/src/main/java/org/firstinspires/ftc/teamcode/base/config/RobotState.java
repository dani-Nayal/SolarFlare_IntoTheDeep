package org.firstinspires.ftc.teamcode.base.config;

import java.util.HashMap;

public class RobotState {
    private static RobotState state;
    HardwareConfig hw;
    HashMap<MotorEnum, Integer> targets = new HashMap<>(10);
    HashMap<ServoEnum, Double> positions = new HashMap<>(10);
    public int EXTENDO_PITCH_TRANSFER = 0;
    public int EXTENDO_PITCH_PICK_UP = -960;
    public int BUCKET_SLIDES_HIGH_BUCKET = 1030;
    public int BUCKET_SLIDES_TRANSFER = 0;
    public int CLAW_FINGERS_OPEN = 20;
    public int CLAW_FINGERS_CLOSED = 92;
    public int CLAW_WRIST_DEFAULT = 95;
    public int CLAW_PITCH_PICK_UP = 22;
    public int CLAW_PITCH_HOVER = 73;
    public int CLAW_PITCH_TRANSFER = 115;
    public int CLAW_PITCH_BACK_OFF = 77;
    public int INNER_CLAW_PITCH_PICK_UP = 62;
    public int INNER_CLAW_PITCH_HOVER = 0;
    public int INNER_CLAW_PITCH_TRANSFER = 186;
    public int INNER_CLAW_PITCH_BACK_OFF = 170;
    public int BUCKET_TRANSFER = 46;
    public int BUCKET_DEPOSIT = 158;

    // Specimen stuff

    public int EXTENDO_SCORE_SPECIMEN = 800;
    public int EXTENDO_PITCH_SCORE_SPECIMEN = -620;
    public int CLAW_PITCH_SCORE_SPECIMEN = 127;
    public int INNER_CLAW_PITCH_SCORE_SPECIMEN = 179;

    public int EXTENDO_RETRACTED = 0;
    public int EXTENDO_PITCH_GRAB_SPECIMEN = -960;
    public int CLAW_PITCH_GRAB_SPECIMEN = 151;
    public int INNER_CLAW_PITCH_GRAB_SPECIMEN = 73;

    public static RobotState getInstance(){
        if (state == null){
            state = new RobotState();
        }
        return state;
    }
    private RobotState(){
        hw = HardwareConfig.getInstance();
        targets.put(MotorEnum.TESTING_MOTOR, 0);

        /*
        targets.put(MotorEnum.EXTENDO, 0);
        targets.put(MotorEnum.EXTENDO_PITCH, 0);
        targets.put(MotorEnum.BUCKET_SLIDES, 0);

        positions.put(ServoEnum.CLAW_PITCH_LEFT, 115);
        positions.put(ServoEnum.CLAW_PITCH_RIGHT, 115);
        positions.put(ServoEnum.CLAW_WRIST, 95);
        positions.put(ServoEnum.BUCKET, 46);
        positions.put(ServoEnum.INNER_CLAW_PITCH, 186);
        positions.put(ServoEnum.CLAW_FINGERS, 20);
         */
    }
    public int getMotorTarget(MotorEnum motorEnum) throws IllegalArgumentException{
        Integer target = targets.get(motorEnum);
        if (target == null) {
            throw new IllegalArgumentException("No target set for " + motorEnum.name());
        }
        return target;
    }
    public void setMotorTarget(MotorEnum motorEnum, int target){
        target = Math.max(target, hw.getMotorConfig(motorEnum).minTarget);
        target = Math.min(target, hw.getMotorConfig(motorEnum).maxTarget);

        targets.put(motorEnum, target);
    }
    public double getServoPosition(ServoEnum servoEnum) throws IllegalArgumentException{
        Double position = positions.get(servoEnum);
        if (position == null){
            throw new IllegalArgumentException("No position set for" + servoEnum.name());
        }
        return position;
    }
    public void setServoPosition(ServoEnum servoEnum, double position){
        position = Math.max(position, hw.getServoConfig(servoEnum).minServoPosition);
        position = Math.min(position, hw.getServoConfig(servoEnum).maxServoPosition);

        positions.put(servoEnum, position);
    }
}
