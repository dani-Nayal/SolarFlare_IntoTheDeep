package org.firstinspires.ftc.teamcode.base.config;

import java.util.HashMap;

public class RobotState {
    private static RobotState state;
    HardwareConfig hw;
    HashMap<MotorEnum, Integer> targets = new HashMap<>(10);
    HashMap<ServoEnum, Double> positions = new HashMap<>(10);

    // TODO: Store preset mechanism position values here

    public static RobotState getInstance(){
        if (state == null){
            state = new RobotState();
        }
        return state;
    }
    private RobotState(){
        hw = HardwareConfig.getInstance();
        // TODO: make sure to add default positions
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
