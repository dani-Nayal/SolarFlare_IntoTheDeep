package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.MotorEnum;
import org.firstinspires.ftc.teamcode.ServoEnum;
import org.firstinspires.ftc.teamcode.HardwareConfig;

import java.util.HashMap;

public class RobotState {
    HashMap<MotorEnum, Integer> targets = new HashMap<>(10);
    HashMap<ServoEnum, Double> positions = new HashMap<>(10);
    HashMap<MotorEnum, Integer> autoDefaultMotorTargets = new HashMap<>(10);
    HashMap<ServoEnum, Double> autoDefaultServoPositions = new HashMap<>(10);
    HashMap<MotorEnum, Integer> teleOpDefaultMotorTargets = new HashMap<>(10);
    HashMap<ServoEnum, Double> teleOpDefaultServoPositions = new HashMap<>(10);

    public void setTeleOpDefaultState(){
        teleOpDefaultMotorTargets.put(MotorEnum.EXTENDO, 0);
        teleOpDefaultMotorTargets.put(MotorEnum.EXTENDO_PITCH, 0);
        teleOpDefaultMotorTargets.put(MotorEnum.HANG, 0);
        teleOpDefaultMotorTargets.put(MotorEnum.BUCKET_SLIDES, 0);

        teleOpDefaultServoPositions.put(ServoEnum.CLAW_PITCH_LEFT, 215.0);
        teleOpDefaultServoPositions.put(ServoEnum.CLAW_PITCH_RIGHT, 215.0);
        teleOpDefaultServoPositions.put(ServoEnum.CLAW_FINGERS, 120.0);
        teleOpDefaultServoPositions.put(ServoEnum.CLAW_WRIST, 76.5);
        teleOpDefaultServoPositions.put(ServoEnum.BUCKET, 85.0);

        targets.put(MotorEnum.EXTENDO, teleOpDefaultMotorTargets.get(MotorEnum.EXTENDO));
        targets.put(MotorEnum.EXTENDO_PITCH, teleOpDefaultMotorTargets.get(MotorEnum.EXTENDO_PITCH));
        targets.put(MotorEnum.HANG, teleOpDefaultMotorTargets.get(MotorEnum.HANG));
        targets.put(MotorEnum.BUCKET_SLIDES, teleOpDefaultMotorTargets.get(MotorEnum.BUCKET_SLIDES));

        positions.put(ServoEnum.CLAW_PITCH_LEFT, teleOpDefaultServoPositions.get(ServoEnum.CLAW_PITCH_LEFT));
        positions.put(ServoEnum.CLAW_PITCH_RIGHT, teleOpDefaultServoPositions.get(ServoEnum.CLAW_PITCH_RIGHT));
        positions.put(ServoEnum.CLAW_FINGERS, teleOpDefaultServoPositions.get(ServoEnum.CLAW_FINGERS));
        positions.put(ServoEnum.CLAW_WRIST, teleOpDefaultServoPositions.get(ServoEnum.CLAW_WRIST));
        positions.put(ServoEnum.BUCKET, teleOpDefaultServoPositions.get(ServoEnum.BUCKET));
    }
    public void setAutoDefaultState(){
        autoDefaultMotorTargets.put(MotorEnum.EXTENDO, 0);
        autoDefaultMotorTargets.put(MotorEnum.EXTENDO_PITCH, 0);
        autoDefaultMotorTargets.put(MotorEnum.HANG, 0);
        autoDefaultMotorTargets.put(MotorEnum.BUCKET_SLIDES, 0);

        autoDefaultServoPositions.put(ServoEnum.CLAW_PITCH_LEFT, 200.0);
        autoDefaultServoPositions.put(ServoEnum.CLAW_PITCH_RIGHT, 200.0);
        autoDefaultServoPositions.put(ServoEnum.CLAW_FINGERS, 39.0);
        autoDefaultServoPositions.put(ServoEnum.CLAW_WRIST, 76.5);
        autoDefaultServoPositions.put(ServoEnum.BUCKET, 85.0);

        targets.put(MotorEnum.EXTENDO, autoDefaultMotorTargets.get(MotorEnum.EXTENDO));
        targets.put(MotorEnum.EXTENDO_PITCH, autoDefaultMotorTargets.get(MotorEnum.EXTENDO_PITCH));
        targets.put(MotorEnum.HANG, autoDefaultMotorTargets.get(MotorEnum.HANG));
        targets.put(MotorEnum.BUCKET_SLIDES, autoDefaultMotorTargets.get(MotorEnum.BUCKET_SLIDES));

        positions.put(ServoEnum.CLAW_PITCH_LEFT, autoDefaultServoPositions.get(ServoEnum.CLAW_PITCH_LEFT));
        positions.put(ServoEnum.CLAW_PITCH_RIGHT, autoDefaultServoPositions.get(ServoEnum.CLAW_PITCH_RIGHT));
        positions.put(ServoEnum.CLAW_FINGERS, autoDefaultServoPositions.get(ServoEnum.CLAW_FINGERS));
        positions.put(ServoEnum.CLAW_WRIST, autoDefaultServoPositions.get(ServoEnum.CLAW_WRIST));
        positions.put(ServoEnum.BUCKET, autoDefaultServoPositions.get(ServoEnum.BUCKET));
    }
    public int getMotorTarget(MotorEnum motorEnum) throws IllegalArgumentException{
        Integer target = targets.get(motorEnum);
        if (target == null) {
            throw new IllegalArgumentException("No target set for " + motorEnum.name());
        }
        return target;
    }
    public void setMotorTarget(MotorEnum motorEnum, int target){
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
        HardwareConfig hw = HardwareConfig.getHardwareConfig();

       // position = Math.max(position, hw.getServoConfig(servoEnum).minServoPosition);
      //  position = Math.min(position, hw.getServoConfig(servoEnum).maxServoPosition);

        positions.put(servoEnum, position);

    }
}
