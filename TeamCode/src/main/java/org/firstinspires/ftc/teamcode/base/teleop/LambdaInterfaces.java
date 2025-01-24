package org.firstinspires.ftc.teamcode.base.teleop;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

public abstract class LambdaInterfaces {
    public interface ShortFunction{
       void call();
    }
    public interface Condition{
        boolean call();
    }
    public interface DoubleFunction{
        double call();
    }
    public interface Vector2dFunction{
        Vector2d call();
    }
    public interface RoadrunnerFunction {}
    public interface StrafeToLinearHeading extends RoadrunnerFunction {
        TrajectoryActionBuilder call(Vector2d vector, double heading);
    }
    public interface StrafeAndTurn extends RoadrunnerFunction {
        TrajectoryActionBuilder call(Vector2d vector, double heading);
    }
    public interface WaitSeconds extends RoadrunnerFunction {
        TrajectoryActionBuilder call(double time);
    }
    public interface TurnTo extends RoadrunnerFunction {
        TrajectoryActionBuilder call(double heading);
    }
    public interface Turn extends RoadrunnerFunction {
        TrajectoryActionBuilder call(double headingChange);
    }
}