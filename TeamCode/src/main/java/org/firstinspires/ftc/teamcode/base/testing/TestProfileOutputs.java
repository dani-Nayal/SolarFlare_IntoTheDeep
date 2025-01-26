package org.firstinspires.ftc.teamcode.base.testing;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.motorcontrol.TrapezoidalMotionProfile;

public class TestProfileOutputs {
    public static void main(String[] args){

        ElapsedTime timer = new ElapsedTime();
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile();
        double maxAccel = 2000;
        double maxVelocity = 1000;
        double initialVelocity = 0;
        int distance = 2000;
        int initialPosition = 0;
        double lastInstantTargetPosition = 0;
        profile.resetProfile(maxAccel, maxVelocity, initialVelocity, distance, initialPosition);

        timer.reset();

        while (profile.runProfile(timer.seconds()) != distance){
            System.out.println("------------------------------");

            System.out.println("position: " + profile.runProfile(timer.seconds()));
            System.out.println("time: " + timer.seconds());



            try {
                Thread.sleep(15);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }


            double instantTargetPosition = profile.runProfile(timer.seconds());

            double currentVelocity = (instantTargetPosition - lastInstantTargetPosition) / timer.seconds();
            System.out.println("speed: " + currentVelocity);

            lastInstantTargetPosition = instantTargetPosition;
        }

        System.out.println("accel time " + profile.accelerationTime);
        System.out.println("accel distance " + profile.accelerationDistance);
        System.out.println("cruise time " + profile.cruiseTime);
        System.out.println("cruise distance " + profile.cruiseDistance);
        System.out.println("decel time " + profile.decelerationTime);
        System.out.println("decel distance " + profile.decelerationDistance);
        System.out.println("max accel " + profile.maxAcceleration);
        System.out.println("max decel " + profile.maxDeceleration);
        System.out.println("total time " + profile.totalTime);
        System.out.println("distance " + profile.accelerationDistance + profile.cruiseDistance + profile.decelerationDistance);
        System.out.println("max velocity " + profile.maxVelocity);
        System.out.println("initial velocity " + profile.initialVelocity);
    }
}
