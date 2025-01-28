package org.firstinspires.ftc.teamcode.base.testing;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.motorcontrol.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.base.motorcontrol.TrapezoidalMotionProfile1D;

public class TestProfileOutputs {
    public static void main(String[] args){

        ElapsedTime timer = new ElapsedTime();
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile();
        double maxAccel = 2000;
        double maxDecel = 2000;
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
        System.out.println(profile.toString());

    }
}
