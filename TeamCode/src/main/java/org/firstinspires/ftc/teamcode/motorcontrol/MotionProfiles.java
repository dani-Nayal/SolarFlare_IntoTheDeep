package org.firstinspires.ftc.teamcode.motorcontrol;

public class MotionProfiles {
    public double runTrapezoidalMotionProfile(double maxMotorVelocity, double maxMotorAcceleration, double distance, double elapsedTime){
        double accelerationDT = maxMotorVelocity / maxMotorAcceleration;
        double halfWayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxMotorAcceleration * Math.pow(accelerationDT, 2);

        if (Math.abs(accelerationDistance) > Math.abs(halfWayDistance)){
            accelerationDT = Math.sqrt(halfWayDistance / (0.5 * maxMotorAcceleration));
        }
        accelerationDistance = 0.5 * maxMotorAcceleration * Math.pow(accelerationDT, 2);

        maxMotorVelocity = maxMotorAcceleration * accelerationDT;

        double decelerationDT = accelerationDT;

        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDT = cruiseDistance / maxMotorVelocity;
        double accelerateAndCruiseDT = accelerationDT + cruiseDT;

        double entireDT = accelerationDT + cruiseDT + decelerationDT;
        if (elapsedTime > entireDT){
            return distance;
        }

        if (elapsedTime < accelerationDT){
            return 0.5 * maxMotorAcceleration * Math.pow(elapsedTime, 2);
        }

        else if (elapsedTime < accelerateAndCruiseDT){
            accelerationDistance = 0.5 * maxMotorAcceleration * Math.pow(accelerationDT, 2);
            double cruiseCurrentDT = elapsedTime - accelerationDT;
            return accelerationDistance + maxMotorVelocity * cruiseCurrentDT;
        }

        else {
            accelerationDistance = 0.5 * maxMotorAcceleration * Math.pow(accelerationDT, 2);
            cruiseDistance = maxMotorVelocity * cruiseDT;
            return accelerationDistance + cruiseDistance + maxMotorVelocity * decelerationDT - 0.5 * maxMotorAcceleration * Math.pow(decelerationDT, 2);
        }
    }
}
