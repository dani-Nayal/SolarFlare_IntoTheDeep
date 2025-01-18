package org.firstinspires.ftc.teamcode.base.motorcontrol;

public class OldTrapezoidalMotionProfile {
    public double runProfile(double maxVelocity, double maxAcceleration, double distance, double elapsedTime, int initialPosition) {
        double accelerationTime = maxVelocity / maxAcceleration;
        double decelerationTime = accelerationTime;

        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
        double decelerationDistance = accelerationDistance;

        double halfWayDistance = distance / 2;

        if (Math.abs(accelerationDistance) > Math.abs(halfWayDistance)) {
            accelerationTime = Math.sqrt(halfWayDistance / (0.5 * maxAcceleration));
            decelerationTime = accelerationTime;
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
            decelerationDistance = accelerationDistance;
            maxVelocity = maxAcceleration * accelerationTime;
        }

        double cruiseDistance = distance - accelerationDistance - decelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;

        double entireTime = accelerationTime + cruiseTime + decelerationTime;

        if (elapsedTime > entireTime){
            return initialPosition + distance;
        }
        else if (elapsedTime < accelerationTime){
            return initialPosition + 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }
        else if (elapsedTime < accelerationTime + cruiseTime){
            double cruiseCurrentTime = elapsedTime - accelerationTime;
            return initialPosition + accelerationDistance + maxVelocity * cruiseCurrentTime;
        }
        else {
            double currentDecelerateTime = elapsedTime - accelerationTime - cruiseTime;
            return initialPosition + accelerationDistance + cruiseDistance + maxVelocity * currentDecelerateTime - 0.5 * maxAcceleration * Math.pow(currentDecelerateTime, 2);
        }
    }
}