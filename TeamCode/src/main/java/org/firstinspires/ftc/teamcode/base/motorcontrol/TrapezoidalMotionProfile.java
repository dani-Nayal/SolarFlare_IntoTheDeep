package org.firstinspires.ftc.teamcode.base.motorcontrol;

public class TrapezoidalMotionProfile {
    double accelerationDistance;
    double accelerationTime;
    double cruiseDistance;
    double cruiseTime;
    double decelerationDistance;
    double decelerationTime;
    double totalTime;
    double maxAcceleration;
    double maxVelocity;
    double initialVelocity;
    double distance;
    public void resetProfile(double maxAcceleration, double maxVelocity, double initialVelocity, double distance){
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.initialVelocity = initialVelocity;
        this.distance = distance;
        accelerationTime = (maxVelocity - initialVelocity) / maxAcceleration;
        accelerationDistance = initialVelocity * accelerationTime + 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);

        decelerationTime = maxVelocity / maxAcceleration;
        decelerationDistance = maxVelocity * decelerationTime + 0.5 * maxAcceleration + Math.pow(decelerationTime, 2);

        cruiseDistance = distance - accelerationDistance - decelerationDistance;
        cruiseTime = cruiseDistance / maxVelocity;

        if (cruiseDistance <= 0){
            double exceededDistance = (accelerationDistance + decelerationDistance) - distance;

            accelerationDistance = initialVelocity * accelerationTime + 0.5 * maxAcceleration * Math.pow(accelerationTime, 2) - (exceededDistance / 2);
            decelerationDistance = maxVelocity * decelerationTime + 0.5 * maxAcceleration + Math.pow(decelerationTime, 2) - (exceededDistance / 2);

            accelerationTime = calculateKinematicTime(initialVelocity, maxAcceleration, accelerationDistance);

            maxVelocity = accelerationTime * maxAcceleration;

            decelerationTime = calculateKinematicTime(maxVelocity, maxAcceleration, distance);
        }
        totalTime = accelerationTime + cruiseTime + decelerationTime;
    }
    // Run this method in a loop
    public double runProfile(double elapsedTime){
        if (elapsedTime < accelerationTime) {
            return initialVelocity * elapsedTime + 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }
        else if (elapsedTime < (accelerationTime + cruiseTime)){
            double cruiseElapsedTime = elapsedTime - accelerationTime;
            return accelerationDistance + maxVelocity * cruiseElapsedTime;
        }
        else if (elapsedTime < totalTime){
            double decelerateElapsedTime = elapsedTime - accelerationTime - cruiseTime;
            return accelerationDistance + cruiseDistance + maxVelocity * decelerateElapsedTime - 0.5 * maxAcceleration * Math.pow(decelerateElapsedTime, 2);
        }
        else{
            return distance;
        }
    }
    public double calculateKinematicTime(double initialVelocity, double acceleration, double displacement){
        double firstRoot = (-initialVelocity + Math.sqrt(Math.pow(initialVelocity, 2) - acceleration + 2 * displacement)) / acceleration;
        double secondRoot = (-initialVelocity - Math.abs(Math.pow(initialVelocity, 2) - acceleration + 2 * displacement));
        return Math.max(firstRoot, secondRoot);
    }
}
