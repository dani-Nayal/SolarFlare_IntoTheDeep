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
    double maxDeceleration;
    double maxVelocity;
    double initialVelocity;
    double distance;
    int initialPosition;
    public void resetProfile(double maxAcceleration, double maxVelocity, double initialVelocity, double distance, int initialPosition){
        this.maxVelocity = maxVelocity * Math.signum(distance);
        this.initialVelocity = initialVelocity;
        this.distance = distance;
        this.initialPosition = initialPosition;
        this.maxAcceleration = maxAcceleration * Math.signum(maxVelocity - initialVelocity);
        this.maxDeceleration = -maxAcceleration * Math.signum(distance);


        accelerationTime = Math.abs((this.maxVelocity - this.initialVelocity) / this.maxAcceleration);
        accelerationDistance = this.initialVelocity * accelerationTime + 0.5 * this.maxAcceleration * Math.pow(accelerationTime, 2);

        decelerationTime = Math.abs(this.maxVelocity / maxDeceleration);
        decelerationDistance = this.maxVelocity * decelerationTime + 0.5 * maxDeceleration * Math.pow(decelerationTime, 2);

        cruiseDistance = this.distance - accelerationDistance - decelerationDistance;
        cruiseTime = Math.abs(cruiseDistance / this.maxVelocity);

        if (accelerationDistance + decelerationDistance > this.distance){
            double exceededDistance = (accelerationDistance + decelerationDistance) - this.distance;

            accelerationDistance -= (Math.abs(accelerationDistance) / (Math.abs(accelerationDistance) + Math.abs(decelerationDistance))) * exceededDistance;
            decelerationDistance -= (Math.abs(decelerationDistance) / (Math.abs(accelerationDistance) + Math.abs(decelerationDistance))) * exceededDistance;

            accelerationTime = Math.abs(calculateKinematicTime(this.initialVelocity, this.maxAcceleration, accelerationDistance));

            this.maxVelocity = this.initialVelocity + accelerationTime * this.maxAcceleration;

            decelerationTime = Math.abs(calculateKinematicTime(this.maxVelocity, this.maxDeceleration, decelerationDistance));

            cruiseDistance = this.distance - accelerationDistance - decelerationDistance;
            cruiseTime = Math.abs(cruiseDistance / this.maxVelocity);
        }

        totalTime = Math.max(0, accelerationTime) + Math.max(0, cruiseTime) + Math.max(0, decelerationTime);
    }
    // Run this method in a loop
    public double runProfile(double elapsedTime){
        if (elapsedTime < accelerationTime) {
            return initialPosition + initialVelocity * elapsedTime + 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }
        else if (elapsedTime < (accelerationTime + cruiseTime)){
            double cruiseElapsedTime = elapsedTime - accelerationTime;
            return initialPosition + accelerationDistance + maxVelocity * cruiseElapsedTime;
        }
        else if (elapsedTime < totalTime){
            double decelerateElapsedTime = elapsedTime - accelerationTime - cruiseTime;
            return initialPosition + accelerationDistance + cruiseDistance + maxVelocity * decelerateElapsedTime + 0.5 * maxDeceleration * Math.pow(decelerateElapsedTime, 2);
        }
        else {
            return initialPosition + distance;
        }
    }
    public double calculateKinematicTime(double initialVelocity, double acceleration, double displacement){
        double firstRoot = (-initialVelocity + Math.sqrt(Math.abs(Math.pow(initialVelocity, 2) + 2 * displacement * acceleration))) / acceleration;
        double secondRoot = (-initialVelocity - Math.sqrt(Math.abs(Math.pow(initialVelocity, 2) +  2 * displacement * acceleration))) / acceleration;
        return Math.max(firstRoot, secondRoot);
    }
}
