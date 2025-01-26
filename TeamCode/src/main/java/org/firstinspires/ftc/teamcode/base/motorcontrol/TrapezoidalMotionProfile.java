package org.firstinspires.ftc.teamcode.base.motorcontrol;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TrapezoidalMotionProfile {
    public double accelerationDistance;
    public double accelerationTime;
    public double cruiseDistance;
    public double cruiseTime;
    public double decelerationDistance;
    public double decelerationTime;
    public double totalTime;
    public double maxAcceleration;
    public double maxDeceleration;
    public double maxVelocity;
    public double initialVelocity;
    public int distance;
    int initialPosition;
    public void resetProfile(double maxAcceleration, double maxVelocity, double initialVelocity, int distance, int initialPosition){
        this.maxVelocity = maxVelocity * Math.signum(distance);
        this.initialVelocity = initialVelocity;
        this.distance = distance;
        this.initialPosition = initialPosition;
        this.maxAcceleration = maxAcceleration * Math.signum(maxVelocity - initialVelocity);
        this.maxDeceleration = Math.signum(distance) * -maxAcceleration;


        accelerationTime = (this.maxVelocity - this.initialVelocity) / this.maxAcceleration;
        accelerationDistance = this.initialVelocity * accelerationTime + 0.5 * this.maxAcceleration * Math.pow(accelerationTime, 2);

        decelerationTime = (0- this.maxVelocity) / maxDeceleration;
        decelerationDistance = this.maxVelocity * decelerationTime + 0.5 * maxDeceleration * Math.pow(decelerationTime, 2);

        cruiseDistance = this.distance - accelerationDistance - decelerationDistance;
        cruiseTime = Math.abs(cruiseDistance / this.maxVelocity);

        if (Math.abs(accelerationDistance) + Math.abs(decelerationDistance) > Math.abs(this.distance)){
            double exceededDistance = (accelerationDistance + decelerationDistance) - this.distance;

            accelerationDistance -= exceededDistance / 2;
            decelerationDistance -= exceededDistance / 2;

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
