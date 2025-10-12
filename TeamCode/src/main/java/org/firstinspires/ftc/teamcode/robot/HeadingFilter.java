package org.firstinspires.ftc.teamcode.robot;

// TODO: Try to implement Lynx BulkCache system for faster loops

public class HeadingFilter {
    private double Q; // Process noise covariance
    private double R; // Measurement noise covariance
    private double heading;
    private double covariance;

    public HeadingFilter(double initialHeading, double processNoise, double measurementNoise) {
        this.heading = initialHeading;
        this.covariance = 1.0; // Initial covariance
        this.Q = processNoise;
        this.R = measurementNoise;
    }

    public void update(double measurement) {
        // Prediction step
        double predictedHeading = heading;
        double predictedCovariance = covariance + Q;

        // Correction step (using the measurement)
        double kalmanGain = predictedCovariance / (predictedCovariance + R);
        heading = predictedHeading + kalmanGain * (measurement - predictedHeading);
        covariance = (1 - kalmanGain) * predictedCovariance;
    }

    public double getHeading() {
        return heading;
    }
}

