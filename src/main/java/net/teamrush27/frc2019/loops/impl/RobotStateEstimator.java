package net.teamrush27.frc2019.loops.impl;

import net.teamrush27.frc2018.base.RobotState;
import net.teamrush27.frc2018.loops.Loop;
import net.teamrush27.frc2018.subsystems.impl.Drivetrain;
import net.teamrush27.frc2018.util.math.KinematicsUtils;
import net.teamrush27.frc2018.util.math.Rotation2d;
import net.teamrush27.frc2018.util.math.Twist2d;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    private static RobotStateEstimator INSTANCE = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return INSTANCE;
    }

    RobotStateEstimator() {
    }

    private final RobotState robotState = RobotState.getInstance();
    private final Drivetrain driveTrain = Drivetrain.getInstance();
    double leftEncoderPreviousDistance = 0;
    double rightEncoderPreviousDistance = 0;

    @Override
    public synchronized void onStart(double timestamp) {
        leftEncoderPreviousDistance = driveTrain.getLeftDistanceInches();
        rightEncoderPreviousDistance = driveTrain.getRightDistanceInches();
    }

    @Override
    public synchronized void onLoop(double timestamp) {
        final double rightDistanceTraveled = driveTrain.getRightDistanceInches();
        final double leftDistanceTraveled = driveTrain.getLeftDistanceInches();
        final Rotation2d gyroAngle = driveTrain.getGyroAngle();
        final Twist2d odometryVelocity = robotState.generateOdometryFromSensors(leftDistanceTraveled - leftEncoderPreviousDistance, rightDistanceTraveled - rightEncoderPreviousDistance, gyroAngle);
        final Twist2d predictedVelocity = KinematicsUtils.forwardKinematics(driveTrain.getLeftVelocityInchesPerSec(),
                driveTrain.getRightVelocityInchesPerSec());
        robotState.addObservations(timestamp, odometryVelocity, predictedVelocity);
        leftEncoderPreviousDistance = leftDistanceTraveled;
        rightEncoderPreviousDistance = rightDistanceTraveled;
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}
