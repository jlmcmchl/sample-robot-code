package net.teamrush27.frc2019.base;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.constants.FieldConstants;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.util.interpolate.InterpolatingDouble;
import net.teamrush27.frc2019.util.interpolate.InterpolatingTreeMap;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.RigidTransform2d;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Translation2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public class RobotState {
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    private static final int OBSERVATION_BUFFER_SIZE = 100;

    private static final RigidTransform2d VEHICLE_TO_CAMERA = new RigidTransform2d(
            new Translation2d(RobotConstants.CAMERA_X_OFFSET, RobotConstants.CAMERA_Y_OFFSET), new Rotation2d());

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> fieldToVehicle;
    private Twist2d vehicleVelocityPredicted;
    private Twist2d vehicleVelocityMeasured;
    private double distanceDriven;
    //private GoalTracker goalTracker;
    private Rotation2d cameraPitchCorrection;
    private Rotation2d cameraYawCorrection;
    private double heightDifferential;
    //private ShooterAimingParameters cachedShooterAimingParameters = null;


    private RobotState() {
        reset(0, new RigidTransform2d());
    }

    public synchronized void reset(double startTime, RigidTransform2d initalFieldToVehicle) {
        fieldToVehicle = new InterpolatingTreeMap<>(OBSERVATION_BUFFER_SIZE);
        fieldToVehicle.put(new InterpolatingDouble(startTime), initalFieldToVehicle);
        vehicleVelocityPredicted = Twist2d.identity();
        vehicleVelocityMeasured = Twist2d.identity();
//        goalTracker = new GoalTracker();
        cameraPitchCorrection = Rotation2d.fromDegrees(-RobotConstants.CAMERA_PITCH_ANGLE);
        cameraYawCorrection = Rotation2d.fromDegrees(-RobotConstants.CAMERA_YAW_ANGLE);
        heightDifferential = FieldConstants.BOILER_TARGET_TOP_HEIGHT - RobotConstants.CAMERA_Z_OFFSET;
        distanceDriven = 0.0;
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
        return fieldToVehicle.lastEntry();
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform2d observation) {
        fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d measuredVelocity,
                                             Twist2d predictedVelocity) {
        addFieldToVehicleObservation(timestamp,
                KinematicsUtils.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measuredVelocity));
        vehicleVelocityMeasured = measuredVelocity;
        vehicleVelocityPredicted = predictedVelocity;
    }

    public synchronized Twist2d generateOdometryFromSensors(double leftEncoderDeltaDistance,
                                                            double rightEncoderDeltaDistance, Rotation2d currentGyroAngle) {
        final RigidTransform2d lastMeasurement = getLatestFieldToVehicle().getValue();
        final Twist2d change = KinematicsUtils.forwardKinematics(lastMeasurement.getRotation(),
                leftEncoderDeltaDistance, rightEncoderDeltaDistance, currentGyroAngle);
        distanceDriven += change.deltaX;
        return change;
    }



}
