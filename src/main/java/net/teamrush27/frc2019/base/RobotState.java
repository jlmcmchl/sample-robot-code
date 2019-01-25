package net.teamrush27.frc2019.base;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.constants.FieldConstants;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.interpolate.InterpolatingDouble;
import net.teamrush27.frc2019.util.interpolate.InterpolatingTreeMap;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Translation2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public class RobotState {
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    private static final int OBSERVATION_BUFFER_SIZE = 100;

    private static final Pose2d VEHICLE_TO_CAMERA = new Pose2d(
            new Translation2d(RobotConstants.CAMERA_X_OFFSET, RobotConstants.CAMERA_Y_OFFSET), new Rotation2d());

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> fieldToVehicle;
    private Twist2d vehicleVelocityPredicted;
    private Twist2d vehicleVelocityMeasured;
    private double distanceDriven;
    //private GoalTracker goalTracker;
    private Rotation2d cameraPitchCorrection;
    private Rotation2d cameraYawCorrection;
    private double heightDifferential;
    //private ShooterAimingParameters cachedShooterAimingParameters = null;


    private RobotState() {
        reset(0, new Pose2d());
    }

    public synchronized void reset(double startTime, Pose2d initalFieldToVehicle) {
        fieldToVehicle = new InterpolatingTreeMap<>(OBSERVATION_BUFFER_SIZE);
        fieldToVehicle.put(new InterpolatingDouble(startTime), initalFieldToVehicle);
        vehicleVelocityPredicted = Twist2d.identity();
        vehicleVelocityMeasured = Twist2d.identity();
        Drivetrain.getInstance().setHeading(initalFieldToVehicle.getRotation());
//        goalTracker = new GoalTracker();
        cameraPitchCorrection = Rotation2d.fromDegrees(-RobotConstants.CAMERA_PITCH_ANGLE);
        cameraYawCorrection = Rotation2d.fromDegrees(-RobotConstants.CAMERA_YAW_ANGLE);
        heightDifferential = FieldConstants.BOILER_TARGET_TOP_HEIGHT - RobotConstants.CAMERA_Z_OFFSET;
        distanceDriven = 0.0;
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return fieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return fieldToVehicle.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
            .transformBy(Pose2d.exp(vehicleVelocityPredicted.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
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
        final Pose2d lastMeasurement = getLatestFieldToVehicle().getValue();
        final Twist2d change = KinematicsUtils.forwardKinematics(lastMeasurement.getRotation(),
                leftEncoderDeltaDistance, rightEncoderDeltaDistance, currentGyroAngle);
        distanceDriven += change.deltaX;
        return change;
    }

    public synchronized double getDistanceDriven() {
        return distanceDriven;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicleVelocityPredicted;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicleVelocityMeasured;
    }

    public void outputToSmartDashboard() {
        Pose2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
        SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
        SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Linear Velocity", vehicleVelocityMeasured.deltaX);
    }
}
