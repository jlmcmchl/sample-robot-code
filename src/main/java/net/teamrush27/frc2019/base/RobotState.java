package net.teamrush27.frc2019.base;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.interpolate.InterpolatingDouble;
import net.teamrush27.frc2019.util.interpolate.InterpolatingTreeMap;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.MathUtils;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Translation2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public class RobotState {

  private static RobotState instance_ = new RobotState();

  public static RobotState getInstance() {
    return instance_;
  }

  private static final int kObservationBufferSize = 100;

  // FPGATimestamp -> RigidTransform2d or Rotation2d
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
  private Twist2d vehicle_velocity_predicted_;
  private Twist2d vehicle_velocity_measured_;
  private double distance_driven_;

  private RobotState() {
    reset(0, new Pose2d());
  }

  /** Resets the field to robot transform (robot's position on the field) */
  public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
    field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
    field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
    Drivetrain.getInstance().setHeading(initial_field_to_vehicle.getRotation());
    vehicle_velocity_predicted_ = Twist2d.identity();
    vehicle_velocity_measured_ = Twist2d.identity();
    distance_driven_ = 0.0;
  }

  public synchronized void resetDistanceDriven() {
    distance_driven_ = 0.0;
  }

  /**
   * Returns the robot's position on the field at a certain time. Linearly interpolates between
   * stored robot positions to fill in the gaps.
   */
  public synchronized Pose2d getFieldToVehicle(double timestamp) {
    return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
    return field_to_vehicle_.lastEntry();
  }

  public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
    return getLatestFieldToVehicle()
        .getValue()
        .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
  }

  public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
    field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
  }

  public synchronized void addObservations(
      double timestamp, Twist2d measured_velocity, Twist2d predicted_velocity) {
    addFieldToVehicleObservation(
        timestamp,
        KinematicsUtils.integrateForwardKinematics(
            getLatestFieldToVehicle().getValue(), measured_velocity));
    vehicle_velocity_measured_ = measured_velocity;
    vehicle_velocity_predicted_ = predicted_velocity;
  }

  public synchronized Twist2d generateOdometryFromSensors(
      double left_encoder_delta_distance,
      double right_encoder_delta_distance,
      Rotation2d current_gyro_angle) {
    final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
    final Twist2d delta =
        KinematicsUtils.forwardKinematics(
            last_measurement.getRotation(),
            left_encoder_delta_distance,
            right_encoder_delta_distance,
            current_gyro_angle);
    distance_driven_ += delta.deltaX; // do we care about dy here?
    return delta;
  }

  public synchronized double getDistanceDriven() {
    return distance_driven_;
  }

  public synchronized Twist2d getPredictedVelocity() {
    return vehicle_velocity_predicted_;
  }

  public synchronized Twist2d getMeasuredVelocity() {
    return vehicle_velocity_measured_;
  }

  public void outputToSmartDashboard() {
    Pose2d odometry = getLatestFieldToVehicle().getValue();
    SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
    SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
    SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Linear Velocity", vehicle_velocity_measured_.deltaX);
  }
}