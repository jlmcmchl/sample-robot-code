package net.teamrush27.frc2019.subsystems.impl;

import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public class RobotStateEstimator extends Subsystem {
  static RobotStateEstimator instance_ = new RobotStateEstimator();
  private RobotState robot_state_ = RobotState.getInstance();
  private Drivetrain drive_ = Drivetrain.getInstance();
  private double left_encoder_prev_distance_ = 0.0;
  private double right_encoder_prev_distance_ = 0.0;

  RobotStateEstimator() {
  }

  public static RobotStateEstimator getInstance() {
    return instance_;
  }

  @Override
  public void outputToSmartDashboard() {

  }

  @Override
  public void stop() {
    // No-op
  }

  @Override
  public void zeroSensors() {

  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void test() {

  }

  private Loop loop = new Loop() {
    @Override
    public synchronized void onStart(double timestamp) {
      left_encoder_prev_distance_ = drive_.getLeftEncoderDistance();
      right_encoder_prev_distance_ = drive_.getRightEncoderDistance();

    }

    @Override
    public synchronized void onLoop(double timestamp) {
      final double left_distance = drive_.getLeftEncoderDistance();
      final double right_distance = drive_.getRightEncoderDistance();
      final double delta_left = left_distance - left_encoder_prev_distance_;
      final double delta_right = right_distance - right_encoder_prev_distance_;
      final Rotation2d gyro_angle = drive_.getHeading();
      final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
          delta_left, delta_right, gyro_angle);
      final Twist2d predicted_velocity = KinematicsUtils.forwardKinematics(drive_.getLeftLinearVelocity(),
          drive_.getRightLinearVelocity());
      robot_state_.addObservations(timestamp, odometry_velocity,
          predicted_velocity);
      left_encoder_prev_distance_ = left_distance;
      right_encoder_prev_distance_ = right_distance;
    }

    @Override
    public void onStop(double timestamp) {
      // no-op
    }
  };
}