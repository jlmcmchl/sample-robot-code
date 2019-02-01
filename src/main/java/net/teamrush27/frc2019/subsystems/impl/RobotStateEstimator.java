package net.teamrush27.frc2019.subsystems.impl;

import java.sql.Ref;
import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.util.CSVWritable;
import net.teamrush27.frc2019.util.ReflectingCSVWriter;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public class RobotStateEstimator extends Subsystem {
  private static String TAG = "ROBOTSTATEESTIMATOR";

  static RobotStateEstimator instance_ = new RobotStateEstimator();
  private RobotState robot_state_ = RobotState.getInstance();
  private Drivetrain drive_ = Drivetrain.getInstance();
  private double left_encoder_prev_distance_ = 0.0;
  private double right_encoder_prev_distance_ = 0.0;
  private ReflectingCSVWriter CSVWriter;

  private StateFrame stateFrame;

  RobotStateEstimator() {
    stateFrame = new StateFrame();
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
  public void registerEnabledLoops(ILooper enabledLooper) {
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
      final Twist2d predicted_velocity = KinematicsUtils
          .forwardKinematics(drive_.getLeftLinearVelocity(),
              drive_.getRightLinearVelocity());
      robot_state_.addObservations(timestamp, odometry_velocity,
          predicted_velocity);
      left_encoder_prev_distance_ = left_distance;
      right_encoder_prev_distance_ = right_distance;

      stateFrame.timestamp = timestamp;

      stateFrame.left_distance = left_distance;
      stateFrame.right_distance = right_distance;
      stateFrame.delta_left = delta_left;
      stateFrame.delta_right = delta_right;
      stateFrame.gyro_angle = gyro_angle;
      stateFrame.odometry_velocity = odometry_velocity;
      stateFrame.predicted_velocity = predicted_velocity;

      if (CSVWriter != null) {
        CSVWriter.add(stateFrame);
      }

      //System.out.println(String.format("%s,%s", timestamp, robot_state_.getLatestFieldToVehicle().getValue()));
    }

    @Override
    public void onStop(double timestamp) {
      // no-op
    }

    @Override
    public String id() {
      return TAG;
    }
  };

  @Override
  public String id() {
    return TAG;
  }

  public synchronized void startLogging() {
    if (CSVWriter == null) {
      CSVWriter = new ReflectingCSVWriter<>("/home/lvuser/STATE-LOGS.csv", StateFrame.class);
    }
  }

  public synchronized void stopLogging() {
    if (CSVWriter != null) {
      CSVWriter.flush();
      CSVWriter = null;
    }
  }

  public class StateFrame {

    public double timestamp;

    public double left_distance;
    public double right_distance;
    public double delta_left;
    public double delta_right;
    public Rotation2d gyro_angle;
    public Twist2d odometry_velocity;
    public Twist2d predicted_velocity;
  }
}