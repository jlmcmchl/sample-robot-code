package net.teamrush27.frc2019.util.math;

import net.teamrush27.frc2019.Robot;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a
 * differential drive (with a corrective factor to account for skidding).
 */
public class KinematicsUtils {
  private static final double kEpsilon = 1E-9;

  /**
     * Uses inverse kinematics to convert a Twist2d into left and right wheel velocities
     */
    public static DriveCommand inverseKinematics(Twist2d velocity) {
      if (Math.abs(velocity.deltaTheta) < kEpsilon) {
          return new DriveCommand(velocity.deltaX, velocity.deltaY);
      }
      double delta_v = RobotConstants.TRACK_WIDTH * velocity.deltaTheta / (2 * RobotConstants.SCRUB_FACTOR);
      return new DriveCommand(velocity.deltaX - delta_v, velocity.deltaX + delta_v);
  }
}
