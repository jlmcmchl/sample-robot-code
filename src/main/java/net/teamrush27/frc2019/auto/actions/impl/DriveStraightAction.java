package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.math.MathUtils;

public class DriveStraightAction implements Action {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final RobotState state = RobotState.getInstance();
  private final double distance;
  private double leftStartDistance, rightStartDistance;

  public DriveStraightAction(double distance) {
    this.distance = distance;
  }

  @Override
  public boolean isFinished() {
    return drivetrain.getLeftEncoderDistance() - leftStartDistance > distance
        && drivetrain.getRightEncoderDistance() - rightStartDistance > distance;
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {

  }

  @Override
  public void start() {
    leftStartDistance = drivetrain.getLeftEncoderDistance();
    rightStartDistance = drivetrain.getRightEncoderDistance();
    drivetrain.setDriveDistance(distance);
  }
}