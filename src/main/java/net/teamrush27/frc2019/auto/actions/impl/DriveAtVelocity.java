package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.math.MathUtils;

public class DriveAtVelocity implements Action {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final RobotState state = RobotState.getInstance();
  private final double velocity;

  public DriveAtVelocity(double velocity) {
    this.velocity = velocity;
  }

  @Override
  public boolean isFinished() {
    return MathUtils.epsilonEquals(state.getMeasuredVelocity().deltaX, this.velocity, 100);
  }

  @Override
  public void update() {
    // TODO Auto-generated method stub
  }

  @Override
  public void done() {
    // TODO Auto-generated method stub

  }

  @Override
  public void start() {
    // TODO Auto-generated method stub
    drivetrain.setDriveVelocity(velocity);
  }

}