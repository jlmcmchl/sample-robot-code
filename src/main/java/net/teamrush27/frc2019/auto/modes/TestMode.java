package net.teamrush27.frc2019.auto.modes;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.DriveTrajectory;
import net.teamrush27.frc2019.auto.actions.impl.TurnInPlaceAction;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class TestMode extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    System.out.println("Test mode");
    Trajectory trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().habToFrontShip.get(false);
    DriveTrajectory driveTrajectory = new DriveTrajectory(trajectory, true);
    runAction(driveTrajectory);

    //runAction(new TurnInPlaceAction(2 * Math.PI * 8, 50));
  }
}