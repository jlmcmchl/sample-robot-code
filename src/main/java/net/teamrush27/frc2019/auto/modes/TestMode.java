package net.teamrush27.frc2019.auto.modes;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.DriveTrajectory;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator.TrajectorySet.MirroredTrajectory;

public class TestMode extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    System.out.println("Test mode");
    Trajectory trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().justStraight.get(true);
    DriveTrajectory driveTrajectory = new DriveTrajectory(trajectory, true);
    runAction(driveTrajectory);

    Drivetrain.getInstance().stopLogging();
  }
}