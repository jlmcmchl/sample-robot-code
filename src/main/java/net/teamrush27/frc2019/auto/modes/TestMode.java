package net.teamrush27.frc2019.auto.modes;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.DrivePercentOutputAction;
import net.teamrush27.frc2019.auto.actions.impl.DriveTrajectory;
import net.teamrush27.frc2019.auto.actions.impl.TurnInPlaceAction;
import net.teamrush27.frc2019.auto.actions.impl.WaitAction;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class TestMode extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    System.out.println("Test mode");
    //Trajectory trajectory = TrajectoryGenerator.getInstance().getTrajectorySet().habToFrontShip.get(false);
    //DriveTrajectory driveTrajectory = new DriveTrajectory(trajectory, true);
    //runAction(driveTrajectory);

    Drivetrain driveTrain = Drivetrain.getInstance();
    driveTrain.shift(true);

/*
    runAction(new DrivePercentOutputAction(new DriveCommand(-0.2, -0.2)));
    runAction(new WaitAction(1));
    driveTrain.startLogging();
    runAction(new DrivePercentOutputAction(new DriveCommand(1.0, 1.0)));
    runAction(new WaitAction(5.0));
    runAction(new DrivePercentOutputAction(DriveCommand.defaultCommand()));
    driveTrain.stopLogging();*/

    runAction(new TurnInPlaceAction(2 * Math.PI * 8, 36));
  }
}