package net.teamrush27.frc2019.auto.modes;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.DriveTrajectory;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class TestMode extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    System.out.println("Test mode");
    Drivetrain.getInstance().startLogging();

    runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().justStraight.get(true), true));

    Drivetrain.getInstance().stopLogging();

        /*runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().generateTrajectory(
                false,
                Arrays.asList(Pose2d.identity(), Pose2d.fromTranslation(new Translation2d(48.0, 0.0)),
                        new Pose2d(new Translation2d(96.0, -48.0), Rotation2d.fromDegrees(-90.0)),
                        new Pose2d(new Translation2d(96.0, -96.0), Rotation2d.fromDegrees(-90.0))),
                Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                120.0, 120.0, 10.0),true));*/

/*
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().generateTrajectory(
                false,
                Arrays.asList(
                        new Pose2d(new Translation2d(96.0, -96.0), Rotation2d.fromDegrees(90.0)),
                        new Pose2d(new Translation2d(96.0, -48.0), Rotation2d.fromDegrees(90.0)),
                        Pose2d.fromRotation(Rotation2d.fromDegrees(180.0))),
                Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                120.0, 120.0, 10.0)));*/
  }
}