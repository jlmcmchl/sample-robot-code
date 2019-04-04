package net.teamrush27.frc2019.auto.modes;

import java.util.Arrays;
import java.util.List;
import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.AutoSuperstructurePosition;
import net.teamrush27.frc2019.auto.actions.impl.DriveTrajectory;
import net.teamrush27.frc2019.auto.actions.impl.GripperStateAction;
import net.teamrush27.frc2019.auto.actions.impl.LimelightTrackingAction;
import net.teamrush27.frc2019.auto.actions.impl.ParallelAction;
import net.teamrush27.frc2019.auto.actions.impl.SeriesAction;
import net.teamrush27.frc2019.auto.actions.impl.WaitUntilCrossXBoundaryCommand;
import net.teamrush27.frc2019.managers.SuperstructureManager.WantedState;
import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class LeftRocket extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    Trajectory habToFarRocket = TrajectoryGenerator.getInstance()
        .getTrajectorySet().habToRocketRear.getLeft();
    Trajectory farRocketToMidline = TrajectoryGenerator.getInstance()
        .getTrajectorySet().rocketRearToMidline.getLeft();
    Trajectory midlineToHP = TrajectoryGenerator.getInstance().getTrajectorySet().midlineToHP
        .getLeft();
    Trajectory hpToRocketClose = TrajectoryGenerator.getInstance()
        .getTrajectorySet().hpToRocketFront.getLeft();

    Trajectory scootBack = TrajectoryGenerator.getInstance().getTrajectorySet().scootBack
        .getLeft();


    Trajectory altRocketRearToHP = TrajectoryGenerator.getInstance().getTrajectorySet().altRocketRearToHP.getRight();

    List commands = Arrays.asList(
        new ParallelAction(
            new SeriesAction(
                new WaitUntilCrossXBoundaryCommand(260),
                new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_2, false, true)),
            new DriveTrajectory(habToFarRocket, true)),
        new LimelightTrackingAction(false, 1000),
        new GripperStateAction(Gripper.WantedState.EXHAUST_HATCH),
        new DriveTrajectory(farRocketToMidline, false),
        new GripperStateAction(Gripper.WantedState.OFF),
        new AutoSuperstructurePosition(WantedState.HUMAN_LOAD, false, true),
        new DriveTrajectory(midlineToHP, false),
        new LimelightTrackingAction(false, 430),
        new ParallelAction(
            new SeriesAction(
                new WaitUntilCrossXBoundaryCommand(24),
                new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_2, true, true)),
            new DriveTrajectory(hpToRocketClose, false)),
        new LimelightTrackingAction(true, 1000)/*,
        new GripperStateAction(Gripper.WantedState.EXHAUST_HATCH),
        new DriveTrajectory(scootBack, false),
        new GripperStateAction(Gripper.WantedState.INTAKE_HATCH),
        new AutoSuperstructurePosition(WantedState.STOW, false, false)
        */);

    runAction(new SeriesAction(commands));
  }
}
