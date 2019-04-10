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

public class RightCargo extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    Trajectory habToCargoSideClose = TrajectoryGenerator.getInstance()
        .getTrajectorySet().habToCargoSideClose.getRight();
    Trajectory cargoSideCloseToHP = TrajectoryGenerator.getInstance()
        .getTrajectorySet().cargoSideCloseToHP.getRight();
    Trajectory hpToCargoSideMid = TrajectoryGenerator.getInstance()
        .getTrajectorySet().hpToCargoSideMid.getRight();

    Trajectory scootBack2 = TrajectoryGenerator.getInstance().getTrajectorySet().scootBack2
        .getRight();

    List commands = Arrays.asList(
        new ParallelAction(
            new SeriesAction(
                new WaitUntilCrossXBoundaryCommand(100),
                new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_1, false, true)),
            new DriveTrajectory(habToCargoSideClose, true, true)),
        new LimelightTrackingAction(false, 500),
        new GripperStateAction(Gripper.WantedState.EXHAUST_HATCH, .1),
        new ParallelAction(
            new SeriesAction(
                new WaitUntilCrossXBoundaryCommand(200, true),
                new GripperStateAction(Gripper.WantedState.INTAKE_HATCH),
                new AutoSuperstructurePosition(WantedState.HUMAN_LOAD, true, true)),
            new DriveTrajectory(cargoSideCloseToHP, false)),
        new LimelightTrackingAction(true, 430),
        new ParallelAction(
            new SeriesAction(
                new WaitUntilCrossXBoundaryCommand(70),
                new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_1, false, true)),
            new DriveTrajectory(hpToCargoSideMid, false)),
        new LimelightTrackingAction(false, 350)/*,
        new GripperStateAction(Gripper.WantedState.EXHAUST_HATCH, .1),
        new DriveTrajectory(scootBack2, false),
        new GripperStateAction(Gripper.WantedState.INTAKE_HATCH),
        new AutoSuperstructurePosition(WantedState.STOW, false, false)
        */);

    runAction(new SeriesAction(commands));
  }
}
