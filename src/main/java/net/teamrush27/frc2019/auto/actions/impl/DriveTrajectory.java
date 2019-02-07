package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.math.Pose2dWithCurvature;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryIterator;
import net.teamrush27.frc2019.util.trajectory.timing.TimedState;
import net.teamrush27.frc2019.util.trajectory.timing.TimedView;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class DriveTrajectory implements Action {
  private static final Logger LOG = LogManager.getLogger(DriveTrajectory.class);
  
  private static final Drivetrain mDrive = Drivetrain.getInstance();
  private static final RobotState mRobotState = RobotState.getInstance();

  private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
  private final boolean mResetPose;

  public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
    this(trajectory, false);
  }


  public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
    mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
    mResetPose = resetPose;
  }

  @Override
  public boolean isFinished() {
    if (mDrive.isDoneWithTrajectory()) {
      LOG.info("Trajectory finished");
      return true;
    }
    return false;
  }

  @Override
  public void update() {
  }

  @Override
  public void done() {
  }

  @Override
  public void start() {
    LOG.info("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
    if (mResetPose) {
      mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
    }
    mDrive.setTrajectory(mTrajectory);
  }
}


