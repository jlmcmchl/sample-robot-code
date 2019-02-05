package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.ReflectingCSVWriter;
import net.teamrush27.frc2019.util.math.MathUtils;
import net.teamrush27.frc2019.util.physics.DriveCharacterization;
import net.teamrush27.frc2019.util.physics.DriveCharacterization.AccelerationDataPoint;

public class CollectAccelerationData implements Action {
  private static final double kPower = 0.5;
  private static final double kTotalTime = 2.0; //how long to run the test for
  private static final Drivetrain mDrive = Drivetrain.getInstance();

  private final ReflectingCSVWriter<DriveCharacterization.AccelerationDataPoint> mCSVWriter;
  private final List<DriveCharacterization.AccelerationDataPoint> mAccelerationData;
  private final boolean mTurn;
  private final boolean mReverse;

  private double mStartTime = 0.0;
  private double mPrevVelocity = 0.0;
  private double mPrevTime = 0.0;

  /**
   * @param data     reference to the list where data points should be stored
   * @param reverse  if true drive in reverse, if false drive normally
   * @param turn     if true turn, if false drive straight
   */
  public CollectAccelerationData(List<AccelerationDataPoint> data, boolean reverse, boolean turn) {
    mAccelerationData = data;
    mReverse = reverse;
    mTurn = turn;
    mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/ACCEL_DATA.csv", DriveCharacterization.AccelerationDataPoint.class);
  }

  @Override
  public void start() {
    mDrive.setOpenLoop(new DriveCommand((mReverse ? -1.0 : 1.0) * kPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kPower));
    mStartTime = Timer.getFPGATimestamp();
    mPrevTime = mStartTime;
  }

  @Override
  public void update() {
    double currentVelocity = (Math.abs(mDrive.getLeftVelocityNativeUnits()) + Math.abs(mDrive.getRightVelocityNativeUnits())) / 4096.0 * Math.PI * 10;
    double currentTime = Timer.getFPGATimestamp();

    //don't calculate acceleration until we've populated prevTime and prevVelocity
    if (mPrevTime == mStartTime) {
      mPrevTime = currentTime;
      mPrevVelocity = currentVelocity;
      return;
    }

    double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);

    //ignore accelerations that are too small
    if (acceleration < MathUtils.DEFAULT_MAX_ERROR) {
      mPrevTime = currentTime;
      mPrevVelocity = currentVelocity;
      return;
    }

    mAccelerationData.add(new DriveCharacterization.AccelerationDataPoint(
        currentVelocity, //convert to radians per second
        kPower * 12.0, //convert to volts
        acceleration
    ));

    mCSVWriter.add(mAccelerationData.get(mAccelerationData.size() - 1));

    mPrevTime = currentTime;
    mPrevVelocity = currentVelocity;
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - mStartTime > kTotalTime;
  }

  @Override
  public void done() {
    mDrive.setOpenLoop(DriveCommand.BRAKE);
    mCSVWriter.flush();
  }
}
