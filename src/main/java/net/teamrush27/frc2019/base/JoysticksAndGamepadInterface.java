package net.teamrush27.frc2019.base;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.MathUtils;
import net.teamrush27.frc2019.util.math.Twist2d;
import net.teamrush27.frc2019.wrappers.APEMJoystick;
import net.teamrush27.frc2019.wrappers.PS4Controller;
import net.teamrush27.frc2019.wrappers.XboxController;

public class JoysticksAndGamepadInterface implements OperatorInterface {

  private static OperatorInterface INSTANCE = null;

  public static OperatorInterface getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new JoysticksAndGamepadInterface();
    }
    return INSTANCE;
  }

  private final APEMJoystick driverLeftJoystick;
  private final APEMJoystick driverRightJoystick;
  private final XboxController driver;
  private final PS4Controller gamePad;

  public JoysticksAndGamepadInterface() {
    driverLeftJoystick = new APEMJoystick(0);
    driverRightJoystick = new APEMJoystick(1);
    driver = new XboxController(3);
    gamePad = new PS4Controller(2);
  }

  @Override
  public DriveCommand getTankCommand() {
    if (driverLeftJoystick.getButtonCount() == 0 || driverRightJoystick.getButtonCount() == 0) {
      return new DriveCommand(0.0, 0.0);
    }

    double leftInput = driverLeftJoystick.getY(Hand.kLeft);
    double rightInput = driverRightJoystick.getY(Hand.kRight);

    if (MathUtils.epsilonEquals(leftInput, 0, 0.05)) {
      leftInput = 0;
    }

    if (MathUtils.epsilonEquals(rightInput, 0, 0.05)) {
      rightInput = 0;
    }

    return new DriveCommand(leftInput, rightInput);
  }

  @Override
  public DriveCommand getCheezyishDrive() {
    if (driver.getButtonCount() == 0) {
      return new DriveCommand(0.0, 0.0);
    }

    double throttle = driver.getY(Hand.kLeft);
    double wheel = driver.getX(Hand.kRight);
    boolean quickTurn = driver.getBumper(Hand.kRight);

    if (MathUtils.epsilonEquals(throttle, 0.0, 0.05)) {
      throttle = 0.0;
    }

    if (MathUtils.epsilonEquals(wheel, 0.0, 0.05)) {
      wheel = 0.0;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveCommand signal = KinematicsUtils.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0,
        Math.max(Math.abs(signal.getLeftDriveInput()), Math.abs(signal.getRightDriveInput())));
    return new DriveCommand(signal.getLeftDriveInput() / scaling_factor, signal.getRightDriveInput() / scaling_factor);
  }

  @Override
  public Boolean wantsAutoStop() {
    return false;
  }

  @Override
  public Boolean clear() {
    return driverLeftJoystick.getLeftButtonPressed() ^ driverLeftJoystick.getRightButtonPressed()
        ^ driverRightJoystick.getLeftButtonPressed() ^ driverRightJoystick.getRightButtonPressed()
        ^ gamePad.getXButtonPressed() ^ gamePad.getTriangleButtonPressed() ^ gamePad.getTriggerButtonPressed(Hand.kLeft)
        ^ gamePad.getTriggerButtonPressed(Hand.kRight) ^ gamePad.getPadButtonPressed()
        ^ gamePad.getMiddleButtonPressed() ^ gamePad.getBumperPressed(Hand.kLeft)
        ^ gamePad.getBumperPressed(Hand.kRight) ^ gamePad.getCircleButtonPressed() ^ gamePad.getSquareButtonPressed()
        ^ gamePad.getShareButtonPressed() ^ gamePad.getOptionsButtonPressed();
  }
}
