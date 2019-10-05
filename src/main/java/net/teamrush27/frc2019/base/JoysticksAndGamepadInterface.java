package net.teamrush27.frc2019.base;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.wrappers.APEMJoystick;
import net.teamrush27.frc2019.wrappers.PS4Controller;

public class JoysticksAndGamepadInterface implements OperatorInterface {

  private static OperatorInterface INSTANCE = null;

  public static OperatorInterface getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new JoysticksAndGamepadInterface();
    }
    return INSTANCE;
  }

  //private final APEMJoystick driverLeftJoystick;
  //private final APEMJoystick driverRightJoystick;
  private final PS4Controller gamePad;

  public JoysticksAndGamepadInterface() {
    //driverLeftJoystick = new APEMJoystick(0);
    //driverRightJoystick = new APEMJoystick(1);
    gamePad = new PS4Controller(0);
  }

  @Override
  public DriveCommand getTankCommand() {
    double leftInput = gamePad.getY(Hand.kLeft);
    double rightInput = gamePad.getY(Hand.kRight);

    return new DriveCommand(leftInput, rightInput);
  }

  @Override
  public Boolean wantsAutoStop() {
    return false;
  }

  @Override
  public Boolean clear() {
    return //driverLeftJoystick.getLeftButtonPressed() ^ driverLeftJoystick.getRightButtonPressed()
        //^ driverRightJoystick.getLeftButtonPressed() ^ driverRightJoystick.getRightButtonPressed() ^ 
        gamePad.getXButtonPressed() ^ gamePad.getTriangleButtonPressed()
        ^ gamePad.getTriggerButtonPressed(Hand.kLeft) ^ gamePad.getTriggerButtonPressed(Hand.kRight)
        ^ gamePad.getPadButtonPressed() ^ gamePad.getMiddleButtonPressed()
        ^ gamePad.getBumperPressed(Hand.kLeft) ^ gamePad.getBumperPressed(Hand.kRight)
        ^ gamePad.getCircleButtonPressed() ^ gamePad.getSquareButtonPressed()
        ^ gamePad.getShareButtonPressed() ^ gamePad.getOptionsButtonPressed();
  }
}
