package net.teamrush27.frc2019.base;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.wrappers.FlightstickPro;
import net.teamrush27.frc2019.wrappers.LogitechPad;
import net.teamrush27.frc2019.wrappers.PS4Controller;

public class JoysticksAndGamepadInterface implements OperatorInterface {
	
	private static OperatorInterface INSTANCE = null;
	
	public static OperatorInterface getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new JoysticksAndGamepadInterface();
		}
		return INSTANCE;
	}
	
	private final FlightstickPro driverLeftJoystick;
	private final FlightstickPro driverRightJoystick;
	private final PS4Controller gamePad;
	
	public JoysticksAndGamepadInterface() {
		driverLeftJoystick = new FlightstickPro(0);
		driverRightJoystick = new FlightstickPro(1);
		gamePad = new PS4Controller(2);
	}
	
	@Override
	public DriveCommand getTankCommand() {
		return new DriveCommand(-driverLeftJoystick.getY(), -driverRightJoystick.getY());
	}
	
	@Override
	public ArmInput getArmInput() {
		Double armExtension = gamePad.getTriggerAxis(Hand.kRight) - gamePad.getTriggerAxis(Hand.kLeft);
		return new ArmInput(armExtension * .1, gamePad.getX(Hand.kLeft));
	}
	
	@Override
	public Boolean getWantManipulateHatch() {
		return gamePad.getTriangleButtonPressed();
	}
	
	@Override
	public Boolean getWantManipulateCargo() {
		return gamePad.getXButton();
	}
	
	@Override
	public Double getWristInput() {
		return gamePad.getY(Hand.kRight);
	}
	
	@Override
	public Boolean wantsPreClimb() {
		return gamePad.getShareButtonPressed();
	}
	
	@Override
	public Boolean wantsClimb() {
		return gamePad.getOptionsButtonPressed() && gamePad.getShareButton();
	}
}
