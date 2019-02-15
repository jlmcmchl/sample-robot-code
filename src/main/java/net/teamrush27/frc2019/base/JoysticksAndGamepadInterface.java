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
	public boolean shouldShiftLowGear() {
		return driverRightJoystick.getZ() > 0.8;
	}

	@Override
	public boolean shouldShiftHighGear() {
		return driverRightJoystick.getZ() < 0.2;
	}
	
	@Override
	public ArmInput getArmInput() {
		Double armExtension = gamePad.getTriggerAxis(Hand.kRight) - gamePad.getTriggerAxis(Hand.kLeft);
		return new ArmInput(armExtension * .5, 0d);
	}
	
	@Override
	public Boolean getWantManipulateHatch() {
		return gamePad.getXButtonPressed();
	}
	
	@Override
	public Boolean getWantManipulateCargo() {
		return gamePad.getTriangleButtonPressed();
	}
	
	@Override
	public Boolean wantsStow() {
		return gamePad.getPadButton();
	}
	
	@Override
	public Boolean wantsGroundPickup() {
		return gamePad.getPOV() > 0 && Math.abs(gamePad.getPOV() - 270) <= 10;
	}
	
	@Override
	public Boolean wantsLevel1HumanLoad() {
		return gamePad.getPOV() > 0 && Math.abs(gamePad.getPOV() - 180) <= 10;
	}
	
	@Override
	public Boolean wantsLevel2() {
		return gamePad.getPOV() > 0 && Math.abs(gamePad.getPOV() - 90) <= 10;
	}
	
	@Override
	public Boolean wantsLevel3() {
		return gamePad.getPOV() > 0 && gamePad.getPOV() <= 10;
	}
	
	@Override
	public Boolean getWantsInvert() {
		return gamePad.getBumper(Hand.kRight);
	}
	
	@Override
	public Double getWristInput() {
		return gamePad.getY(Hand.kRight);
	}
	
	@Override
	public Boolean wantsPreClimb() {
		return gamePad.getShareButton();
	}
	
	@Override
	public Boolean wantsClimb() {
		return gamePad.getOptionsButton() && gamePad.getShareButton();
	}
	
	
}
