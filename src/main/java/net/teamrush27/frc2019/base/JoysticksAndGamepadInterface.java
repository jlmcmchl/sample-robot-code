package net.teamrush27.frc2019.base;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.wrappers.APEMJoystick;
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
	
	private final APEMJoystick driverLeftJoystick;
	private final APEMJoystick driverRightJoystick;
	private final PS4Controller gamePad;
	
	public JoysticksAndGamepadInterface() {
		driverLeftJoystick = new APEMJoystick(0);
		driverRightJoystick = new APEMJoystick(1);
		gamePad = new PS4Controller(2);
	}
	
	@Override
	public DriveCommand getTankCommand() {
		return new DriveCommand(driverLeftJoystick.getY(), driverRightJoystick.getY());
	}

	private boolean shiftLatch = false;

	@Override
	public boolean getShift() {
		if (!shiftLatch && driverRightJoystick.getZ() > 0.75) {
			shiftLatch = true;
			return true;
		} else if (driverRightJoystick.getZ() < 0.75) {
			shiftLatch = false;
		}
		return false;
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
	public Boolean getWantsCargoShip() {
		return gamePad.getBumper(Hand.kLeft);
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
		return gamePad.getPOV() >= 0 && gamePad.getPOV() <= 10;
	}
	
	@Override
	public Boolean getWantsInvert() {
		return !gamePad.getBumper(Hand.kRight);
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

	@Override
	public Boolean wantsSwitchPipeline() {
		return driverRightJoystick.getLeftButtonPressed();
	}

	@Override
	public Boolean wantsIncreaseOffset() {
		return gamePad.getTriggerButtonPressed(Hand.kLeft);
	}

	@Override
	public Boolean wantsDecreaseOffset() {
		return gamePad.getTriggerButtonPressed(Hand.kRight);
	}

	@Override
	public Boolean wantsToggleLimelightSteering() {
		return driverRightJoystick.getLeftButtonPressed();
	}

	@Override
	public void setRumble(double frac) {
		gamePad.setRumble(RumbleType.kLeftRumble, frac);
		gamePad.setRumble(RumbleType.kRightRumble, frac);
	}
}
