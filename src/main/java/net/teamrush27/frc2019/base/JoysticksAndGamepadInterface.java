package net.teamrush27.frc2019.base;

import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.wrappers.FlightstickPro;
import net.teamrush27.frc2019.wrappers.LogitechPad;

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
	private final LogitechPad gamePad;
	
	public JoysticksAndGamepadInterface() {
		driverLeftJoystick = new FlightstickPro(0);
		driverRightJoystick = new FlightstickPro(1);
		gamePad = new LogitechPad(2);
	}
	
	@Override
	public DriveCommand getTankCommand() {
		return new DriveCommand(-driverLeftJoystick.getY(), -driverRightJoystick.getY());
	}
	
	@Override
	public ArmInput getArmInput() {
		Double armExtension = gamePad.getRightTriggerAxis() - gamePad.getLeftTriggerAxis();
		
		return new ArmInput(armExtension, gamePad.getLeftStickX());
		
	}
}
