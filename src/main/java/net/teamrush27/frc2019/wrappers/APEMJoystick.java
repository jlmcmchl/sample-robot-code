package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.wpilibj.Joystick;

public class APEMJoystick extends Joystick {

	public APEMJoystick(int port) {
		super(port);
	}

	// FlightstickPro
	// trigger = button 1
	// left button = button 2
	// right button = button 3
	// center button = button 4

	public double getThrottle(){
		return super.getZ();
	}

	public boolean getLeftButton(){
		return super.getRawButton(2);
	}

	public boolean getRightButton(){
		return super.getRawButton(1);
	}

	public boolean getRightButtonPressed() {
		return super.getRawButtonPressed(1);
	}

	public boolean getLeftButtonPressed() {
		return super.getRawButtonPressed(2);
	}
	
}
