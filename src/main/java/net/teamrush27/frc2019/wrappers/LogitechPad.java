package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechPad extends Joystick {

	public LogitechPad(int port) {
		super(port);
	}
	
	public boolean getAButton(){
		return super.getRawButton(2);
	}

	public boolean getBButton(){
		return super.getRawButton(3);
	}

	public boolean getXButton(){
		return super.getRawButton(1);
	}

	public boolean getYButton(){
		return super.getRawButton(4);
	}
	
	public boolean getLeftBumperButton(){
		return super.getRawButton(5);
	}
	
	public boolean getRightBumperButton(){
		return super.getRawButton(6);
	}
	
	public boolean getLeftTriggerButton(){
		return super.getRawButton(7);
	}

	public double getLeftTriggerAxis() {
		return super.getRawAxis(2);
	}
	
	public boolean getRightTriggerButton(){
		return super.getRawButton(8);
	}

	public double getRightTriggerAxis() {
		return super.getRawAxis(3);
	}
	
	public boolean getBackButton(){
		return super.getRawButton(9);
	}
	
	public boolean getStartButton(){
		return super.getRawButton(10);
	}

	public boolean getLeftStickButton(){
		return super.getRawButton(11);
	}
	
	public boolean getRightStickButton(){
		return super.getRawButton(12);
	}
	
	public boolean getLeftDPad(){
		return getDPadDirection() == 270;
	}
	
	public boolean getRightDPad(){
		return getDPadDirection() == 90;
	}
	
	public boolean getUpDPad(){
		return getDPadDirection() == 0;
	}
	
	public boolean getDownDPad(){
		return getDPadDirection() == 180;
	}
	
	private int getDPadDirection(){
		return super.getPOV();
	}
	
	public double getLeftStickX(){
		return super.getRawAxis(0);
	}
	
	public double getLeftStickY(){
		return super.getRawAxis(1);
	}
	
	public double getRightStickX(){
		return super.getRawAxis(2);
	}
	
	public double getRightStickY(){
		return super.getRawAxis(3);
	}

}
