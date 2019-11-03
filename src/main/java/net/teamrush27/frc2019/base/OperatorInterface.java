package net.teamrush27.frc2019.base;

import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public interface OperatorInterface {
	
	/**
	 * @return the {@link DriveCommand} that represents the driver's joystick inputs
	 */
	DriveCommand getTankCommand();

	/**
	 * @return the {@link DriveCommand} that represents the drivers's inputs, as Cheesy-ish Drive
	 */
	DriveCommand getCheezyishDrive();

	/**
	 * @return if the driver wants to stop auto mode
	 */
	Boolean wantsAutoStop(boolean cheesyDrive);

	/*
	 * @return if the operator wants to toggle between cheesy and tank drive
	 */
	Boolean toggleDriveStyle();
	
	/**
	 * Clears joystick and gamepad button presses
	 * so that .getButtonPressed() returns false next loop if button not actively pressed
	 *
	 * @return if any button pressed were suppressed
	 */
	Boolean clear();
}
