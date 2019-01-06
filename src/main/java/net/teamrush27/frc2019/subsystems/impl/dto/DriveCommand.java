package net.teamrush27.frc2019.subsystems.impl.dto;

public class DriveCommand {
	private Double leftDriveInput;
	private Double rightDriveInput;
	
	public DriveCommand(Double leftDriveInput, Double rightDriveInput) {
		this.leftDriveInput = leftDriveInput;
		this.rightDriveInput = rightDriveInput;
	}
	
	public Double getLeftDriveInput() {
		return leftDriveInput;
	}
	
	public void setLeftDriveInput(Double leftDriveInput) {
		this.leftDriveInput = leftDriveInput;
	}
	
	public Double getRightDriveInput() {
		return rightDriveInput;
	}
	
	public void setRightDriveInput(Double rightDriveInput) {
		this.rightDriveInput = rightDriveInput;
	}
}
