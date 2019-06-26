package net.teamrush27.frc2019.subsystems.impl.dto;

public class DriveCommand {
  
  public static DriveCommand COAST = new DriveCommand(0.0, 0.0);
  public static DriveCommand BRAKE = new DriveCommand(0.0, 0.0, true);
  
  private Double leftDriveInput;
  private Double rightDriveInput;
  private Boolean brakeMode;

  public DriveCommand(Double leftDriveInput, Double rightDriveInput) {
    this(leftDriveInput, rightDriveInput, false);
  }

  public DriveCommand(Double leftDriveInput, Double rightDriveInput, Boolean brakeMode) {
    this.leftDriveInput = leftDriveInput;
    this.rightDriveInput = rightDriveInput;
    this.brakeMode = brakeMode;
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

  public Boolean getBrakeMode() {
    return brakeMode;
  }

  public void setBrakeMode(Boolean brakeMode) {
    this.brakeMode = brakeMode;
  }
}
