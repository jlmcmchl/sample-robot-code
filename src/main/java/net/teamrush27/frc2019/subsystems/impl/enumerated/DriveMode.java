package net.teamrush27.frc2019.subsystems.impl.enumerated;

import com.ctre.phoenix.motorcontrol.ControlMode;

public enum DriveMode {
  OPEN_LOOP(ControlMode.PercentOutput);

  DriveMode(ControlMode mode) {
    this.mode = mode;
  }

  private ControlMode mode;

  public ControlMode getControlMode() {
    return mode;
  }
  
}
