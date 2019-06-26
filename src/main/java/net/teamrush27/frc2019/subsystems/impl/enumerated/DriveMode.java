package net.teamrush27.frc2019.subsystems.impl.enumerated;

import com.ctre.phoenix.motorcontrol.ControlMode;

public enum DriveMode {
  OPEN_LOOP(ControlMode.PercentOutput); // open loop control

  private final ControlMode requestedControlMode;

  DriveMode(ControlMode requestedControlMode) {
    this.requestedControlMode = requestedControlMode;
  }

  public ControlMode getRequestedControlMode() {
    return requestedControlMode;
  }

}
