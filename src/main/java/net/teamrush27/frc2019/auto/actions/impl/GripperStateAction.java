package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.subsystems.impl.Gripper.WantedState;

public class GripperStateAction extends RunOnceAction {

  private WantedState wantedState;

  public GripperStateAction(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  @Override
  public void runOnce() {
    Gripper.getInstance().setWantedState(wantedState);
  }
}
