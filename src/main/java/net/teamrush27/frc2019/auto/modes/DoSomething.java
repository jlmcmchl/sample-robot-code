package net.teamrush27.frc2019.auto.modes;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.DriveStraightAction;

public class DoSomething extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new DriveStraightAction(5.0));
  }
}