/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package net.teamrush27.frc2019;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.JoysticksAndGamepadInterface;
import net.teamrush27.frc2019.base.OperatorInterface;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Arm.WantedState;

public class Robot extends IterativeRobot {
  
  private Arm arm = Arm.getInstance();
  private OperatorInterface operatorInterface = JoysticksAndGamepadInterface.getInstance();

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit(){
    arm.setWantedState(WantedState.OPEN_LOOP);
  }
  
  @Override
  public void teleopPeriodic() {
    arm.setOpenLoopInput(operatorInterface.getArmInput());
  }

  @Override
  public void testPeriodic() {
  }
}
