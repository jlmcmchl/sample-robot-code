/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package net.teamrush27.frc2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.JoysticksAndGamepadInterface;
import net.teamrush27.frc2019.base.OperatorInterface;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.SubsystemManager;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Arm.WantedState;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;

public class Robot extends TimedRobot {
	
	private Arm arm = Arm.getInstance();
	private Drivetrain drivetrain = Drivetrain.getInstance();
	private OperatorInterface operatorInterface = JoysticksAndGamepadInterface.getInstance();
	private final SubsystemManager subsystemManager = new SubsystemManager(arm, drivetrain);
	private final Looper enabledLooper = new Looper();
	
	@Override
	public void robotInit() {
		subsystemManager.registerEnabledLoops(enabledLooper);
		
	}
	
	@Override
	public void robotPeriodic() {
		subsystemManager.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
	}
	
	@Override
	public void autonomousInit() {
		enabledLooper.start();
	}
	
	@Override
	public void autonomousPeriodic() {
	}
	
	@Override
	public void teleopInit() {
		enabledLooper.start();
		arm.setWantedState(WantedState.OPEN_LOOP);
		drivetrain.setOpenLoop(DriveCommand.defaultCommand());
	}
	
	@Override
	public void teleopPeriodic() {
		arm.setOpenLoopInput(operatorInterface.getArmInput());
		drivetrain.setOpenLoop(operatorInterface.getTankCommand());
	}
	
	@Override
	public void testPeriodic() {
	}
	
	
	@Override
	public void disabledPeriodic() {
	}
}
