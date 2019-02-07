/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package net.teamrush27.frc2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.auto.AutoModeExecutor;
import net.teamrush27.frc2019.auto.modes.TestMode;
import net.teamrush27.frc2019.base.JoysticksAndGamepadInterface;
import net.teamrush27.frc2019.base.OperatorInterface;
import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.SubsystemManager;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.subsystems.impl.RobotStateEstimator;
import net.teamrush27.frc2019.subsystems.impl.Wrist;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.crash.CrashTracker;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class Robot extends TimedRobot {
	
	private RobotStateEstimator robotStateEstimator = RobotStateEstimator.getInstance();
	private Drivetrain drivetrain = Drivetrain.getInstance();
	private Arm arm = Arm.getInstance();
	private Gripper gripper = Gripper.getInstance();
	private Wrist wrist = Wrist.getInstance();
	private OperatorInterface operatorInterface = JoysticksAndGamepadInterface.getInstance();
	private final SubsystemManager subsystemManager = new SubsystemManager(robotStateEstimator, arm,
		drivetrain);
	private final Looper enabledLooper = new Looper();
	private final Looper disabledLooper = new Looper();
	
	private AutoModeExecutor autoModeExecutor;
	
	@Override
	public void robotInit() {
		subsystemManager.registerEnabledLoops(enabledLooper);
		subsystemManager.registerDisabledLoops(disabledLooper);
		
		TrajectoryGenerator.getInstance().generateTrajectories();
	}
	
	@Override
	public void robotPeriodic() {
		subsystemManager.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
	}
	
	@Override
	public void autonomousInit() {
		disabledLooper.stop();
		drivetrain.startLogging();
		//subsystemManager.startLogging();
		//robotStateEstimator.startLogging();
		enabledLooper.start();
		
		autoModeExecutor = new AutoModeExecutor();
		autoModeExecutor.setAutoMode(new TestMode());
		autoModeExecutor.start();
	}
	
	@Override
	public void autonomousPeriodic() {
	}
	
	@Override
	public void teleopInit() {
		disabledLooper.stop();
		enabledLooper.start();
		arm.setWantedState(Arm.WantedState.OPEN_LOOP);
		gripper.setWantedState(Gripper.WantedState.OFF);
		wrist.setWantedState(Wrist.WantedState.OPEN_LOOP);
		//subsystemManager.startLogging();
		//robotStateEstimator.startLogging();
		//drivetrain.startLogging();
		drivetrain.setOpenLoop(DriveCommand.defaultCommand());
	}
	
	@Override
	public void teleopPeriodic() {
		//arm.setOpenLoopInput(operatorInterface.getArmInput());
		if (operatorInterface.getWantManipulateCargo()) {
			gripper.transitionCargo();
		} else if (operatorInterface.getWantManipulateHatch()) {
			gripper.transitionHatch();
		}
		
		wrist.setOpenLoopInput(operatorInterface.getWristInput());
		
		drivetrain.setOpenLoop(operatorInterface.getTankCommand());
	}
	
	@Override
	public void testInit() {
		enabledLooper.stop();
		disabledLooper.stop();
		
	}
	
	@Override
	public void testPeriodic() {
	}
	
	@Override
	public void disabledInit() {
		SmartDashboard.putString("Match Cycle", "DISABLED");
		
		if (autoModeExecutor != null) {
			autoModeExecutor.stop();
		}
		
		try {
			CrashTracker.logDisabledInit();
			enabledLooper.stop();
			
			//subsystemManager.stopLogging();
			//drivetrain.stopLogging();
			//robotStateEstimator.stopLogging();
			
			Drivetrain.getInstance().zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			
			disabledLooper.start();
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledPeriodic() {
	}
}
