/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package net.teamrush27.frc2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import net.teamrush27.frc2019.auto.AutoModeExecutor;
import net.teamrush27.frc2019.auto.modes.TestMode;
import net.teamrush27.frc2019.base.JoysticksAndGamepadInterface;
import net.teamrush27.frc2019.base.OperatorInterface;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.managers.SuperstructureManager;
import net.teamrush27.frc2019.managers.SuperstructureManager.WantedState;
import net.teamrush27.frc2019.subsystems.SubsystemManager;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.subsystems.impl.LED;
import net.teamrush27.frc2019.subsystems.impl.RobotStateEstimator;
import net.teamrush27.frc2019.subsystems.impl.SpiderLegs;
import net.teamrush27.frc2019.subsystems.impl.Wrist;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.TelemetryUtil;
import net.teamrush27.frc2019.util.crash.CrashTracker;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Robot extends TimedRobot {
	
	private RobotStateEstimator robotStateEstimator = RobotStateEstimator.getInstance();
	private Drivetrain drivetrain = Drivetrain.getInstance();
	private Arm arm = Arm.getInstance();
	private Gripper gripper = Gripper.getInstance();
	private Wrist wrist = Wrist.getInstance();
	private SpiderLegs spiderLegs = SpiderLegs.getInstance();
	private OperatorInterface operatorInterface = JoysticksAndGamepadInterface.getInstance();
	private LED led = LED.getInstance();
	private final SuperstructureManager superman = SuperstructureManager.getInstance();
	private final SubsystemManager subsystemManager = new SubsystemManager(drivetrain, gripper,
		spiderLegs, wrist, arm, led, superman);
	private final Looper enabledLooper = new Looper();
	private final Looper disabledLooper = new Looper();
	
	private final Logger LOG = LogManager.getLogger(Robot.class);
	
	private AutoModeExecutor autoModeExecutor;
	
	@Override
	public void robotInit() {
		subsystemManager.registerEnabledLoops(enabledLooper);
		subsystemManager.registerDisabledLoops(disabledLooper);
		TrajectoryGenerator.getInstance().generateTrajectories();
		superman.zeroSensors();
	}
	
	@Override
	public void robotPeriodic() {
		//subsystemManager.outputToSmartDashboard();
		//enabledLooper.outputToSmartDashboard();
		LOG.trace("rot: {} ext: {} wrist: {}", arm.getArmState().getRotationInDegrees(),
			arm.getArmState().getExtensionInInches(), wrist.getEncoderAngle());
	}
	
	@Override
	public void autonomousInit() {
		led.setWantedState(LED.WantedState.ENABLED);
		disabledLooper.stop();
//		drivetrain.startLogging();
//		subsystemManager.startLogging();
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
		led.setWantedState(LED.WantedState.ENABLED);
		disabledLooper.stop();
		enabledLooper.start();
		
		arm.setWantedState(Arm.WantedState.CLOSED_LOOP);
		spiderLegs.setWantedState(SpiderLegs.WantedState.OFF);
		gripper.setWantedState(Gripper.WantedState.OFF);
		wrist.setWantedState(Wrist.WantedState.CLOSED_LOOP);
		drivetrain.shift(true);
		drivetrain.setOpenLoop(DriveCommand.defaultCommand());
//		arm.setClosedLoopInput(new ArmInput(0d, 0d));

//		subsystemManager.startLogging();
	}
	
	boolean wantedClimb = false;
	
	@Override
	public void teleopPeriodic() {
		
		// bail everything if we're climbing
		if (operatorInterface.wantsPreClimb() && !operatorInterface.wantsClimb()) {
			spiderLegs.setWantedState(SpiderLegs.WantedState.PENDING_CLIMB);
			superman.setWantedState(SuperstructureManager.WantedState.CLIMB, true);
		} else if (operatorInterface.wantsClimb()) {
			drivetrain.shift(false);
			superman.setWantedState(SuperstructureManager.WantedState.CLIMB);
			spiderLegs.setWantedState(SpiderLegs.WantedState.CLIMB);
		} else {
			
			if (operatorInterface.wantsStow()) {
				superman.setWantedState(WantedState.STOW, operatorInterface.getWantsInvert());
			} else if (operatorInterface.wantsLevel1HumanLoad()) {
				superman.setWantedState(WantedState.HATCH_HUMAN_PICKUP,
					operatorInterface.getWantsInvert());
			} else if (operatorInterface.wantsGroundPickup()) {
				superman.setWantedState(WantedState.CARGO_GROUND_PICKUP,
					operatorInterface.getWantsInvert());
			} else if (operatorInterface.getWantsCargoShip()) {
				superman.setWantedState(WantedState.CARGO_SHIP, operatorInterface.getWantsInvert());
			} else if (operatorInterface.wantsLevel2()) {
				superman
					.setWantedState(WantedState.ROCKET_LEVEL_2, operatorInterface.getWantsInvert());
			} else if (operatorInterface.wantsLevel3()) {
				superman
					.setWantedState(WantedState.ROCKET_LEVEL_3, operatorInterface.getWantsInvert());
			}
			
			if (operatorInterface.getShift()) {
				drivetrain.shift();
			}
			
			if (operatorInterface.getWantManipulateCargo()) {
				gripper.transitionCargo();
			} else if (operatorInterface.getWantManipulateHatch()) {
				gripper.transitionHatch();
			}
		}
		
		if (spiderLegs.shouldDrive()) {
			drivetrain.setOpenLoop(new DriveCommand(-.3, -.3, true));
		} else {
			drivetrain.setOpenLoop(operatorInterface.getTankCommand());
		}
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
		led.setWantedState(LED.WantedState.DISABLED);
		wantedClimb = false;
		SmartDashboard.putString("Match Cycle", "DISABLED");
		try {
			TelemetryUtil.getInstance().writeToFile("/media/sda/logs/telemetry.csv");
		} catch (IOException e) {
			LOG.error("could not write telemetry", e);
		}
		
		if (autoModeExecutor != null) {
			autoModeExecutor.stop();
		}
		
		try {
			CrashTracker.logDisabledInit();
			enabledLooper.stop();

			//drivetrain.startLogging();

//			subsystemManager.stopLogging();
//			drivetrain.stopLogging();
			//robotStateEstimator.stopLogging();

//			Drivetrain.getInstance().zeroSensors();
//			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			
			wrist.zeroSensors();
			disabledLooper.start();
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		spiderLegs.zeroSensors();
		arm.zeroSensors();
		
		if(operatorInterface.getWantManipulateHatch() && operatorInterface.getWantManipulateCargo()){
			drivetrain.resetArmPosition(0);
		}
	}
}
