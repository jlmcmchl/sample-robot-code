/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package net.teamrush27.frc2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import net.teamrush27.frc2019.auto.AutoModeExecutor;
import net.teamrush27.frc2019.auto.creators.AutoModeSelector;
import net.teamrush27.frc2019.base.JoysticksAndGamepadInterface;
import net.teamrush27.frc2019.base.OperatorInterface;
import net.teamrush27.frc2019.constants.RobotConfiguration;
import net.teamrush27.frc2019.constants.RobotConfigurationFactory;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.SubsystemManager;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.LED;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Robot extends TimedRobot {

	private static final Logger LOG = LogManager.getLogger(Robot.class);

	public static final RobotConfiguration ROBOT_CONFIGURATION = RobotConfigurationFactory.getRobotConfiguration();

	private OperatorInterface operatorInterface = JoysticksAndGamepadInterface.getInstance();

	private LED led = LED.getInstance();
	private Drivetrain drivetrain = Drivetrain.getInstance();

	private final SubsystemManager subsystemManager = new SubsystemManager(drivetrain, led);

	private final Looper enabledLooper = new Looper();
	private final Looper disabledLooper = new Looper();

	private AutoModeExecutor autoModeExecutor;

	private boolean autoRan;

	/**
	 * Called <i>once</i> when the robot is first started
	 */
	@Override
	public void robotInit() {
		LOG.info("robotInit()");

		// Sets up the subsystem manager
		subsystemManager.registerEnabledLoops(enabledLooper);
		subsystemManager.registerDisabledLoops(disabledLooper);

		// Disables LiveWindow telemetry (takes lots of cpu cycles)
		LiveWindow.disableAllTelemetry();

		// Sets up the auto mode executor and selector
		autoModeExecutor = new AutoModeExecutor();
		AutoModeSelector.initAutoModeSelector();

		drivetrain.zeroSensors();
	}

	/**
	 * Called <i>every</i> time a control message from the driverstation is received
	 */
	@Override
	public void robotPeriodic() {
		// Update the SmartDashboard
		subsystemManager.outputToSmartDashboard();
		AutoModeSelector.update();
	}

	/**
	 * Called <i>once</i> when autonomous is first started
	 */
	@Override
	public void autonomousInit() {
		LOG.info("autonomousInit()");

		// set leds to enabled
		led.setWantedState(LED.WantedState.ENABLED);

		// stop running the disabled code
		disabledLooper.stop();

		// start running the enabled code
		enabledLooper.start();

		// grab the selected auto mode and start it
		autoModeExecutor.setAutoMode(AutoModeSelector.getSelectedAutoMode());
		//autoModeExecutor.start();

		// tell the program the auto mode has been ran
		autoRan = true;
	}

	/**
	 * Called <i>every</i> time a control message from the driverstation is received
	 * during autonomous mode
	 */
	@Override
	public void autonomousPeriodic() {
		// If we're interrupted by driver's input
		if (operatorInterface.wantsAutoStop()) {
			// stop the auto mode
			autoModeExecutor.stop();

			// clear any pressed buttons
			operatorInterface.clear();
		}

		// If the auto mode is not active
		if (!autoModeExecutor.isActive()) {
			// run standard driver control
			driverControl();
		}
	}

	/**
	 * Called <i>once</i> when teleop is first started either after the transition
	 * from autonomous to teleop during a real/practice match or when from disabled
	 * to teleop during testing / practice
	 */
	@Override
	public void teleopInit() {
		LOG.info("teleopInit()");

		// set leds to enabled
		led.setWantedState(LED.WantedState.ENABLED);

		// stop running the disabled code
		disabledLooper.stop();

		// start running the enabled code
		enabledLooper.start();

		// recreate the auto mode executor to make sure auto mode is stopped
		autoModeExecutor = new AutoModeExecutor();
	}

	/**
	 * Called <i>every</i> time a control message from the driverstation is received
	 * during teleop mode
	 */
	@Override
	public void teleopPeriodic() {
		// run standard drive control
		driverControl();
	}

	/**
	 * Called <i>once</i> when test mode is first started
	 */
	@Override
	public void testInit() {
		LOG.info("testInit()");

		// stop running the enabled code
		enabledLooper.stop();

		// stop running the disabled code
		disabledLooper.stop();
	}

	/**
	 * Called <i>every</i> time a control message from the driverstation is received
	 * during test mode
	 */
	@Override
	public void testPeriodic() {
	}

	/**
	 * Called <i>once</i> when disabled is first started either during the
	 * transition from autonomous to teleop during a real/practice match or when
	 * disabled during testing / practice
	 */
	@Override
	public void disabledInit() {
		LOG.info("disabledInit()");

		// set the leds to disabled
		led.setWantedState(LED.WantedState.DISABLED);

		// if auto mode was running, stop it
		if (autoModeExecutor.isActive()) {
			autoModeExecutor.stop();
		}

		// start running the disabled code
		disabledLooper.start();

		// stop running the enabled code
		enabledLooper.stop();

	}

	/**
	 * Called <i>every</i> time a control message from the driverstation is received
	 * during disabled mode
	 */
	@Override
	public void disabledPeriodic() {
	}

	/**
	 * Contains standard driver controls shared between teleop and auto (hybrid)
	 * modes
	 */
	private void driverControl() {
		DriveCommand command = operatorInterface.getTankCommand();

		drivetrain.setOpenLoop(command.getLeftDriveInput(), command.getRightDriveInput());
	}
}
