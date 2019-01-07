package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.SPI;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.DriveConstants;
import net.teamrush27.frc2019.constants.FollowingConstants;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.motion.DistancePathFollower;
import net.teamrush27.frc2019.wrappers.CANTalonFactory;
import net.teamrush27.frc2019.wrappers.NavX;

/**
 * Drivetrain Subsystem (Adapted from 254 Drive.java)
 *
 * @author team254
 * @author cyocom
 */

public class Drivetrain extends Subsystem {
	
	private static Drivetrain instance = new Drivetrain();
	
	private final TalonSRX leftMaster;
	private final TalonSRX leftSlave;

	private final TalonSRX rightMaster;
	private final TalonSRX rightSlave;

	// new
	private double timeSinceModeSwitch;
	
	private final NavX navX;
	
	private DriveMode driveMode;
	private final DistancePathFollower leftEncoderFollower;
	private final DistancePathFollower rightEncoderFollower;
	
	private boolean brakeMode;
	
	private boolean modeChanged = false;
	
	private final Loop loop = new Loop() {
		
		private DriveMode lastMode = DriveMode.OPEN_LOOP;
		
		/**
		 * @author team254
		 */
		@Override
		public void onStart(double timestamp) {
			synchronized (Drivetrain.this) {
				setOpenLoop(DriveCommand.defaultCommand());
				setBrakeMode(false);
				navX.reset();
			}
		}
		
		/**
		 * @author team254
		 */
		@Override
		public void onLoop(double timestamp) {
			synchronized (Drivetrain.this) {
				if (driveMode != lastMode) {
					System.out.println("DriveMode changed from " + lastMode + " to " + driveMode);
					timeSinceModeSwitch = timestamp;
					lastMode = driveMode;
					modeChanged = true;
				} else {
					modeChanged = false;
				}
				
				switch (driveMode) {
					case OPEN_LOOP:
						break;
					default:
						System.out.println("Unexpected drive mode: " + driveMode);
						break;
				}
			}
		}
		
		/**
		 * @author team254
		 */
		@Override
		public void onStop(double timestamp) {
			stop();
		}
		
	};
	
	/**
	 * @author team254
	 */
	public Drivetrain() {
		leftMaster = CANTalonFactory.createDefaultTalon(RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
		leftMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		
		leftSlave = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_LEFT_SLAVE_1_CAN_ID,
			RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
		leftSlave.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		leftSlave.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftSlave.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		
		rightMaster = CANTalonFactory.createDefaultTalon(RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
		rightMaster.setInverted(true);
		rightMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		rightSlave = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_RIGHT_SLAVE_1_CAN_ID,
			RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
		rightSlave.setInverted(true);
		rightSlave.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		rightSlave.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightSlave.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		setCurrentLimiting(false);
		
		navX = new NavX(SPI.Port.kMXP);
		
		brakeMode = true;
		setBrakeMode(false);
		
		leftEncoderFollower = new DistancePathFollower(true);
		leftEncoderFollower.configurePIDVA(
			FollowingConstants.P,
			FollowingConstants.I,
			FollowingConstants.D,
			FollowingConstants.V,
			FollowingConstants.A
		);
		
		rightEncoderFollower = new DistancePathFollower(false);
		rightEncoderFollower.configurePIDVA(
			FollowingConstants.P,
			FollowingConstants.I,
			FollowingConstants.D,
			FollowingConstants.V,
			FollowingConstants.A
		);
	}
	
	private void setCurrentLimiting(boolean shouldCurrentLimit) {
		leftMaster.enableCurrentLimit(shouldCurrentLimit);
		leftSlave.enableCurrentLimit(shouldCurrentLimit);
		rightMaster.enableCurrentLimit(shouldCurrentLimit);
		rightSlave.enableCurrentLimit(shouldCurrentLimit);
	}
	
	/**
	 * @author team254
	 */
	private void setBrakeMode(boolean requestedBrakeMode) {
		if (brakeMode != requestedBrakeMode) {
			brakeMode = requestedBrakeMode;
			rightMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightSlave.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftSlave.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		}
	}
	
	@Override
	public void outputToSmartDashboard() {
	}
	
	@Override
	public void stop() {
		setOpenLoop(DriveCommand.defaultCommand());
	}
	
	/**
	 * Modified from 254's to ditch DriveSignal transfer object
	 *
	 * @author team254
	 * @author cyocom
	 */
	public synchronized void setOpenLoop(DriveCommand driveCommand, boolean brakeMode) {
		if (driveMode != DriveMode.OPEN_LOOP) {// && driveMode != DriveMode.CLIMB) {
			driveMode = DriveMode.OPEN_LOOP;
			setBrakeMode(brakeMode);
			setCurrentLimiting(true);
		}
		leftMaster.set(ControlMode.PercentOutput, -driveCommand.getLeftDriveInput());
		rightMaster.set(ControlMode.PercentOutput, -driveCommand.getRightDriveInput());
	}
	
	public synchronized void setOpenLoop(DriveCommand driveCommand) {
		setOpenLoop(driveCommand, false);
	}
	
	@Override
	public void zeroSensors() {
		resetEncoders();
		navX.zeroYaw();
	}
	
	public synchronized void resetEncoders() {
	}
	
	public static Drivetrain getInstance() {
		return instance;
	}
	
	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	public synchronized Rotation2d getGyroAngle() {
		return navX.getYaw();
	}
	
	public synchronized void setGyroAngle(Rotation2d rotation) {
		navX.reset();
		navX.setAngleAdjustment(rotation);
	}
	
	public void defaultState() {
		leftMaster.setNeutralMode(NeutralMode.Coast);
		rightMaster.setNeutralMode(NeutralMode.Coast);
	}
	
	@Override
	public void test() {
	
	}
}
