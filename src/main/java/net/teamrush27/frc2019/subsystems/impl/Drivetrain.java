package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import jaci.pathfinder.Pathfinder;
//import jaci.pathfinder.Trajectory;
import java.util.Objects;
import net.teamrush27.frc2019.base.RobotMap;
//import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.constants.ChezyConstants;
import net.teamrush27.frc2019.constants.DriveConstants;
import net.teamrush27.frc2019.constants.FollowingConstants;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;
import net.teamrush27.frc2019.subsystems.impl.util.DriveUtils;
import net.teamrush27.frc2019.util.follow.Lookahead;
import net.teamrush27.frc2019.util.follow.Path;
import net.teamrush27.frc2019.util.follow.PathFollower;
import net.teamrush27.frc2019.util.follow.PathFollower254Impl;
//import net.teamrush27.frc2019.util.interpolate.InterpolatingTrajectory;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.RigidTransform2d;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Twist2d;
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
	private final TalonSRX leftSlave1;

	private final TalonSRX rightMaster;
	private final TalonSRX rightSlave1;


	// new
	private double timeSinceModeSwitch;
	
	
	private final NavX navX;
	
	private DriveMode driveMode;
	//private RobotState robotState = RobotState.getInstance();
	private final DistancePathFollower leftEncoderFollower;
	private final DistancePathFollower rightEncoderFollower;
	//private Trajectory trajectory = null;
	
	private boolean brakeMode;
	
	private Rotation2d targetHeading = new Rotation2d();
	private boolean isOnTarget = false;
	
	//private final DoubleSolenoid ptoSolenoid;
	//private final DoubleSolenoid ratchetSolenoid;
	
	private boolean modeChanged = false;
	private boolean isTrajectoryInverted = false;
	
	private final Loop loop = new Loop() {
		
		long count = 0;
		private DriveMode lastMode = DriveMode.OPEN_LOOP;
		
		/**
		 * @author team254
		 */
		@Override
		public void onStart(double timestamp) {
			synchronized (Drivetrain.this) {
				setOpenLoop(DriveCommand.defaultCommand());
				setBrakeMode(false);
				//setVelocitySetpoint(0, 0);
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
					/*case VELOCITY_SETPOINT:
						break;
					case PATH_FOLLOWING:
						if (leftEncoderFollower != null) {
							updatePathFollower(timestamp);
						}
						break;
					case TURN_TO_HEADING:
						updateTurnToHeading(timestamp);
						break;
					case CLIMB:
						handleClimb(timestamp);
						break;
					case CHEZY_PATH_FOLLOWING:
						updateChezyPathFollower(timestamp);
						break;*/
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
	private Path currentPath = null;
	private PathFollower pathFollower = null;
	
	/**
	 * @author team254
	 */
	public Drivetrain() {
		leftMaster = CANTalonFactory.createDefaultTalon(RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configVelocityMeasurementWindow(32, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.setSensorPhase(true);
		
		leftMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		leftMaster.configMotionCruiseVelocity(DriveUtils.inchesPerSecondToEncoderCountPer100ms(10*12), RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configMotionAcceleration(DriveUtils.inchesPerSecondToEncoderCountPer100ms(15*12), RobotConstants.TALON_CONFIG_TIMEOUT);
		
		leftSlave1 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_LEFT_SLAVE_1_CAN_ID,
			RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
		leftSlave1.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		leftSlave1.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftSlave1.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		
		/*leftSlave2 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_LEFT_SLAVE_2_CAN_ID,
			RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
		leftSlave2.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		leftSlave2.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftSlave2.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		*/
		
		rightMaster = CANTalonFactory.createDefaultTalon(RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configVelocityMeasurementWindow(32, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.setSensorPhase(true);
		rightMaster.setInverted(true);
		rightMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		rightMaster.configMotionCruiseVelocity(DriveUtils.inchesPerSecondToEncoderCountPer100ms(10*12), RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configMotionAcceleration(DriveUtils.inchesPerSecondToEncoderCountPer100ms(15*12), RobotConstants.TALON_CONFIG_TIMEOUT);
		
		rightSlave1 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_RIGHT_SLAVE_1_CAN_ID,
			RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
		rightSlave1.setInverted(true);
		rightSlave1.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		rightSlave1.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightSlave1.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		/*rightSlave2 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_RIGHT_SLAVE_2_CAN_ID,
			RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
		rightSlave2.setInverted(true);
		rightSlave2.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,RobotConstants.TALON_CONFIG_TIMEOUT);
		rightSlave2.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightSlave2.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT, RobotConstants.TALON_CONFIG_TIMEOUT);
		*/
		
		setCurrentLimiting(false);
		reloadGains();
		
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
		
		/*ptoSolenoid = new DoubleSolenoid(RobotMap.PTO_MODULE, RobotMap.PTO_ENGAGED_CHANNEL,
			RobotMap.PTO_DISENGAGED_CHANNEL);
		ratchetSolenoid = new DoubleSolenoid(RobotMap.RATCHET_MODULE,
			RobotMap.RATCHET_ENGAGED_CHANNEL, RobotMap.RATCHET_DISENGAGED_CHANNEL);
		ptoSolenoid.set(Value.kReverse);
		ratchetSolenoid.set(Value.kReverse);*/
		
	}
	
	private void setCurrentLimiting(boolean shouldCurrentLimit) {
		leftMaster.enableCurrentLimit(shouldCurrentLimit);
		leftSlave1.enableCurrentLimit(shouldCurrentLimit);
		//leftSlave2.enableCurrentLimit(shouldCurrentLimit);
		rightMaster.enableCurrentLimit(shouldCurrentLimit);
		rightSlave1.enableCurrentLimit(shouldCurrentLimit);
		//rightSlave2.enableCurrentLimit(shouldCurrentLimit);
	}
	
	/*protected void updateTurnToHeading(double timestamp) {
		
		if(modeChanged){
			leftMaster.selectProfileSlot(2,0);
			rightMaster.selectProfileSlot(2,0);
		}
		
		final Rotation2d field_to_robot = robotState.getLatestFieldToVehicle().getValue().getRotation();

		// Figure out the rotation necessary to turn to face the goal.
		final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(targetHeading);

		// Check if we are on target
		final double kGoalPosTolerance = 5; // degrees
		final double kGoalVelTolerance = 300.0; // inches per second
		if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
				&& Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
				&& Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
			// Use the current setpoint and base lock.
			isOnTarget = true;
			updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
			return;
		}

		KinematicsUtils.DriveVelocity wheel_delta = KinematicsUtils
				.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
		updatePositionSetpoint(wheel_delta.leftVelocity + getLeftDistanceInches(),
				wheel_delta.rightVelocity + getRightDistanceInches());
	}*/
	
	/**
	 * @author team254
	 */
	private void setBrakeMode(boolean requestedBrakeMode) {
		if (brakeMode != requestedBrakeMode) {
			brakeMode = requestedBrakeMode;
			rightMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			//rightSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			//leftSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		}
	}
	
	/**
	 * @author team254
	 */
	public synchronized void reloadGains() {
		double startTime = Timer.getFPGATimestamp();
		leftMaster.config_kP(0, DriveConstants.PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kI(0, DriveConstants.PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kD(0, DriveConstants.PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kF(0, DriveConstants.PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster
			.config_IntegralZone(0, DriveConstants.PID_I_ZONE, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.configClosedloopRamp(DriveConstants.PID_RAMP_RATE,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		
		rightMaster.config_kP(0, DriveConstants.PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kI(0, DriveConstants.PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kD(0, DriveConstants.PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kF(0, DriveConstants.PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster
			.config_IntegralZone(0, DriveConstants.PID_I_ZONE, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configClosedloopRamp(DriveConstants.PID_RAMP_RATE,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kP(1, ChezyConstants.PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kI(1, ChezyConstants.PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kD(1, ChezyConstants.PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kF(1, ChezyConstants.PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster
			.config_IntegralZone(1, ChezyConstants.PID_I_ZONE, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		rightMaster.config_kP(1, ChezyConstants.PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kI(1, ChezyConstants.PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kD(1, ChezyConstants.PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kF(1, ChezyConstants.PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster
			.config_IntegralZone(1, ChezyConstants.PID_I_ZONE, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kP(2, ChezyConstants.ROTATE_PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kD(2, ChezyConstants.ROTATE_PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kI(2, ChezyConstants.ROTATE_PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster.config_kF(2, ChezyConstants.ROTATE_PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
		leftMaster
			.config_IntegralZone(2, ChezyConstants.ROTATE_PID_I_ZONE, RobotConstants.TALON_CONFIG_TIMEOUT);
		
		rightMaster.config_kP(2, ChezyConstants.ROTATE_PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kI(2, ChezyConstants.ROTATE_PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kD(2, ChezyConstants.ROTATE_PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.config_kF(2, ChezyConstants.ROTATE_PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster
			.config_IntegralZone(2, ChezyConstants.ROTATE_PID_I_ZONE, RobotConstants.TALON_CONFIG_TIMEOUT);
		System.out.println("reloading gains took " + (Timer.getFPGATimestamp() - startTime) + " seconds");
		
		
		leftMaster.selectProfileSlot(0,0);
		rightMaster.selectProfileSlot(0,0);
	}
	
	/**
	 * @author team254
	 */
	public synchronized void reloadChezyGains() {
		leftMaster.selectProfileSlot(1,0);
		rightMaster.selectProfileSlot(1,0);
		leftMaster.configClosedloopRamp(ChezyConstants.PID_RAMP_RATE,
			RobotConstants.TALON_CONFIG_TIMEOUT);
		rightMaster.configClosedloopRamp(ChezyConstants.PID_RAMP_RATE,
			RobotConstants.TALON_CONFIG_TIMEOUT);
	}



//	private double leftMax = 0;
//	private double rightMax = 0;
	
	@Override
	public void outputToSmartDashboard() {
//		SmartDashboard.putBoolean("climb",DriveMode.CLIMB.equals(driveMode));
//		double currentleftMax = Math.max(leftMaster.getOutputCurrent(),leftMax);
//		double currentRightMax = Math.max(rightMaster.getOutputCurrent(),rightMax);
//
//		if(!Objects.equals(leftMax, currentleftMax) || !Objects.equals(rightMax,currentRightMax)){
//			System.out.println("L: " + currentleftMax + " - R: " + currentRightMax);
//			leftMax = currentleftMax;
//			rightMax = currentRightMax;
//		}
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
		leftMaster.setSelectedSensorPosition(0, 0, 5);
		rightMaster.setSelectedSensorPosition(0, 0, 5);
		leftSlave1.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		//leftSlave2.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		rightSlave1.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
		//rightSlave2.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
	}
	
	public static Drivetrain getInstance() {
		return instance;
	}
	
	private double climbVal = 0;
	
	
	private double startPitch = 0;
	
/*	private void handleClimb(double timestamp) {
		if(modeChanged){
			startPitch = navX.getPitch();
			setCurrentLimiting(false);
		}
		
		ptoSolenoid.set(Value.kForward);
		ratchetSolenoid.set(Value.kForward);
		
		double rotation = (navX.getPitch() - startPitch) * .1;
		double leftVal;
		double rightVal;
		
		if (Math.signum(rotation) > 0) {
			leftVal = -(climbVal) - rotation;
			rightVal = -(climbVal);
		} else {
			leftVal = -(climbVal);
			rightVal = -(climbVal) + rotation;
		}
		
		setOpenLoop(Math.min(leftVal, 0), Math.min(rightVal, 0), false);
	}*/
	
	
	/**
	 * Called periodically when the robot is in path following mode. Updates the path follower with
	 * the robots latest pose, distance driven, and velocity, the updates the wheel velocity
	 * setpoints.
	 */
	// JACI'S PATHFINDER
/*	private void updatePathFollower(double timestamp) {
		double leftSensorPosition =
			(isTrajectoryInverted ? -1 : 1) *
			DriveUtils.encoderCountToInches(
				leftMaster.getSelectedSensorPosition(0));
		double leftSensorVelocity =
			(isTrajectoryInverted ? -1 : 1) *
			DriveUtils.encoderCountToInches(
				leftMaster.getSelectedSensorVelocity(0)) * 10;
		
		double rightSensorPosition =
			(isTrajectoryInverted ? -1 : 1) *
			DriveUtils.encoderCountToInches(
				rightMaster.getSelectedSensorPosition(0));
		double rightSensorVelocity =
			(isTrajectoryInverted ? -1 : 1) *
			DriveUtils.encoderCountToInches(
				rightMaster.getSelectedSensorVelocity(0)) * 10;
		
		double leftInchesPerSecond =
			(isTrajectoryInverted ? -1 : 1) *
			leftEncoderFollower.calculate(leftSensorPosition, leftSensorVelocity);
		double rightInchesPerSecond =
			(isTrajectoryInverted ? -1 : 1) *
			rightEncoderFollower.calculate(rightSensorPosition, rightSensorVelocity);
		
		
		if (!leftEncoderFollower.isFinished() && !rightEncoderFollower.isFinished()) {
			double val1 = (navX.getRawYawDegrees() + 360.0) % 360.0;
			
			double requestedHeading = Pathfinder.r2d(rightEncoderFollower.getHeading());
			requestedHeading = isTrajectoryInverted ? requestedHeading - 180 : requestedHeading;
			double val2 = (requestedHeading + 360.0) % 360.0;
			double err = (val2 - val1 + 360) % 360;
			err = err < 180 ? err : err - 360;
			
			double deltaV =
				(RobotConstants.TRACK_WIDTH * err / (2 * RobotConstants.SCRUB_FACTOR));
			
//			System.out.println(
//				String.format(
//					"1\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s",
//					isTrajectoryInverted,
//					leftSensorPosition,
//					leftSensorVelocity,
//					leftInchesPerSecond,
//					rightSensorPosition,
//					rightSensorVelocity,
//					rightInchesPerSecond,
//					requestedHeading,
//					navX.getRawYawDegrees(),
//					deltaV));
			
			double leftSetpoint = leftInchesPerSecond - deltaV;
			double rightSetpoint = rightInchesPerSecond + deltaV;
			
			updateVelocitySetpoint(leftSetpoint, rightSetpoint);
			
			
		} else {
			updateVelocitySetpoint(0, 0);
		}
	}*/
	
	
	/**
	 * Start up velocity mode. This sets the drive train in high gear as well.
	 */
/*	public synchronized void setVelocitySetpoint(double leftInchesPerSecond,
		double rightInchesPerSecond) {
		configureTalonsForSpeedControl();
		driveMode = DriveMode.VELOCITY_SETPOINT;
		updateVelocitySetpoint(leftInchesPerSecond, rightInchesPerSecond);
	}*/
	
	/**
	 * Configures talons for position control <p><i>(Modified for new DriveMode enum)</i></p>
	 *
	 * @author cyocom
	 * @author team254
	 */
	private void configureTalonsForPositionControl() {
		if (!driveMode.getRequestedControlMode().equals(ControlMode.MotionMagic)) {
			// We entered a position control state.
			setBrakeMode(true);
		}
	}
	
	/**
	 * Adjust position setpoint (if already in position mode)
	 *
	 * @param leftInches
	 * @param rightInches
	 */
/*	private synchronized void updatePositionSetpoint(double leftInches, double rightInches) {
		if (ControlMode.MotionMagic.equals(driveMode.getRequestedControlMode())) {
			leftMaster.set(
				ControlMode.MotionMagic,
				DriveUtils.inchesToEncoderCount(leftInches));
			rightMaster.set(
				ControlMode.MotionMagic,
				DriveUtils.inchesToEncoderCount(rightInches));
		} else {
			System.out.println("Hit a bad position control state");
			setOpenLoop(0,0);
		}
	}*/
	
	
	/**
	 * Configures talons for velocity control <p><i>(Modified for new DriveMode enum)</i></p>
	 *
	 * @author cyocom
	 * @author team254
	 */
/*	private void configureTalonsForSpeedControl() {
		if (driveMode == null || !Objects
			.equals(driveMode.getRequestedControlMode(), ControlMode.Velocity)) {
			// We entered a velocity control state.
			setBrakeMode(true);
		}
	}*/
	
	/**
	 * Adjust Velocity setpoint (if already in velocity mode) <p><i>(Modified for new DriveMode
	 * enum)</i></p>
	 */
/*	private synchronized void updateVelocitySetpoint(double leftInchesPerSecond,
		double rightInchesPerSecond) {
		
		if (driveMode.getRequestedControlMode().equals(ControlMode.Velocity)) {
			double leftValue = DriveUtils
				.inchesPerSecondToEncoderCountPer100ms(leftInchesPerSecond);
			double rightValue = DriveUtils
				.inchesPerSecondToEncoderCountPer100ms(rightInchesPerSecond);
			
			leftMaster.set(ControlMode.Velocity, leftValue);
			rightMaster.set(ControlMode.Velocity, rightValue);
			if (trajectory != null) {
				if (!leftEncoderFollower.isFinished()) {
				}
			}
		} else {
			System.out.println(String.format("Hit a bad velocity control state %s %s",
				driveMode.getRequestedControlMode(), driveMode));
			leftMaster.set(ControlMode.Velocity, 0);
			rightMaster.set(ControlMode.Velocity, 0);
		}
	}*/
	
	public double getLeftVelocityInchesPerSec() {
		return DriveUtils.encoderCountToInches(leftMaster.getSelectedSensorVelocity(0)) * 10;
	}
	
	public double getRightVelocityInchesPerSec() {
		return DriveUtils.encoderCountToInches(rightMaster.getSelectedSensorVelocity(0)) * 10;
	}
	
	public double getLeftDistanceInches() {
		return DriveUtils.encoderCountToInches(leftMaster.getSelectedSensorPosition(0));
	}
	
	public double getRightDistanceInches() {
		return DriveUtils.encoderCountToInches(rightMaster.getSelectedSensorPosition(0));
	}
	
	public int getLeftEncoderRotations() {
		return leftMaster.getSelectedSensorPosition(0);
	}
	
	public int getRightEncoderRotations() {
		return rightMaster.getSelectedSensorPosition(0);
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
	
/*	public synchronized boolean isDoneWithPath() {
//		System.out.println(String.format("Mode: %s\tLeft: %s\tRight: %s", driveMode, leftEncoderFollower.isFinished(), rightEncoderFollower.isFinished()));
		
		if (driveMode == DriveMode.PATH_FOLLOWING && leftEncoderFollower != null) {
			return leftEncoderFollower.isFinished();
		} else if(driveMode == DriveMode.CHEZY_PATH_FOLLOWING && pathFollower != null) {
			return pathFollower.isFinished();
		} else {
				System.out.println("Robot is not in path following mode");
				return true;
		}
	}*/
	
/*	public synchronized void setWantClimb() {
		driveMode = DriveMode.CLIMB;
		
	}*/
	
/*	private void updateChezyPathFollower(double timestamp) {
		if(modeChanged){
			reloadChezyGains();
		}
		RigidTransform2d robotPosition = robotState.getLatestFieldToVehicle().getValue();
		Twist2d twistCommand = pathFollower.update(timestamp, robotPosition,
			RobotState.getInstance().getDistanceDriven(),
			RobotState.getInstance().getPredictedVelocity().deltaX);
		
		if (!pathFollower.isFinished()) {
			KinematicsUtils.DriveVelocity setpoint = KinematicsUtils
				.inverseKinematics(twistCommand);
			updateVelocitySetpoint(setpoint.leftVelocity, setpoint.rightVelocity);
			
//			System.out.println("in path following : "
//				+ setpoint.leftVelocity + " - "
//				+ setpoint.rightVelocity + " - "
//				+ robotPosition.getTranslation().x() + " - "
//				+ robotPosition.getTranslation().y() + " - "
//				+ robotPosition.getRotation().getDegrees());
		} else {
			updateVelocitySetpoint(0, 0);
		}
	}*/
	
	
/*	public synchronized void setWantClimbUp(double climbVal){
		this.climbVal = climbVal;
	}*/
	
/*	public synchronized void setWantDrivePath(InterpolatingTrajectory centerTrajectory,
		InterpolatingTrajectory leftTrajectory, InterpolatingTrajectory rightTrajectory,
		boolean inverted) {
		if (!Objects.equals(trajectory,centerTrajectory) || driveMode != DriveMode.PATH_FOLLOWING) {
			isTrajectoryInverted = inverted;
			
			configureTalonsForSpeedControl();
			resetEncoders();
			this.trajectory = centerTrajectory;
			
			leftEncoderFollower.setTrajectory(leftTrajectory);
			rightEncoderFollower.setTrajectory(rightTrajectory);
			
			driveMode = DriveMode.PATH_FOLLOWING;
		} else {
			setVelocitySetpoint(0, 0);
		}
	}*/
	
	public void defaultState() {
		//ptoSolenoid.set(Value.kReverse);
		//ratchetSolenoid.set(Value.kReverse);
		leftMaster.setNeutralMode(NeutralMode.Coast);
		rightMaster.setNeutralMode(NeutralMode.Coast);
	}
	
/*	public boolean isClimb() {
		return DriveMode.CLIMB.equals(driveMode);
	}*/
	
/*	public synchronized void setWantDrivePath(Path path, boolean reversed, double acceleration) {
		if (!Objects.equals(currentPath,path) || driveMode != DriveMode.CHEZY_PATH_FOLLOWING) {
			configureTalonsForSpeedControl();
			//RobotState.getInstance().resetDistanceDriven();
			pathFollower = new PathFollower254Impl(path, reversed,
				new PathFollower254Impl.Parameters(
					new Lookahead(
						12.0,
						24.0,
						9.0,
						120.0),
					ChezyConstants.INTERIAL_STEERING_GAIN,
					2,
					0.03,
					0,
					1.0,
					0.05,
					120.0,
					acceleration,
					.75,
					50.0,
					5.0));
			driveMode = DriveMode.CHEZY_PATH_FOLLOWING;
			currentPath = path;
		} else {
			setVelocitySetpoint(0, 0);
		}
	}*/

/*	public synchronized void startRotation(Rotation2d heading) {
		this.driveMode = DriveMode.TURN_TO_HEADING;
		this.targetHeading = heading;
		this.isOnTarget = false;
	}

	public synchronized boolean isDoneWithTurn() {
		if (driveMode == driveMode.TURN_TO_HEADING) {
			return isOnTarget;
		} else {
			System.out.println("Robot is not in turn to heading mode");
			return false;
		}
	}*/
	
	@Override
	public void test() {
	
	}
}
