package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Objects;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.base.RobotState;
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
import net.teamrush27.frc2019.util.ReflectingCSVWriter;
import net.teamrush27.frc2019.util.follow.Path;
import net.teamrush27.frc2019.util.follow.PathFollower;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Pose2dWithCurvature;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Twist2d;
import net.teamrush27.frc2019.util.math.Units;
import net.teamrush27.frc2019.util.motion.DistancePathFollower;
import net.teamrush27.frc2019.util.motion.DriveMotionPlanner;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryIterator;
import net.teamrush27.frc2019.util.trajectory.timing.TimedState;
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

  private static final double DRIVE_ENCODER_PPR = 4096.0;

  private final TalonSRX leftMaster;
  private final TalonSRX leftSlave1;
  private final TalonSRX rightMaster;
  private final TalonSRX rightSlave1;

  // new
  private final TalonSRX leftSlave2;
  private final TalonSRX rightSlave2;
  private double timeSinceModeSwitch;

  private final int DRIVE_CONTROL_SLOT = 0;
  private final int VELOCITY_CONTROL_SLOT = 1;
  private final int TURNING_CONTROL_SLOT = 2;

  private final NavX navX;

  private DriveMode driveMode = DriveMode.OPEN_LOOP;
  private RobotState robotState = RobotState.getInstance();
  private Trajectory trajectory = null;
  private DriveMotionPlanner motionPlanner;
  private boolean overrideTrajectory = false;

  private Rotation2d mGyroOffset = Rotation2d.identity();

  private PeriodicIO periodicIO;

  private boolean brakeMode;

  private boolean modeChanged = false;
  private boolean isTrajectoryInverted = false;

  private Rotation2d targetHeading = null;
  private boolean isOnTarget = false;

  private ReflectingCSVWriter<PeriodicIO> CSVWriter = null;

  private final Loop loop = new Loop() {
    private DriveMode lastMode = DriveMode.OPEN_LOOP;

    /**
     * @author team254
     */
    @Override
    public void onStart(double timestamp) {
      synchronized (Drivetrain.this) {
        setOpenLoop(DriveCommand.defaultCommand());
        navX.reset();
//        startLogging();
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
          case VELOCITY_SETPOINT:
            break;
          //         case PATH_FOLLOWING:
          //           if (leftEncoderFollower != null) {
          //             updatePathFollower(timestamp);
          //           }
          //           break;
          case TURN_TO_HEADING:
            updateTurnToHeading(timestamp);
            break;
          //         case CLIMB:
          //           handleClimb(timestamp);
          //           break;
          case CHEZY_PATH_FOLLOWING:
            updateChezyPathFollower(timestamp);
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
      stopLogging();
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
    leftMaster.setInverted(true);

    leftMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster.configMotionCruiseVelocity(DriveUtils.inchesPerSecondToEncoderCountPer100ms(10 * 12),
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configMotionAcceleration(DriveUtils.inchesPerSecondToEncoderCountPer100ms(15 * 12),
        RobotConstants.TALON_CONFIG_TIMEOUT);

    leftSlave1 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_LEFT_SLAVE_1_CAN_ID,
        RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
    leftSlave1.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.setInverted(true);

    leftSlave2 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_LEFT_SLAVE_2_CAN_ID,
        RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
    leftSlave2.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.setInverted(true);

    rightMaster = CANTalonFactory.createDefaultTalon(RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configVelocityMeasurementWindow(32, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.setSensorPhase(true);
    //rightMaster.setInverted(true);
    rightMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster
        .configMotionCruiseVelocity(DriveUtils.inchesPerSecondToEncoderCountPer100ms(10 * 12),
            RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configMotionAcceleration(DriveUtils.inchesPerSecondToEncoderCountPer100ms(15 * 12),
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightSlave1 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_RIGHT_SLAVE_1_CAN_ID,
        RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
    //rightSlave1.setInverted(true);
    rightSlave1.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightSlave2 = CANTalonFactory.createPermanentSlaveTalon(RobotMap.DRIVE_RIGHT_SLAVE_2_CAN_ID,
        RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
    //rightSlave2.setInverted(true);
    rightSlave2.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    setCurrentLimiting(false);
    reloadGains();

    periodicIO = new PeriodicIO();
    navX = new NavX(SPI.Port.kMXP);

    brakeMode = true;
    setBrakeMode(false);

    motionPlanner = new DriveMotionPlanner();

  }

  private void setCurrentLimiting(boolean shouldCurrentLimit) {
    leftMaster.enableCurrentLimit(shouldCurrentLimit);
    leftSlave1.enableCurrentLimit(shouldCurrentLimit);
    leftSlave2.enableCurrentLimit(shouldCurrentLimit);
    rightMaster.enableCurrentLimit(shouldCurrentLimit);
    rightSlave1.enableCurrentLimit(shouldCurrentLimit);
    rightSlave2.enableCurrentLimit(shouldCurrentLimit);
  }

  protected void updateTurnToHeading(double timestamp) {

    if (modeChanged) {
      leftMaster.selectProfileSlot(TURNING_CONTROL_SLOT, 0);
      rightMaster.selectProfileSlot(TURNING_CONTROL_SLOT, 0);
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
      periodicIO.left_turn = getLeftDistanceInches();
      periodicIO.right_turn = getRightDistanceInches();
      return;
    }

    KinematicsUtils.DriveVelocity wheel_delta = KinematicsUtils
        .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
    periodicIO.left_turn = wheel_delta.leftVelocity + getLeftDistanceInches();
    periodicIO.right_turn = wheel_delta.rightVelocity + getRightDistanceInches();
  }

  private synchronized void updatePositionSetpoint(double leftInches, double rightInches) {
    if (ControlMode.MotionMagic.equals(driveMode.getRequestedControlMode())) {
      periodicIO.left_turn = leftInches;
      periodicIO.right_turn = rightInches;
    } else {
      System.out.println("Hit a bad position control state");
      setOpenLoop(DriveCommand.defaultCommand());
    }
  }


  /**
   * @author team254
   */
  private void setBrakeMode(boolean requestedBrakeMode) {
    if (brakeMode != requestedBrakeMode) {
      brakeMode = requestedBrakeMode;
      rightMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
      rightSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
      rightSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
      leftMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
      leftSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
      leftSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    }
  }

  /**
   * @author team254
   */
  public synchronized void reloadGains() {
    double startTime = Timer.getFPGATimestamp();
    leftMaster
        .config_kP(DRIVE_CONTROL_SLOT, DriveConstants.PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster
        .config_kI(DRIVE_CONTROL_SLOT, DriveConstants.PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster
        .config_kD(DRIVE_CONTROL_SLOT, DriveConstants.PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster
        .config_kF(DRIVE_CONTROL_SLOT, DriveConstants.PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster
        .config_IntegralZone(DRIVE_CONTROL_SLOT, DriveConstants.PID_I_ZONE,
            RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configClosedloopRamp(DriveConstants.PID_RAMP_RATE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster
        .config_kP(DRIVE_CONTROL_SLOT, DriveConstants.PID_P, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster
        .config_kI(DRIVE_CONTROL_SLOT, DriveConstants.PID_I, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster
        .config_kD(DRIVE_CONTROL_SLOT, DriveConstants.PID_D, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster
        .config_kF(DRIVE_CONTROL_SLOT, DriveConstants.PID_F, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster
        .config_IntegralZone(DRIVE_CONTROL_SLOT, DriveConstants.PID_I_ZONE,
            RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configClosedloopRamp(DriveConstants.PID_RAMP_RATE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster.config_kP(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kI(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kD(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kF(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster
        .config_IntegralZone(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I_ZONE,
            RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster.config_kP(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kI(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kD(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kF(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster
        .config_IntegralZone(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I_ZONE,
            RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster.config_kP(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kD(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kI(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kF(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster
        .config_IntegralZone(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I_ZONE,
            RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster.config_kP(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kI(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kD(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kF(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster
        .config_IntegralZone(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I_ZONE,
            RobotConstants.TALON_CONFIG_TIMEOUT);

    System.out
        .println("reloading gains took " + (Timer.getFPGATimestamp() - startTime) + " seconds");

    leftMaster.selectProfileSlot(DRIVE_CONTROL_SLOT, 0);
    rightMaster.selectProfileSlot(DRIVE_CONTROL_SLOT, 0);
  }

  /**
   * @author team254
   */
  public synchronized void reloadChezyGains() {
    leftMaster.selectProfileSlot(VELOCITY_CONTROL_SLOT, 0);
    rightMaster.selectProfileSlot(VELOCITY_CONTROL_SLOT, 0);
    leftMaster.configClosedloopRamp(ChezyConstants.PID_RAMP_RATE,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configClosedloopRamp(ChezyConstants.PID_RAMP_RATE,
        RobotConstants.TALON_CONFIG_TIMEOUT);
  }

//	private double leftMax = 0;
//	private double rightMax = 0;

  @Override
  public void outputToSmartDashboard() {
//    SmartDashboard.putBoolean("climb", DriveMode.CLIMB.equals(driveMode));
//		double currentleftMax = Math.max(leftMaster.getOutputCurrent(),leftMax);
//		double currentRightMax = Math.max(rightMaster.getOutputCurrent(),rightMax);
//
//		if(!Objects.equals(leftMax, currentleftMax) || !Objects.equals(rightMax,currentRightMax)){
//			System.out.println("L: " + currentleftMax + " - R: " + currentRightMax);
//			leftMax = currentleftMax;
//			rightMax = currentRightMax;
//		}
    //if (CSVWriter != null) {
    //            CSVWriter.write();
    //        }
  }

  /**
   * Configures talons for velocity control
   */
  public synchronized void setVelocity(DriveCommand signal, DriveCommand feedforward) {
    if (driveMode != DriveMode.CHEZY_PATH_FOLLOWING) {
      // We entered a velocity control state.
      setBrakeMode(true);
      leftMaster.selectProfileSlot(VELOCITY_CONTROL_SLOT, 0);
      rightMaster.selectProfileSlot(VELOCITY_CONTROL_SLOT, 0);
      leftMaster.configNeutralDeadband(0.0, 0);
      rightMaster.configNeutralDeadband(0.0, 0);

      driveMode = DriveMode.CHEZY_PATH_FOLLOWING;
    }
    periodicIO.left_demand = signal.getLeftDriveInput();
    periodicIO.right_demand = signal.getRightDriveInput();
    periodicIO.left_feedforward = feedforward.getLeftDriveInput();
    periodicIO.right_feedforward = feedforward.getRightDriveInput();
  }

  public synchronized void setTrajectory(
      TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    if (motionPlanner != null) {
      overrideTrajectory = false;
      motionPlanner.reset();
      motionPlanner.setTrajectory(trajectory);
      driveMode = DriveMode.CHEZY_PATH_FOLLOWING;
    }
  }

  public boolean isDoneWithTrajectory() {
    if (motionPlanner == null || driveMode != DriveMode.CHEZY_PATH_FOLLOWING) {
      return true;
    }
    return motionPlanner.isDone() || overrideTrajectory;
  }

  public synchronized Rotation2d getHeading() {
    return periodicIO.gyro_heading;
  }

  public synchronized void setHeading(Rotation2d heading) {
    System.out.println("SET HEADING: " + heading.getDegrees());

    mGyroOffset = heading.rotateBy(navX.getYaw().inverse());

    System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

    periodicIO.gyro_heading = heading;
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
  public synchronized void setOpenLoop(DriveCommand command) {
    if (driveMode != DriveMode.OPEN_LOOP) {
      driveMode = DriveMode.OPEN_LOOP;
      setBrakeMode(command.getBrakeMode());
      setCurrentLimiting(true);

      leftMaster.configNeutralDeadband(0.04, 0);
      rightMaster.configNeutralDeadband(0.04, 0);
    }

    periodicIO.left_demand = -command.getLeftDriveInput();
    periodicIO.right_demand = -command.getRightDriveInput();
    periodicIO.left_feedforward = 0.0;
    periodicIO.right_feedforward = 0.0;
  }

  @Override
  public void zeroSensors() {
    resetEncoders();
    navX.zeroYaw();
  }

  public double getLeftEncoderRotations() {
    return -periodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
  }

  public double getRightEncoderRotations() {
    return -periodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
  }

  public double getLeftEncoderDistance() {
    return DriveUtils.rotationsToInches(getLeftEncoderRotations());
  }

  public double getRightEncoderDistance() {
    return DriveUtils.rotationsToInches(getRightEncoderRotations());
  }

  public double getRightVelocityNativeUnits() {
    return periodicIO.right_velocity_ticks_per_100ms;
  }

  public double getRightLinearVelocity() {
    return DriveUtils.rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLeftVelocityNativeUnits() {
    return periodicIO.left_velocity_ticks_per_100ms;
  }

  public double getLeftLinearVelocity() {
    return DriveUtils.rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLinearVelocity() {
    return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
  }

  public double getAngularVelocity() {
    return (getRightLinearVelocity() - getLeftLinearVelocity()) / ChezyConstants.kDriveWheelTrackWidthInches;
  }

  public synchronized void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, 5);
    rightMaster.setSelectedSensorPosition(0, 0, 5);
    leftSlave1.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
  }

  public static Drivetrain getInstance() {
    return instance;
  }


  /**
   * Start up velocity mode. This sets the drive train in high gear as well.
   */
  public synchronized void setVelocitySetpoint(double leftInchesPerSecond,
      double rightInchesPerSecond) {
    configureTalonsForSpeedControl();
    driveMode = DriveMode.VELOCITY_SETPOINT;
    updateVelocitySetpoint(leftInchesPerSecond, rightInchesPerSecond);
  }

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
   * Configures talons for velocity control <p><i>(Modified for new DriveMode enum)</i></p>
   *
   * @author cyocom
   * @author team254
   */
  private void configureTalonsForSpeedControl() {
    if (driveMode == null || !Objects
        .equals(driveMode.getRequestedControlMode(), ControlMode.Velocity)) {
      // We entered a velocity control state.
      setBrakeMode(true);
    }
  }

  /**
   * Adjust Velocity setpoint (if already in velocity mode) <p><i>(Modified for new DriveMode
   * enum)</i></p>
   */
  private synchronized void updateVelocitySetpoint(double leftInchesPerSecond,
      double rightInchesPerSecond) {

    if (driveMode.getRequestedControlMode().equals(ControlMode.Velocity)) {
      periodicIO.left_demand = DriveUtils
          .inchesPerSecondToEncoderCountPer100ms(leftInchesPerSecond);
      periodicIO.right_demand = DriveUtils
          .inchesPerSecondToEncoderCountPer100ms(rightInchesPerSecond);

    } else {
      System.out.println(String.format("Hit a bad velocity control state %s %s",
          driveMode.getRequestedControlMode(), driveMode));
      periodicIO.left_demand = 0;
      periodicIO.right_demand = 0;
    }
  }

  public double getLeftVelocityInchesPerSec() {
    return DriveUtils.encoderCountToInches(periodicIO.left_velocity_ticks_per_100ms) * 10;
  }

  public double getRightVelocityInchesPerSec() {
    return DriveUtils.encoderCountToInches(periodicIO.right_velocity_ticks_per_100ms) * 10;
  }

  public double getLeftDistanceInches() {
    return DriveUtils.encoderCountToInches(periodicIO.left_position_ticks);
  }

  public double getRightDistanceInches() {
    return DriveUtils.encoderCountToInches(periodicIO.right_position_ticks);
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

  /*public synchronized boolean isDoneWithPath() {
//		System.out.println(String.format("Mode: %s\tLeft: %s\tRight: %s", driveMode, leftEncoderFollower.isFinished(), rightEncoderFollower.isFinished()));

    if (driveMode == DriveMode.PATH_FOLLOWING && leftEncoderFollower != null) {
      return leftEncoderFollower.isFinished();
    } else if (driveMode == DriveMode.CHEZY_PATH_FOLLOWING && pathFollower != null) {
      return pathFollower.isFinished();
    } else {
      System.out.println("Robot is not in path following mode");
      return true;
    }
  }

  public synchronized void setWantClimb() {
    driveMode = DriveMode.CLIMB;

  }*/

  private void updateChezyPathFollower(double timestamp) {
    if (driveMode == DriveMode.CHEZY_PATH_FOLLOWING) {
      final double now = Timer.getFPGATimestamp();

      DriveMotionPlanner.Output output = motionPlanner
          .update(now, RobotState.getInstance().getLatestFieldToVehicle().getValue());

      // DriveCommand command = new DriveCommand(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

      periodicIO.error = motionPlanner.error();
      periodicIO.path_setpoint = motionPlanner.setpoint();

      if (output.left_velocity != 0) {
        System.out.println(String.format("[%s]: %s", timestamp, output));
        //System.out.println(String.format("[%s]: %s - %s => %s", timestamp, periodicIO.path_setpoint.state().getPose(),
        //    RobotState.getInstance().getLatestFieldToVehicle().getValue(), periodicIO.error));

      }

      if (!overrideTrajectory) {
        setVelocity(new DriveCommand(Units.rads_per_sec_to_tp100ms(output.left_velocity),
                Units.rads_per_sec_to_tp100ms(output.right_velocity)),
            new DriveCommand(output.left_feedforward_voltage / 12.0,
                output.right_feedforward_voltage / 12.0));

        periodicIO.left_accel = Units.rads_per_sec_to_tp100ms(output.left_accel) / 1000.0;
        periodicIO.right_accel = Units.rads_per_sec_to_tp100ms(output.right_accel) / 1000.0;
      } else {
        setVelocity(DriveCommand.BRAKE, DriveCommand.BRAKE);
        periodicIO.left_accel = periodicIO.right_accel = 0.0;
      }
    } else {
      DriverStation.reportError("Drive is not in path following state", false);
    }
  }

/*
  public synchronized void setWantDrivePath(InterpolatingTrajectory centerTrajectory,
      InterpolatingTrajectory leftTrajectory, InterpolatingTrajectory rightTrajectory,
      boolean inverted) {
    if (!Objects.equals(trajectory, centerTrajectory) || driveMode != DriveMode.PATH_FOLLOWING) {
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
    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
  }

  public synchronized void setWantDrivePath(Path path, boolean reversed, double acceleration) {
    /*if (!Objects.equals(currentPath, path) || driveMode != DriveMode.CHEZY_PATH_FOLLOWING) {
      configureTalonsForSpeedControl();
      RobotState.getInstance().resetDistanceDriven();
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
    }*/
  }

  public synchronized void startRotation(Rotation2d heading) {
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
  }

  @Override
  public void test() {
  }

  public synchronized void startLogging() {
    if (CSVWriter == null) {
      CSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
    }
  }

  public synchronized void stopLogging() {
    if (CSVWriter != null) {
      CSVWriter.flush();
      CSVWriter = null;
    }
  }

  @Override
  public synchronized void readPeriodicInputs() {
    double prevLeftTicks = periodicIO.left_position_ticks;
    double prevRightTicks = periodicIO.right_position_ticks;
    periodicIO.left_position_ticks = leftMaster.getSelectedSensorPosition(0);
    periodicIO.right_position_ticks = rightMaster.getSelectedSensorPosition(0);
    periodicIO.left_velocity_ticks_per_100ms = leftMaster.getSelectedSensorVelocity(0);
    periodicIO.right_velocity_ticks_per_100ms = rightMaster.getSelectedSensorVelocity(0);
    periodicIO.gyro_heading = navX.getYaw();

    double deltaLeftTicks = ((periodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
    if (deltaLeftTicks > 0.0) {
      periodicIO.left_distance += deltaLeftTicks * RobotConstants.DRIVE_WHEEL_DIAMETER;
    } else {
      periodicIO.left_distance += deltaLeftTicks * RobotConstants.DRIVE_WHEEL_DIAMETER;
    }

    double deltaRightTicks =
        ((periodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
    if (deltaRightTicks > 0.0) {
      periodicIO.right_distance += deltaRightTicks * RobotConstants.DRIVE_WHEEL_DIAMETER;
    } else {
      periodicIO.right_distance += deltaRightTicks * RobotConstants.DRIVE_WHEEL_DIAMETER;
    }

    if (CSVWriter != null) {
      CSVWriter.add(periodicIO);
    }

    // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    if (driveMode == DriveMode.OPEN_LOOP) {
      leftMaster
          .set(ControlMode.PercentOutput, periodicIO.left_demand, DemandType.ArbitraryFeedForward,
              0.0);
      rightMaster
          .set(ControlMode.PercentOutput, periodicIO.right_demand, DemandType.ArbitraryFeedForward,
              0.0);
    } else if (driveMode == DriveMode.CHEZY_PATH_FOLLOWING) {
      leftMaster.set(ControlMode.Velocity, periodicIO.left_demand, DemandType.ArbitraryFeedForward,
          periodicIO.left_feedforward + ChezyConstants.PID_D * periodicIO.left_accel / 1023.0);
      rightMaster
          .set(ControlMode.Velocity, periodicIO.right_demand, DemandType.ArbitraryFeedForward,
              periodicIO.right_feedforward
                  + ChezyConstants.PID_D * periodicIO.right_accel / 1023.0);
    } else if (driveMode == DriveMode.TURN_TO_HEADING) {
      leftMaster.set(
          ControlMode.MotionMagic,
          DriveUtils.inchesToEncoderCount(periodicIO.left_turn));
      rightMaster.set(
          ControlMode.MotionMagic,
          DriveUtils.inchesToEncoderCount(periodicIO.right_turn));
    } else if (driveMode == DriveMode.VELOCITY_SETPOINT) {
      leftMaster.set(
          ControlMode.Velocity,
          periodicIO.left_demand);

      rightMaster.set(
          ControlMode.Velocity,
          periodicIO.right_demand);

    } else {
      System.out.println(String.format("Hit a bad control state %s %s",
          driveMode.getRequestedControlMode(), driveMode));
    }
    //if (!isDoneWithTrajectory())

      //System.out.println(String.format("[%s]:\t%s %s %s %s", Timer.getFPGATimestamp(), periodicIO.left_demand, periodicIO.right_demand, periodicIO.left_feedforward, periodicIO.right_feedforward));
  }

  public static class PeriodicIO {

    // INPUTS
    public int left_position_ticks;
    public int right_position_ticks;
    public double left_distance;
    public double right_distance;
    public int left_velocity_ticks_per_100ms;
    public int right_velocity_ticks_per_100ms;
    public Rotation2d gyro_heading = Rotation2d.identity();
    public Pose2d error = Pose2d.identity();

    // OUTPUTS
    public double left_turn;
    public double right_turn;
    public double left_demand;
    public double right_demand;
    public double left_accel;
    public double right_accel;
    public double left_feedforward;
    public double right_feedforward;
    public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
        Pose2dWithCurvature.identity());
  }
}
