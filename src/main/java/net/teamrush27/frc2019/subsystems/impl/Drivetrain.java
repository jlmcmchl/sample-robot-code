package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Objects;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.constants.ChezyConstants;
import net.teamrush27.frc2019.constants.DriveConstants;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.managers.SuperstructureManager;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;
import net.teamrush27.frc2019.subsystems.impl.util.DriveUtils;
import net.teamrush27.frc2019.util.ReflectingCSVWriter;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Pose2dWithCurvature;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Twist2d;
import net.teamrush27.frc2019.util.motion.DriveMotionPlanner;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryIterator;
import net.teamrush27.frc2019.util.trajectory.timing.TimedState;
import net.teamrush27.frc2019.wrappers.NavX;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Drivetrain Subsystem (Adapted from 254 Drive.java)
 *
 * @author team254
 * @author cyocom
 */

public class Drivetrain extends Subsystem {

  private static final Logger LOG = LogManager.getLogger(Drivetrain.class);
  private static final String TAG = "DRIVETRAIN";

  private static Drivetrain INSTANCE = null;

  public static Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }

  private static SuperstructureManager superman = SuperstructureManager.getInstance();

  private static final double DRIVE_ENCODER_PPR = 4096.0;

  private final TalonSRX leftMaster;
  private final TalonSRX leftSlave1;
  private final TalonSRX rightMaster;
  private final TalonSRX rightSlave1;

  // new
  private final TalonSRX leftSlave2;
  private final TalonSRX rightSlave2;
  private double timeSinceModeSwitch;

  private final Servo leftShifter;
  private final Servo rightShifter;

  private final int DRIVE_CONTROL_SLOT = 0;
  private final int VELOCITY_CONTROL_SLOT = 1;
  private final int TURNING_CONTROL_SLOT = 2;

  private final AHRS navX;

  private final Limelights limelights;

  private DriveMode driveMode = DriveMode.OPEN_LOOP;
  private Boolean inHighGear = false;
  private RobotState robotState = RobotState.getInstance();
  private Trajectory trajectory = null;
  private DriveMotionPlanner motionPlanner;
  private boolean overrideTrajectory = false;

  private Rotation2d gyroOffset = Rotation2d.identity();

  private PeriodicIO periodicIO;

  private boolean brakeMode = true;

  private boolean modeChanged = false;
  private boolean isTrajectoryInverted = false;

  private Rotation2d targetHeading = null;
  private boolean isOnTarget = false;

  private ReflectingCSVWriter<PeriodicIO> CSVWriter = null;

  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

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
      }
    }

    /**
     * @author team254
     */
    @Override
    public void onLoop(double timestamp) {
      synchronized (Drivetrain.this) {
        if (driveMode != lastMode) {
          LOG.info("DriveMode changed from {} to {}", lastMode, driveMode);
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
          case TURN_TO_HEADING:
            updateTurnToHeading(timestamp);
            break;
          case CHEZY_PATH_FOLLOWING:
            updateChezyPathFollower(timestamp);
            break;
          case LIMELIGHT_STEERING:
            updateLimelightSteering(timestamp);
            break;
          default:
            LOG.warn("Unexpected drive mode: " + driveMode);
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

    @Override
    public String id() {
      return TAG;
    }

  };

  /**
   * @author team254
   */
  public Drivetrain() {
    periodicIO = new PeriodicIO();

    leftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
    leftMaster.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configVelocityMeasurementWindow(32, RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.setSensorPhase(true);

    leftMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster.enableVoltageCompensation(true);
    leftMaster.configVoltageCompSaturation(12.0, RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster
        .configMotionCruiseVelocity(DriveUtils.inchesPerSecondToEncoderCountPer100ms(10 * 12),
            RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster
        .configMotionAcceleration(DriveUtils.inchesPerSecondToEncoderCountPer100ms(15 * 12),
            RobotConstants.TALON_CONFIG_TIMEOUT);

    leftSlave1 = new TalonSRX(RobotMap.DRIVE_LEFT_SLAVE_1_CAN_ID);
    leftSlave1.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.setSensorPhase(false);
    leftSlave1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.follow(leftMaster);

    leftSlave2 = new TalonSRX(RobotMap.DRIVE_LEFT_SLAVE_2_CAN_ID);
    leftSlave2.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.follow(leftMaster);

    rightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
    rightMaster.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configVelocityMeasurementWindow(32, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.setSensorPhase(true);

    rightMaster.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster
        .configMotionCruiseVelocity(DriveUtils.inchesPerSecondToEncoderCountPer100ms(10 * 12),
            RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster
        .configMotionAcceleration(DriveUtils.inchesPerSecondToEncoderCountPer100ms(15 * 12),
            RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster.enableVoltageCompensation(true);
    rightMaster.configVoltageCompSaturation(12.0, RobotConstants.TALON_CONFIG_TIMEOUT);

    rightSlave1 = new TalonSRX(RobotMap.DRIVE_RIGHT_SLAVE_1_CAN_ID);
    rightSlave1.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.follow(rightMaster);

    rightSlave2 = new TalonSRX(RobotMap.DRIVE_RIGHT_SLAVE_2_CAN_ID);
    rightSlave2.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.configContinuousCurrentLimit(DriveConstants.MAX_CONTINUOUS_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.configPeakCurrentDuration(DriveConstants.PEAK_CURRENT_DURATION,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.configPeakCurrentLimit(DriveConstants.MAX_PEAK_CURRENT,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.follow(rightMaster);

    leftMaster.setInverted(false);
    leftSlave1.setInverted(false);
    leftSlave2.setInverted(false);
    rightMaster.setInverted(true);
    rightSlave1.setInverted(true);
    rightSlave2.setInverted(true);

    leftShifter = new Servo(RobotMap.LEFT_DRIVE_SHIFT_SERVO_ID);
    rightShifter = new Servo(RobotMap.RIGHT_DRIVE_SHIFT_SERVO_ID);

    setCurrentLimiting(false);
    reloadGains();

    periodicIO = new PeriodicIO();
    navX = new NavX(SPI.Port.kMXP);

    setBrakeMode(false);

    shift(true);

    motionPlanner = new DriveMotionPlanner();

    limelights = Limelights.getInstance();
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

    final Rotation2d field_to_robot = robotState.getLatestFieldToVehicle().getValue()
        .getRotation();

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
      LOG.warn("Hit a bad position control state");
      setOpenLoop(DriveCommand.defaultCommand());
    }
  }


  /**
   * @author team254
   */
  public void setBrakeMode(boolean requestedBrakeMode) {
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

    leftMaster.configClosedloopRamp(DriveConstants.PID_RAMP_RATE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster.configClosedloopRamp(DriveConstants.PID_RAMP_RATE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster.config_kP(DRIVE_CONTROL_SLOT, DriveConstants.PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kI(DRIVE_CONTROL_SLOT, DriveConstants.PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kD(DRIVE_CONTROL_SLOT, DriveConstants.PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kF(DRIVE_CONTROL_SLOT, DriveConstants.PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_IntegralZone(DRIVE_CONTROL_SLOT, DriveConstants.PID_I_ZONE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster.config_kP(DRIVE_CONTROL_SLOT, DriveConstants.PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kI(DRIVE_CONTROL_SLOT, DriveConstants.PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kD(DRIVE_CONTROL_SLOT, DriveConstants.PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kF(DRIVE_CONTROL_SLOT, DriveConstants.PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_IntegralZone(DRIVE_CONTROL_SLOT, DriveConstants.PID_I_ZONE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster.config_kP(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kI(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kD(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kF(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_IntegralZone(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I_ZONE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster.config_kP(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kI(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kD(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kF(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_IntegralZone(VELOCITY_CONTROL_SLOT, ChezyConstants.PID_I_ZONE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    leftMaster.config_kP(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kI(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kD(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_kF(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.config_IntegralZone(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I_ZONE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    rightMaster.config_kP(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_P,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kI(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kD(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_D,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_kF(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_F,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.config_IntegralZone(TURNING_CONTROL_SLOT, ChezyConstants.ROTATE_PID_I_ZONE,
        RobotConstants.TALON_CONFIG_TIMEOUT);

    LOG.info("reloading gains took {} seconds", (Timer.getFPGATimestamp() - startTime));

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
  public void outputToSmartDashboard(SmartDashboardCollection collection) {

    LOG.trace("left {} - right {}", leftMaster.getSelectedSensorPosition(),
        rightMaster.getSelectedSensorPosition());

    //collection.setArmAbsoluteRotation(leftSlave1.getSelectedSensorPosition());
    //collection.setDrivetrainLeftPosition(leftMaster.getSelectedSensorPosition());
    //collection.setDrivetrainRightPosition(rightMaster.getSelectedSensorPosition());

    SmartDashboard.putNumber("arm.absolute_rotation", leftSlave1.getSelectedSensorPosition());
    SmartDashboard.putNumber("drivetrain.left.position", leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("drivetrain.right.position", rightMaster.getSelectedSensorPosition());


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

  public void shift() {
    synchronized (inHighGear) {
      shift(!inHighGear);
    }
  }

  public void shift(boolean wantsHighGear) {
    synchronized (inHighGear) {
      if (wantsHighGear != inHighGear) {
        if (wantsHighGear) {
          LOG.info("Shifted to High Gear");
          leftShifter.set(1);
          rightShifter.set(0);
          inHighGear = true;
        } else {
          LOG.info("Shifted to Low Gear");
          leftShifter.set(0);
          rightShifter.set(1);
          inHighGear = false;
        }
      }
    }
  }

  /**
   * Configures talons for velocity control
   */
  public synchronized void setVelocity(DriveCommand signal, DriveCommand feedforward) {
    if (driveMode != DriveMode.CHEZY_PATH_FOLLOWING) {
      // We entered a velocity control state.
      setBrakeMode(true);
      reloadChezyGains();
      leftMaster.configNeutralDeadband(0.0, 0);
      rightMaster.configNeutralDeadband(0.0, 0);

      reloadChezyGains();

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
    LOG.info("SET HEADING: {}", heading.getDegrees());

    navX.reset();

    gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(navX.getFusedHeading()).inverse());
    LOG.info("Gyro offset: {}", gyroOffset.getDegrees());

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
    if (!DriveMode.OPEN_LOOP.equals(driveMode) && !DriveMode.LIMELIGHT_STEERING.equals(driveMode)) {
      driveMode = DriveMode.OPEN_LOOP;
      setBrakeMode(command.getBrakeMode());
      setCurrentLimiting(true);

      leftMaster.configNeutralDeadband(0.04, 0);
      rightMaster.configNeutralDeadband(0.04, 0);
    }

    periodicIO.left_demand = command.getLeftDriveInput();
    periodicIO.right_demand = command.getRightDriveInput();
    periodicIO.left_feedforward = 0.0;
    periodicIO.right_feedforward = 0.0;
  }

  @Override
  public void zeroSensors() {
    resetEncoders();
    navX.zeroYaw();
  }

  public double getLeftEncoderRotations() {
    return periodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
  }

  public double getRightEncoderRotations() {
    return periodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
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
    return DriveUtils
        .rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLeftVelocityNativeUnits() {
    return periodicIO.left_velocity_ticks_per_100ms;
  }

  public double getLeftLinearVelocity() {
    return DriveUtils
        .rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLinearVelocity() {
    return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
  }

  public double getAngularVelocity() {
    return (getRightLinearVelocity() - getLeftLinearVelocity())
        / ChezyConstants.kDriveWheelTrackWidthInches;
  }

  public synchronized void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.setSelectedSensorPosition(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);

    Arm.getInstance().setAbsolutePosition(leftSlave1.getSelectedSensorPosition());
  }

  public synchronized void fixRotationEncoder() {
    int rotationTicks = leftSlave1.getSelectedSensorPosition();
    if (rotationTicks < -180) {
      rotationTicks += 360;
    } else if (rotationTicks > 180) {
      rotationTicks -= 360;
    }

    leftSlave1.setSelectedSensorPosition(rotationTicks, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
  }


  /**
   * Start up velocity mode. This sets the drive train in high gear as well.
   */
  public synchronized void setVelocitySetpoint(DriveCommand command) {
    if (driveMode != DriveMode.VELOCITY_SETPOINT) {
      configureTalonsForSpeedControl();
      driveMode = DriveMode.VELOCITY_SETPOINT;
    }
    setBrakeMode(command.getBrakeMode());
    updateVelocitySetpoint(command);
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
      leftMaster.selectProfileSlot(TURNING_CONTROL_SLOT, 0);
      rightMaster.selectProfileSlot(TURNING_CONTROL_SLOT, 0);
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
      leftMaster.selectProfileSlot(TURNING_CONTROL_SLOT, 0);
      rightMaster.selectProfileSlot(TURNING_CONTROL_SLOT, 0);
      leftMaster.configNeutralDeadband(0.0, 0);
      rightMaster.configNeutralDeadband(0.0, 0);
    }
  }

  /**
   * Adjust Velocity setpoint (if already in velocity mode) <p><i>(Modified for new DriveMode
   * enum)</i></p>
   */
  private synchronized void updateVelocitySetpoint(DriveCommand command) {
    if (driveMode.getRequestedControlMode().equals(ControlMode.Velocity)) {
      periodicIO.left_demand = DriveUtils
          .inchesPerSecondToEncoderCountPer100ms(command.getLeftDriveInput());
      periodicIO.right_demand = DriveUtils
          .inchesPerSecondToEncoderCountPer100ms(command.getRightDriveInput());

    } else {
      LOG.warn("Hit a bad velocity control state {} {}",
          driveMode.getRequestedControlMode(), driveMode);
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
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  private void updateChezyPathFollower(double timestamp) {
    if (driveMode == DriveMode.CHEZY_PATH_FOLLOWING) {
      double now = Timer.getFPGATimestamp();

      //periodicIO.field_to_vehicle = RobotState.getInstance().getPredictedFieldToVehicle(now);
      periodicIO.field_to_vehicle = new Pose2d(
          RobotState.getInstance().getLatestFieldToVehicle().getValue());

      DriveMotionPlanner.Output output = motionPlanner
          .update(now, periodicIO.field_to_vehicle);

      periodicIO.error = motionPlanner.error();
      periodicIO.path_setpoint = motionPlanner.setpoint();

      if (!overrideTrajectory) {
        setVelocity(new DriveCommand(
                DriveUtils.radiansPerSecondToEncoderCountPer100ms(output.left_velocity),
                DriveUtils.radiansPerSecondToEncoderCountPer100ms(output.right_velocity)),
            new DriveCommand(output.left_feedforward_voltage / 12.0,
                output.right_feedforward_voltage / 12.0));

        periodicIO.left_accel =
            DriveUtils.radiansPerSecondToEncoderCountPer100ms(output.left_accel) / 1000.0;
        periodicIO.right_accel =
            DriveUtils.radiansPerSecondToEncoderCountPer100ms(output.right_accel) / 1000.0;
      } else {
        setVelocity(DriveCommand.BRAKE, DriveCommand.BRAKE);
        periodicIO.left_accel = periodicIO.right_accel = 0.0;
      }
    } else {
      DriverStation.reportError("Drive is not in path following state", false);
    }
  }

  public void setLimelightSteering(Limelights.SystemState limelightState) {
    switch (limelightState) {
      case REAR_TRACKING:
      case FRONT_TRACKING:
      case BOTH_TRACKING:
        driveMode = DriveMode.LIMELIGHT_STEERING;
        break;
      case DRIVE:
      default:
        driveMode = DriveMode.OPEN_LOOP;
    }
  }

  private void updateLimelightSteering(double timestamp) {
    periodicIO.left_turn = 1;
    periodicIO.right_turn = 1;

    LOG.info(String.format("superman: %s\tdemand: %s", superman.overBack(), periodicIO.left_demand));

    if (superman.overBack() && periodicIO.left_demand <= 0.01)  {
      periodicIO.left_demand = Math.min(periodicIO.left_demand, -.2);
    } else if (!superman.overBack() && periodicIO.left_demand >= -0.01) {
      periodicIO.left_demand = Math.max(periodicIO.left_demand, .2);
    } else {
      return;
    }


    // INVERTS REAR ANGLE OFFSET TO ACCOUNT FOR THROTTLE DIRECTION

    double angle_offset = limelights.getOffset(!superman.overBack());
    double target_area = limelights.getTargetArea(!superman.overBack());

    // more aggressive further away at lower angles
    if (target_area < 3d && Math.abs(angle_offset) < 4) {
      if (angle_offset > 0) {
        periodicIO.right_turn = 1 - angle_offset / 5;
      } else {
        periodicIO.left_turn = 1 + angle_offset / 5;
      }
    } else {
      if (angle_offset > 0) {
        periodicIO.right_turn = 1 - angle_offset / 6;
      } else {
        periodicIO.left_turn = 1 + angle_offset / 6;
      }
    }
  }

  public void defaultState() {
    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
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
      LOG.warn("Robot is not in turn to heading mode");
      return false;
    }
  }

  public void fixArm() {
    Arm.getInstance().setAbsolutePosition(leftSlave1.getSelectedSensorPosition());
  }

  @Override
  public void test() {
  }

  public synchronized void startLogging() {
    if (CSVWriter == null) {
      CSVWriter = new ReflectingCSVWriter<>("/media/sda/logs/DRIVE-LOGS.csv", PeriodicIO.class);
    }
  }

  public synchronized void stopLogging() {
    if (CSVWriter != null) {
      CSVWriter.flush();
      LOG.info("Drivetrain DONE logging");
      CSVWriter = null;
    }
  }

  public void resetArmPosition(int armPositionTicks) {
    leftSlave1.setSelectedSensorPosition(armPositionTicks, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
  }

  @Override
  public synchronized void readPeriodicInputs() {
    double prevLeftTicks = periodicIO.left_position_ticks;
    double prevRightTicks = periodicIO.right_position_ticks;
    double prevLeftVelocity = periodicIO.left_velocity_ticks_per_100ms;
    double prevRightVelocity = periodicIO.right_velocity_ticks_per_100ms;
    double prevTimestamp = periodicIO.timestamp;

    periodicIO.timestamp = Timer.getFPGATimestamp();
    periodicIO.left_position_ticks = leftMaster.getSelectedSensorPosition(0);
    periodicIO.right_position_ticks = rightMaster.getSelectedSensorPosition(0);
    periodicIO.left_velocity_ticks_per_100ms = leftMaster.getSelectedSensorVelocity(0);
    periodicIO.right_velocity_ticks_per_100ms = rightMaster.getSelectedSensorVelocity(0);

    periodicIO.left_accel_ticks_per_100ms_per_1000ms =
        (periodicIO.left_velocity_ticks_per_100ms - prevLeftVelocity)
            / (periodicIO.timestamp - prevTimestamp);
    periodicIO.right_accel_ticks_per_100ms_per_1000ms =
        (periodicIO.right_velocity_ticks_per_100ms - prevRightVelocity)
            / (periodicIO.timestamp - prevTimestamp);

    periodicIO.gyro_heading = Rotation2d
        .fromDegrees(navX.getFusedHeading()).rotateBy(gyroOffset);
    periodicIO.can_read_delta = Timer.getFPGATimestamp() - periodicIO.timestamp;

    double deltaLeftTicks =
        ((periodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
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
      periodicIO = new PeriodicIO(periodicIO);
    }
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    /*if (!isDoneWithTrajectory()) {
      System.out.println(String
          .format("[%s]:\t%s %s\t%s %s\t%s %s", Timer.getFPGATimestamp(), periodicIO.left_demand,
              periodicIO.right_demand, periodicIO.left_accel, periodicIO.right_accel,
              periodicIO.left_feedforward, periodicIO.right_feedforward));
    }*/

    if (driveMode == DriveMode.OPEN_LOOP) {
      periodicIO.field_to_vehicle = RobotState.getInstance().getLatestFieldToVehicle()
          .getValue();

      leftMaster
          .set(ControlMode.PercentOutput, periodicIO.left_demand,
              DemandType.ArbitraryFeedForward,
              0.0);
      rightMaster
          .set(ControlMode.PercentOutput, periodicIO.right_demand,
              DemandType.ArbitraryFeedForward,
              0.0);
    } else if (driveMode == DriveMode.CHEZY_PATH_FOLLOWING) {
      leftMaster.set(ControlMode.Velocity, periodicIO.left_demand,
          DemandType.ArbitraryFeedForward,
          periodicIO.left_feedforward
              + ChezyConstants.PID_D * periodicIO.left_accel / 1023.0);
      rightMaster.set(ControlMode.Velocity, periodicIO.right_demand,
          DemandType.ArbitraryFeedForward,
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
    } else if (driveMode == DriveMode.LIMELIGHT_STEERING) {
      leftMaster.set(
          ControlMode.PercentOutput,
          periodicIO.left_demand * periodicIO.left_turn);

      rightMaster.set(
          ControlMode.PercentOutput,
          periodicIO.left_demand * periodicIO.right_turn);
    } else {
      LOG.warn("Hit a bad control state {} {}",
          driveMode.getRequestedControlMode(), driveMode);
    }
  }

  @Override
  public String id() {
    return TAG;
  }

  public static class PeriodicIO {

    public double timestamp;
    public double can_read_delta;

    // INPUTS
    public int left_position_ticks;
    public int right_position_ticks;
    public double left_distance;
    public double right_distance;
    public int left_velocity_ticks_per_100ms;
    public int right_velocity_ticks_per_100ms;
    public double left_accel_ticks_per_100ms_per_1000ms;
    public double right_accel_ticks_per_100ms_per_1000ms;

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
    public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(
        Pose2dWithCurvature.identity());
    public Pose2d field_to_vehicle = Pose2d.identity();

    public PeriodicIO() {

    }

    public PeriodicIO(PeriodicIO other) {
      this.timestamp = other.timestamp;
      this.can_read_delta = 0;

      this.left_position_ticks = other.left_position_ticks;
      this.right_position_ticks = other.right_position_ticks;
      this.left_distance = other.left_distance;
      this.right_distance = other.right_distance;
      this.left_velocity_ticks_per_100ms = other.left_velocity_ticks_per_100ms;
      this.right_velocity_ticks_per_100ms = other.right_velocity_ticks_per_100ms;
      this.left_accel_ticks_per_100ms_per_1000ms = other.left_accel_ticks_per_100ms_per_1000ms;
      this.right_accel_ticks_per_100ms_per_1000ms = other.left_accel_ticks_per_100ms_per_1000ms;
      this.gyro_heading = new Rotation2d(other.gyro_heading);
      this.error = new Pose2d(other.error);

      this.left_turn = other.left_turn;
      this.right_turn = other.right_turn;
      this.left_demand = other.left_demand;
      this.right_demand = other.right_demand;
      this.left_accel = other.left_accel;
      this.right_accel = other.right_accel;
      this.left_feedforward = other.left_feedforward;
      this.right_feedforward = other.right_feedforward;
      this.path_setpoint = new TimedState<>(
          other.path_setpoint.state(),
          other.path_setpoint.t(),
          other.path_setpoint.velocity(),
          other.path_setpoint.acceleration());
      this.field_to_vehicle = new Pose2d(other.field_to_vehicle);
    }
  }
}
