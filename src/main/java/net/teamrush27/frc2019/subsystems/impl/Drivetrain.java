package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;
import net.teamrush27.frc2019.util.math.Rotation2d;

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

  private DriveMode driveMode = DriveMode.OPEN_LOOP;

  private PeriodicIO periodicIO;

  private double timeSinceModeSwitch;
  private boolean modeChanged;

  private TalonSRX leftMaster;
  private TalonSRX leftSlave1;
  private TalonSRX leftSlave2;
  private TalonSRX rightMaster;
  private TalonSRX rightSlave1;
  private TalonSRX rightSlave2;

  private AHRS navx;

  private final Loop loop = new Loop() {
    private DriveMode lastMode;

    @Override
    public void onStart(double timestamp) {
      synchronized (Drivetrain.this) {

      }
    }

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
          driveMode = handleOpenLoop();
          break;
        case TURN_TO_HEADING:
          driveMode = handleTurnToHeading();
          break;
        case DRIVE_DISTANCE:
          driveMode = handleDriveDistance();
          break;
        case DRIVE_VELOCITY:
          driveMode = handleDriveVelocity();
          break;
        default:
          LOG.warn("Unexpected drive mode: " + driveMode);
          break;
        }
      }
    }

    @Override
    public void onStop(double timestamp) {
      stop();
    }

    @Override
    public String id() {
      return TAG;
    }

  };

  public void setOpenLoop(double leftInput, double rightInput) {
    if (!driveMode.equals(DriveMode.OPEN_LOOP)) {
      periodicIO.clearInputs();
      driveMode = DriveMode.OPEN_LOOP;
    }

    periodicIO.leftInput = leftInput;
    periodicIO.rightInput = rightInput;
  }

  private DriveMode handleOpenLoop() {
    periodicIO.leftOutput = periodicIO.leftInput;
    periodicIO.rightOutput = periodicIO.rightInput;

    return DriveMode.OPEN_LOOP;
  }

  public void setTurnToHeading(double heading) {
    // TODO figure it out
    if (!driveMode.equals(DriveMode.TURN_TO_HEADING)) {
      // STUFF
      leftMaster.selectProfileSlot(0, 0);
      rightMaster.selectProfileSlot(0, 0);

      periodicIO.clearInputs();
      driveMode = DriveMode.TURN_TO_HEADING;
    }

    periodicIO.goalHeading = heading;
  }

  private DriveMode handleTurnToHeading() {
    periodicIO.leftOutput = periodicIO.leftDistance;
    periodicIO.rightOutput = periodicIO.rightDistance;

    return DriveMode.TURN_TO_HEADING;
  }

  public void setDriveDistance(double distance) {
    if (!driveMode.equals(DriveMode.DRIVE_DISTANCE)) {

      leftMaster.selectProfileSlot(0, 0);
      rightMaster.selectProfileSlot(0, 0);

      periodicIO.clearInputs();
      driveMode = DriveMode.DRIVE_DISTANCE;
    }
    periodicIO.goalLeftDistance = periodicIO.leftDistance + distance * RobotConstants.DT_TICKS_PER_INCH;
    periodicIO.goalRightDistance = periodicIO.rightDistance + distance * RobotConstants.DT_TICKS_PER_INCH;

  }

  private DriveMode handleDriveDistance() {
    periodicIO.leftOutput = periodicIO.goalLeftDistance;
    periodicIO.rightOutput = periodicIO.goalRightDistance;

    return DriveMode.DRIVE_DISTANCE;
  }

  public void setDriveVelocity(double velocity) {
    if (!driveMode.equals(DriveMode.DRIVE_VELOCITY)) {
      leftMaster.selectProfileSlot(1, 0);
      rightMaster.selectProfileSlot(1, 0);

      periodicIO.clearInputs();
      driveMode = DriveMode.DRIVE_VELOCITY;
    }

    periodicIO.goalVelocity = velocity;
  }

  private DriveMode handleDriveVelocity() {
    periodicIO.leftOutput = periodicIO.goalVelocity;
    periodicIO.rightOutput = periodicIO.goalVelocity;

    return DriveMode.DRIVE_VELOCITY;
  }

  public Drivetrain() {
    periodicIO = new PeriodicIO();

    leftMaster = new TalonSRX(11);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftMaster.setSensorPhase(false);
    leftMaster.setInverted(true);

    leftSlave1 = new TalonSRX(13);
    leftSlave1.follow(leftMaster);
    leftSlave1.setInverted(true);

    leftSlave2 = new TalonSRX(15);
    leftSlave2.follow(leftMaster);
    leftSlave2.setInverted(true);

    rightMaster = new TalonSRX(12);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMaster.setSensorPhase(true);
    rightMaster.setInverted(false);

    rightSlave1 = new TalonSRX(14);
    rightSlave1.follow(rightMaster);
    rightSlave1.setInverted(false);

    rightSlave2 = new TalonSRX(16);
    rightSlave2.follow(rightMaster);
    rightSlave2.setInverted(false);


    /* Drivetrain PID Constants */
    // Left Slot 0 - Motion Magic
    leftMaster.config_kP(0, 1.0);
    leftMaster.config_kI(0, 0.0);
    leftMaster.config_kD(0, 0.0);
    leftMaster.config_kF(0, 0.0);

    // Left slot 1 - Velocity
    leftMaster.config_kP(1, 1.0 / RobotConstants.DRIVE_CRUISE_VELOCITY);
    leftMaster.config_kI(1, 0.0);
    leftMaster.config_kD(1, 10.0 / RobotConstants.DRIVE_CRUISE_VELOCITY);
    leftMaster.config_kF(1, 0.0);

    // Motion Magic Cruise Velocity and Acceleration
    leftMaster.configMotionCruiseVelocity(RobotConstants.DRIVE_CRUISE_VELOCITY);
    leftMaster.configMotionAcceleration(RobotConstants.DRIVE_ACCELERATION);

    //
    rightMaster.config_kP(0, 1.0);
    rightMaster.config_kI(0, 0.0);
    rightMaster.config_kD(0, 0.0);
    rightMaster.config_kF(0, 0.0);

    rightMaster.config_kP(1, 1.0 / RobotConstants.DRIVE_CRUISE_VELOCITY);
    rightMaster.config_kI(1, 0.0);
    rightMaster.config_kD(1, 10.0 / RobotConstants.DRIVE_CRUISE_VELOCITY);
    rightMaster.config_kF(1, 0.0);

    rightMaster.configMotionCruiseVelocity(RobotConstants.DRIVE_CRUISE_VELOCITY);
    rightMaster.configMotionAcceleration(RobotConstants.DRIVE_ACCELERATION);

    navx = new AHRS(Port.kMXP);

    setBrakeMode(false);
  }

  private void setBrakeMode(boolean brakeMode) {
    NeutralMode mode = brakeMode ? NeutralMode.Brake : NeutralMode.Coast;

    leftMaster.setNeutralMode(mode);
    leftSlave1.setNeutralMode(mode);
    leftSlave2.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    rightSlave1.setNeutralMode(mode);
    rightSlave2.setNeutralMode(mode);
  }

  @Override
  public void outputToSmartDashboard() {
    SmartDashboard.putString("driveMode", driveMode.name());

    SmartDashboard.putNumber("leftInput", periodicIO.leftInput);
    SmartDashboard.putNumber("rightInput", periodicIO.rightInput);
    SmartDashboard.putNumber("goalHeading", periodicIO.goalHeading);
    SmartDashboard.putNumber("goalLeftDistance", periodicIO.goalLeftDistance);
    SmartDashboard.putNumber("goalRightDistance", periodicIO.goalRightDistance);
    SmartDashboard.putNumber("goalVelocity", periodicIO.goalVelocity);
    SmartDashboard.putNumber("leftDistance", periodicIO.leftDistance);
    SmartDashboard.putNumber("rightDistance", periodicIO.rightDistance);
    SmartDashboard.putNumber("leftVelocity", periodicIO.leftVelocity);
    SmartDashboard.putNumber("rightVelocity", periodicIO.rightVelocity);
    SmartDashboard.putNumber("heading", periodicIO.heading);
    SmartDashboard.putNumber("leftOutput", periodicIO.leftOutput);
    SmartDashboard.putNumber("rightOutput", periodicIO.rightOutput);

  }

  @Override
  public void stop() {
    leftMaster.set(ControlMode.Disabled, 0);
    rightMaster.set(ControlMode.Disabled, 0);
  }

  @Override
  public void zeroSensors() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

    navx.zeroYaw();
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void test() {
  }

  @Override
  public synchronized void readPeriodicInputs() {
    periodicIO.timestamp = Timer.getFPGATimestamp();

    periodicIO.leftDistance = leftMaster.getSelectedSensorPosition();
    periodicIO.rightDistance = rightMaster.getSelectedSensorPosition();

    periodicIO.leftVelocity = leftMaster.getSelectedSensorVelocity();
    periodicIO.rightVelocity = rightMaster.getSelectedSensorVelocity();

    periodicIO.heading = navx.getAngle();
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    leftMaster.set(driveMode.getControlMode(), periodicIO.leftOutput);
    rightMaster.set(driveMode.getControlMode(), periodicIO.rightOutput);
  }

  @Override
  public String id() {
    return TAG;
  }

  public static class PeriodicIO {
    public double timestamp;

    // Control Inputs
    public double leftInput;
    public double rightInput;
    public double goalHeading;
    public double goalLeftDistance;
    public double goalRightDistance;
    public double goalVelocity;

    // Periodic IO
    public double leftDistance;
    public double rightDistance;
    public double leftVelocity;
    public double rightVelocity;
    public double heading;

    // Outputs
    public double leftOutput;
    public double rightOutput;

    public void clearInputs() {
      leftInput = 0;
      rightInput = 0;
      goalHeading = 0;
      goalLeftDistance = 0;
      goalRightDistance = 0;
      goalVelocity = 0;
    }
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(periodicIO.heading);
  }

  public void setHeading(Rotation2d heading) {
    navx.setAngleAdjustment(heading.getDegrees());
  }

  public double getLeftEncoderDistance() {
    return periodicIO.leftDistance / RobotConstants.DT_TICKS_PER_INCH;
  }

  public double getRightEncoderDistance() {
    return periodicIO.rightDistance / RobotConstants.DT_TICKS_PER_INCH;
  }

  public double getLeftLinearVelocity() {
    return periodicIO.leftVelocity / RobotConstants.DT_TICKS_PER_INCH;
  }

  public double getRightLinearVelocity() {
    return periodicIO.rightVelocity / RobotConstants.DT_TICKS_PER_INCH;
  }
}
