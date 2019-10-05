package net.teamrush27.frc2019.subsystems.impl;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

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
    periodicIO.leftInput = leftInput;
    periodicIO.rightInput = rightInput;
  }

  private DriveMode handleOpenLoop() {
    periodicIO.leftOutput = periodicIO.leftInput;
    periodicIO.rightOutput = periodicIO.rightInput;

    return DriveMode.OPEN_LOOP;
  }

  public Drivetrain() {
    periodicIO = new PeriodicIO();

    leftMaster = new TalonSRX(11);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftMaster.setInverted(true);

    leftSlave1 = new TalonSRX(13);
    leftSlave1.follow(leftMaster);
    leftSlave1.setInverted(true);

    leftSlave2 = new TalonSRX(15);
    leftSlave2.follow(leftMaster);
    leftSlave2.setInverted(true);

    rightMaster = new TalonSRX(12);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMaster.setInverted(false);

    rightSlave1 = new TalonSRX(14);
    rightSlave1.follow(rightMaster);
    rightSlave1.setInverted(false);


    rightSlave2 = new TalonSRX(16);
    rightSlave2.follow(rightMaster);
    rightSlave2.setInverted(false);


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
    SmartDashboard.putNumber("leftInput", periodicIO.leftInput);
    SmartDashboard.putNumber("leftOutput", periodicIO.leftOutput);
    SmartDashboard.putNumber("rightInput", periodicIO.rightInput);
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
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    leftMaster.set(ControlMode.PercentOutput, periodicIO.leftOutput);
    rightMaster.set(ControlMode.PercentOutput, periodicIO.rightOutput);
  }

  @Override
  public String id() {
    return TAG;
  }

  public static class PeriodicIO {
    public double timestamp;

    public double leftInput;
    public double rightInput;

    public double leftOutput;
    public double rightOutput;
  }
}
