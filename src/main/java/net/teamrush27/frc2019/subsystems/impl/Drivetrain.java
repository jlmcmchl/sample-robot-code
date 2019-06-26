package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.subsystems.impl.enumerated.DriveMode;
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

  private final TalonSRX leftMaster;
  private final TalonSRX leftSlave1;
  private final TalonSRX rightMaster;
  private final TalonSRX rightSlave1;
  private final TalonSRX leftSlave2;
  private final TalonSRX rightSlave2;
  private double timeSinceModeSwitch;

  private DriveMode driveMode = DriveMode.OPEN_LOOP;

  private boolean brakeMode = true;
  private boolean modeChanged = false;
  
  private PeriodicIO periodicIO;
  
  private final Loop loop = new Loop() {
    private DriveMode lastMode = DriveMode.OPEN_LOOP;

    @Override
    public void onStart(double timestamp) {
      synchronized (Drivetrain.this) {
        setOpenLoop(DriveCommand.COAST);
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

  public Drivetrain() {
    periodicIO = new PeriodicIO();


    leftMaster = new TalonSRX(RobotMap.DRIVE_LEFT_MASTER_CAN_ID);
    leftMaster.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    leftMaster.setInverted(false);

    leftSlave1 = new TalonSRX(RobotMap.DRIVE_LEFT_SLAVE_1_CAN_ID);
    leftSlave1.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave1.overrideLimitSwitchesEnable(false);
    leftSlave1.follow(leftMaster);
    leftSlave1.setInverted(false);

    leftSlave2 = new TalonSRX(RobotMap.DRIVE_LEFT_SLAVE_2_CAN_ID);
    leftSlave2.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    leftSlave2.follow(leftMaster);
    leftSlave2.setInverted(false);

    rightMaster = new TalonSRX(RobotMap.DRIVE_RIGHT_MASTER_CAN_ID);
    rightMaster.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    rightMaster.setInverted(true);
  
    rightSlave1 = new TalonSRX(RobotMap.DRIVE_RIGHT_SLAVE_1_CAN_ID);
    rightSlave1.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave1.follow(rightMaster);
    rightSlave1.setInverted(true);

    rightSlave2 = new TalonSRX(RobotMap.DRIVE_RIGHT_SLAVE_2_CAN_ID);
    rightSlave2.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    rightSlave2.follow(rightMaster);
    rightSlave2.setInverted(true);

    periodicIO = new PeriodicIO();
    
    setBrakeMode(false);
  }


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

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
  }

  @Override
  public void stop() {
    setOpenLoop(DriveCommand.COAST);
  }

  public synchronized void setOpenLoop(DriveCommand command) {
    if (!DriveMode.OPEN_LOOP.equals(driveMode)) {
      driveMode = DriveMode.OPEN_LOOP;
      setBrakeMode(command.getBrakeMode());

      leftMaster.configNeutralDeadband(0.04, 0);
      rightMaster.configNeutralDeadband(0.04, 0);
    }

    periodicIO.left_demand = command.getLeftDriveInput();
    periodicIO.right_demand = command.getRightDriveInput();
  }

  @Override
  public void zeroSensors() {
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
    if (driveMode == DriveMode.OPEN_LOOP) {
      leftMaster
          .set(ControlMode.PercentOutput, periodicIO.left_demand);
      rightMaster
          .set(ControlMode.PercentOutput, periodicIO.right_demand);
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

    // OUTPUTS
    public double left_demand;
    public double right_demand;

    public PeriodicIO() {

    }

    public PeriodicIO(PeriodicIO other) {
      this.timestamp = other.timestamp;
      this.left_demand = other.left_demand;
      this.right_demand = other.right_demand;
    }
  }
}
