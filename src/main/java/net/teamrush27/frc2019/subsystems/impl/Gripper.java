package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.Robot;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.util.math.CircularBuffer;
import net.teamrush27.frc2019.wrappers.InvertableDigitalInput;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Gripper extends Subsystem {

  private static final String TAG = "GRIPPER";
  private static final Logger LOG = LogManager.getLogger(Gripper.class);
  private static Gripper INSTANCE = null;

  public static Gripper getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Gripper();
    }
    return INSTANCE;
  }

  public enum WantedState {
    OFF, INTAKE_CARGO, EXHAUST_CARGO, INTAKE_HATCH, EXHAUST_HATCH
  }

  private enum SystemState {
    OFF, INTAKE_CARGO, HOLD_CARGO, EXHAUST_CARGO, INTAKE_HATCH, HOLD_HATCH, EXHAUST_HATCH
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.OFF;

  private boolean hasGamepiece = false;
  private boolean stateChanged = false;
  private double currentStateStartTime;

  private Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      currentStateStartTime = timestamp;
    }

    @Override
    public void onLoop(double timestamp) {
      SystemState newState;
      switch (systemState) {
        case INTAKE_CARGO:
          newState = handleIntakeCargo(timestamp);
          break;
        case HOLD_CARGO:
          newState = handleHoldCargo(timestamp);
          break;
        case EXHAUST_CARGO:
          newState = handleExhaustCargo(timestamp);
          break;
        case INTAKE_HATCH:
          newState = handleIntakeHatch(timestamp);
          break;
        case HOLD_HATCH:
          newState = handleHoldHatch(timestamp);
          break;
        case OFF:
        default:
          newState = handleOff(timestamp);
          break;
      }
      if (newState != systemState) {
        LOG.info("Gripper state {} to {}", systemState, newState);
        systemState = newState;
        currentStateStartTime = timestamp;
        stateChanged = true;
      } else {
        stateChanged = false;
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


  private final TalonSRX gripperMotor;
  private final TalonSRX jawMotor;

  private final DigitalInput kermit;

  private final DigitalInput jawHome;
  private final DigitalInput jawMax;
  
  private final CircularBuffer circularBuffer = new CircularBuffer(10);

  public Gripper() {
    gripperMotor = new TalonSRX(RobotMap.GRIPPER_MOTOR_CAN_ID);
    gripperMotor.setInverted(true);
    gripperMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    gripperMotor.configOpenloopRamp(.1, RobotConstants.TALON_CONFIG_TIMEOUT);
    gripperMotor.configContinuousCurrentLimit(30, RobotConstants.TALON_CONFIG_TIMEOUT);
    gripperMotor.enableCurrentLimit(true);

    jawMotor = new TalonSRX(RobotMap.GRIPPER_JAWS_CAN_ID);
    jawMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    jawMotor.configOpenloopRamp(.1, RobotConstants.TALON_CONFIG_TIMEOUT);
    jawMotor.configContinuousCurrentLimit(30, RobotConstants.TALON_CONFIG_TIMEOUT);
    jawMotor.enableCurrentLimit(true);
    jawMotor.setInverted(true);
    jawMotor.configVoltageCompSaturation(6);
    jawMotor.enableVoltageCompensation(false);

    kermit = new DigitalInput(RobotMap.GRIPPER_HATCH_DIGITAL_SENSOR_ID);
    jawHome = new InvertableDigitalInput(RobotMap.GRIPPER_JAW_HOME_SENSOR_ID, true);
    jawMax = new InvertableDigitalInput(RobotMap.GRIPPER_JAW_MAX_SENSOR_ID, true);
  }

  private double test = 0d;

  public void setTest(double test) {
    this.test = test;
  }

  private SystemState handleOff(double timestamp) {
    gripperMotor.set(ControlMode.Disabled, 0);

    homeJaws();

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleHoldHatch(double timestamp) {
    if (!jawMax.get()) {
      jawMotor.set(ControlMode.PercentOutput, 1);
    } else {
      jawMotor.set(ControlMode.Disabled, 1);
    }

    jawMotor.enableVoltageCompensation(true);

    if (WantedState.INTAKE_HATCH.equals(wantedState)) {
      return SystemState.HOLD_HATCH;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleIntakeHatch(double timestamp) {
    double delta = timestamp - currentStateStartTime;
    if (delta <= .375 && !jawMax.get()) {
      jawMotor.set(ControlMode.PercentOutput, 1);
    } else {
      jawMotor.set(ControlMode.Disabled, 0);
    }

    jawMotor.enableVoltageCompensation(false);

    gripperMotor.set(ControlMode.Disabled, 0);

    if (!kermit.get()) {
      return SystemState.HOLD_HATCH;
    }

    return defaultStateTransfer(timestamp);
  }

  double startExhaust = 0;

  private SystemState handleExhaustCargo(double timestamp) {
    if (startExhaust == 0) {
      startExhaust = timestamp;
    }
    firstFoundBall = 0;
    gripperMotor.set(ControlMode.PercentOutput, -1);

    homeJaws();

    if (WantedState.EXHAUST_CARGO.equals(wantedState) && timestamp - startExhaust > .5) {
      startExhaust = 0;
      wantedState = WantedState.OFF;
      return SystemState.OFF;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleHoldCargo(double timestamp) {
    firstFoundBall = 0;
    gripperMotor.set(ControlMode.PercentOutput, .2);

    homeJaws();

    //if (detective.getVoltage() < 1) {
      //return SystemState.INTAKE_CARGO;
    //}

    if (WantedState.EXHAUST_CARGO.equals(wantedState)) {
      return SystemState.EXHAUST_CARGO;
    }

    if (WantedState.INTAKE_CARGO.equals(wantedState)) {
      return SystemState.HOLD_CARGO;
    }

    return defaultStateTransfer(timestamp);
  }

  private double firstFoundBall = 0;

  private SystemState handleIntakeCargo(double timestamp) {
    if(stateChanged){
      circularBuffer.clear();
    }

    homeJaws();
    
    gripperMotor.set(ControlMode.PercentOutput, 1);
    
    if (WantedState.INTAKE_CARGO.equals(wantedState) && circularBuffer.getAverage() > 18 && circularBuffer.isFull()) {
      if (firstFoundBall == 0) {
        firstFoundBall = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp() - firstFoundBall > .1) {
        return SystemState.HOLD_CARGO;
      }
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState defaultStateTransfer(double timestamp) {
    switch (wantedState) {
      case INTAKE_CARGO:
        return SystemState.INTAKE_CARGO;
      case EXHAUST_CARGO:
        return SystemState.EXHAUST_CARGO;
      case INTAKE_HATCH:
        return SystemState.INTAKE_HATCH;
      default:
      case OFF:
      case EXHAUST_HATCH:
        return SystemState.OFF;
    }
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    circularBuffer.addValue(Robot.pdp.getCurrent(5));
    
    if (SystemState.HOLD_CARGO.equals(systemState)) {
      LED.getInstance().setHasGamePiece(true);
    } else if (SystemState.HOLD_HATCH.equals(systemState)
        && Timer.getFPGATimestamp() - currentStateStartTime > .5) {
      LED.getInstance().setHasGamePiece(true);
    } else {
      hasGamepiece = false;
      LED.getInstance().setHasGamePiece(false);
    }
    if (SystemState.INTAKE_HATCH.equals(systemState) || SystemState.INTAKE_CARGO
        .equals(systemState)) {
      LED.getInstance().setIntaking(true);
    } else {
      LED.getInstance().setIntaking(false);
    }
    if (SystemState.EXHAUST_CARGO.equals(systemState)) {
      LED.getInstance().setExhausting(true);
    } else {
      LED.getInstance().setExhausting(false);
    }

    //collection.setDetectiveVoltage(detective.getVoltage());
    //collection.setGripperState(systemState.toString());

    SmartDashboard.putString("gripper.state", systemState.toString());
    SmartDashboard.putNumber("gripper.amps", circularBuffer.getAverage());
  }

  @Override
  public void stop() {
    //wantedState = WantedState.OFF;
  }

  @Override
  public void zeroSensors() {

  }

  @Override
  public void test() {

  }

  private void homeJaws() {
    if (!jawHome.get()) {
      jawMotor.set(ControlMode.PercentOutput, -1);
    } else {
      jawMotor.set(ControlMode.Disabled, 0);
    }

    jawMotor.enableVoltageCompensation(false);
  }

  @Override
  public void writePeriodicOutputs() {
    //System.out.println(detective.getVoltage());
    //System.out.println(whatchman.get());
  }

  public void setWantedState(WantedState wantedState) {
    synchronized (wantedState) {
      this.wantedState = wantedState;
    }
  }

  public void transitionCargo() {
    synchronized (wantedState) {
      switch (wantedState) {
        case INTAKE_CARGO:
          wantedState = WantedState.EXHAUST_CARGO;
          break;
        case EXHAUST_CARGO:
          wantedState = WantedState.OFF;
          break;
        default:
          if (!SystemState.HOLD_HATCH.equals(systemState)) {
            wantedState = WantedState.INTAKE_CARGO;
          }
      }
    }
  }

  public void transitionHatch() {
    synchronized (systemState) {
      switch (systemState) {
        default:
        case OFF:
          wantedState = WantedState.INTAKE_HATCH;
          break;
        case INTAKE_HATCH:
          wantedState = WantedState.INTAKE_HATCH;
          systemState = SystemState.HOLD_HATCH;
          currentStateStartTime = Timer.getFPGATimestamp();
          break;
        case HOLD_HATCH:
          wantedState = WantedState.OFF;
          break;
        case HOLD_CARGO:
          break;
      }
    }
  }

  public boolean hasGamepiece() {
    return SystemState.HOLD_CARGO.equals(systemState) || SystemState.HOLD_HATCH.equals(systemState);
  }

  public boolean hasHatch() {
    return SystemState.HOLD_HATCH.equals(systemState);
  }

  public boolean hasCargo() {
    return SystemState.HOLD_CARGO.equals(systemState);
  }

  @Override
  public String id() {
    return TAG;
  }

}
