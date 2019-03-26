package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private static final Integer JAW_HATCH_INTAKE_POSITION = 250;
  private static final Integer JAW_RETRACT_POSITION = -23;
  private static final Integer JAW_HATCH_EXHAUST_POSITION = 306;

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
    OFF, INTAKE_CARGO, HOLD_CARGO, EXHAUST_CARGO, HOLD_HATCH, EXHAUST_HATCH
  }

  private WantedState wantedState = WantedState.INTAKE_HATCH;
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
        case EXHAUST_HATCH:
          newState = handleExhaustHatch(timestamp);
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
        zeroSensors();
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


  private final CANSparkMax gripperMotor;
//   private final TalonSRX gripperMotor;
  private final TalonSRX jawMotor;

  private final DigitalInput jawHome;
  private final DigitalInput jawMax;
  
  private final CircularBuffer circularBuffer = new CircularBuffer(10);

  public Gripper() {
    gripperMotor = new CANSparkMax(RobotMap.GRIPPER_MOTOR_SPARK_CAN_ID, MotorType.kBrushless);
    gripperMotor.restoreFactoryDefaults();
    gripperMotor.setIdleMode(IdleMode.kBrake);
    gripperMotor.setSmartCurrentLimit(80);
    gripperMotor.setOpenLoopRampRate(.1);
    gripperMotor.enableVoltageCompensation(12);
    gripperMotor.getPIDController().setP(.1);
    gripperMotor.getPIDController().setI(0);
    gripperMotor.getPIDController().setD(0);
    
//    gripperMotor = new TalonSRX(RobotMap.GRIPPER_MOTOR_CAN_ID);
//    gripperMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
//    gripperMotor.setNeutralMode(NeutralMode.Brake);
//    gripperMotor.configContinuousCurrentLimit(30);
//    gripperMotor.configVoltageCompSaturation(12);
//    gripperMotor.enableVoltageCompensation(true);
    

    jawMotor = new TalonSRX(RobotMap.GRIPPER_JAWS_CAN_ID);
    jawMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    jawMotor.enableCurrentLimit(true);
    jawMotor.setInverted(true);
    jawMotor.setSensorPhase(false);
    jawMotor.configContinuousCurrentLimit(20);
    jawMotor.configVoltageCompSaturation(12);
    jawMotor.enableVoltageCompensation(true);
    jawMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    jawMotor.config_kP(0, 100);
    jawMotor.config_kI(0, 0);
    jawMotor.config_kD(0, 1000);

    jawHome = new InvertableDigitalInput(RobotMap.GRIPPER_JAW_HOME_SENSOR_ID, true);
    jawMax = new InvertableDigitalInput(RobotMap.GRIPPER_JAW_MAX_SENSOR_ID, true);
  }

  private SystemState handleOff(double timestamp) {
    gripperMotor.set(0);
    //gripperMotor.set(ControlMode.PercentOutput, 0);
    jawMotor.set(ControlMode.Position, JAW_RETRACT_POSITION);

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleHoldHatch(double timestamp) {

    jawMotor.set(ControlMode.Position, JAW_HATCH_INTAKE_POSITION);
    gripperMotor.set(0);
    //gripperMotor.set(ControlMode.PercentOutput, 0);

    if (WantedState.INTAKE_HATCH.equals(wantedState)) {
      return SystemState.HOLD_HATCH;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleExhaustHatch(double timestamp) {
    jawMotor.set(ControlMode.Position, JAW_HATCH_EXHAUST_POSITION);
    gripperMotor.set(0);
    //gripperMotor.set(ControlMode.PercentOutput, 0);
  
    return defaultStateTransfer(timestamp);
  }

  double startExhaust = 0;

  private SystemState handleExhaustCargo(double timestamp) {
    if (startExhaust == 0) {
      startExhaust = timestamp;
    }
    firstFoundBall = 0;
    gripperMotor.set(-1);
    //gripperMotor.set(ControlMode.PercentOutput, -1);

    jawMotor.set(ControlMode.Position, JAW_RETRACT_POSITION);

    if (WantedState.EXHAUST_CARGO.equals(wantedState) && timestamp - startExhaust > .2) {
      startExhaust = 0;
      wantedState = WantedState.OFF;
      return SystemState.OFF;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleHoldCargo(double timestamp) {
    firstFoundBall = 0;
    if(stateChanged){
      gripperMotor.setSmartCurrentLimit(30);
      //gripperMotor.configVoltageCompSaturation(3);
    }

    gripperMotor.set(.2);
    //gripperMotor.set(ControlMode.PercentOutput, 1);
    jawMotor.set(ControlMode.Position, JAW_RETRACT_POSITION);

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
      gripperMotor.setSmartCurrentLimit(80);
      //gripperMotor.configVoltageCompSaturation(12);
    }

    gripperMotor.set(.33);
    //gripperMotor.set(ControlMode.PercentOutput, 1);
    jawMotor.set(ControlMode.Position, JAW_RETRACT_POSITION);

    if (WantedState.INTAKE_CARGO.equals(wantedState) && circularBuffer.getAverage() > 30 && circularBuffer.isFull() && timestamp - currentStateStartTime > .1) {
      if (gripperMotor.getEncoder().getVelocity() < 10){
        if (firstFoundBall == 0) {
          firstFoundBall = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - firstFoundBall > .25) {
          return SystemState.HOLD_CARGO;
        }
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
        return SystemState.HOLD_HATCH;
      case EXHAUST_HATCH:
        return SystemState.EXHAUST_HATCH;
      default:
      case OFF:
        return SystemState.OFF;
    }
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    circularBuffer.addValue(Math.abs(gripperMotor.getOutputCurrent()));
    
    if (SystemState.HOLD_CARGO.equals(systemState)) {
      LED.getInstance().setHasGamePiece(true);
    } else if (SystemState.HOLD_HATCH.equals(systemState)) {
      LED.getInstance().setHasGamePiece(true);
    } else {
      hasGamepiece = false;
      LED.getInstance().setHasGamePiece(false);
    }
    if (SystemState.INTAKE_CARGO
        .equals(systemState)) {
      LED.getInstance().setIntaking(true);
    } else {
      LED.getInstance().setIntaking(false);
    }
    if (SystemState.EXHAUST_CARGO.equals(systemState) || SystemState.EXHAUST_HATCH.equals(systemState)) {
      LED.getInstance().setExhausting(true);
    } else {
      LED.getInstance().setExhausting(false);
    }

    //collection.setDetectiveVoltage(detective.getVoltage());
    //collection.setGripperState(systemState.toString());

    SmartDashboard.putString("gripper.state", systemState.toString());
    SmartDashboard.putNumber("gripper.amps", circularBuffer.getMax());
    SmartDashboard.putNumber("gripper.pdpAmps", Robot.pdp.getCurrent(4));
    SmartDashboard.putNumber("gripper.speed", gripperMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("gripper.jaw.position", jawMotor.getSelectedSensorPosition());
  }

  @Override
  public void stop() {
    //wantedState = WantedState.OFF;
  }

  @Override
  public void zeroSensors() {
    int position = (jawMotor.getSelectedSensorPosition() % 1024 + 1024) % 1024;
    int position_wrapped = position >= 512? position - 1024 : position;
    jawMotor.setSelectedSensorPosition(position_wrapped);
  }

  @Override
  public void test() {

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
        case EXHAUST_HATCH:
          wantedState = WantedState.OFF;
          break;
        case HOLD_HATCH:
          wantedState = WantedState.EXHAUST_HATCH;
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
