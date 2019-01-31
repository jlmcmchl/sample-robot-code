package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.wrappers.CANTalonFactory;

public class Arm extends Subsystem {

  private static String TAG = "ARM";

  private static Arm INSTANCE = null;

  public static Arm getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Arm();
    }
    return INSTANCE;
  }


  public enum WantedState {
    OFF, OPEN_LOOP
  }

  public enum SystemState {
    OFF, OPEN_LOOP
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.OFF;


  private final TalonSRX rotationMotor;
  private final TalonSRX extensionMotor;

  private final AnalogPotentiometer rotationPot;
  private final AnalogPotentiometer extensionPot;

  private boolean stateChanged = false;
  private double currentStateStartTime;


  private static double MAX_ROTATION = 339.12;
  private static double MIN_ROTATION = 46.58;
  private static double MAX_EXTENSION = 108.78;
  private static double MIN_EXTENSION = 35.83;

  private ArmInput openLoopInput = new ArmInput(0d, 0d);


  private Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      currentStateStartTime = timestamp;
    }

    @Override
    public void onLoop(double timestamp) {
      SystemState newState;
      switch (systemState) {
        case OPEN_LOOP:
          newState = handleOpenLoop(timestamp);
          break;
        case OFF:
        default:
          newState = handleOff(timestamp);
          break;
      }
      if (newState != systemState) {
        System.out.println("Arm state " + systemState + " to " + newState);
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

  public Arm() {
    rotationMotor = CANTalonFactory.createDefaultTalon(RobotMap.ARM_ROTATION_CAN_ID);
    rotationMotor.setNeutralMode(NeutralMode.Brake);

    extensionMotor = CANTalonFactory.createDefaultTalon(RobotMap.ARM_EXTENSION_CAN_ID);
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    rotationPot = new AnalogPotentiometer(RobotMap.ARM_ROTATION_POT_CHANNEL);
    extensionPot = new AnalogPotentiometer(RobotMap.ARM_EXTENSION_POT_CHANNEL);
  }

  @Override
  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("armPot", getArmAngle());
    SmartDashboard.putNumber("extPot", getArmExtension());
  }

  private SystemState handleOpenLoop(double timestamp) {

    Double rotationInput = openLoopInput.getRotationInput();
    Double extensionInput = openLoopInput.getExtensionInput();

    if (getArmAngle() <= 28d) {
      rotationInput = Math.min(0, rotationInput);
    }

    if (getArmExtension() >= 36d) {
      extensionInput = Math.max(0, extensionInput);
    } else if (getArmExtension() <= 2d) {
      extensionInput = Math.min(0, extensionInput);
    }

    System.out.println(
        "rot : " + getArmAngle() + " rot 2 : " + rotationInput + " - ext : " + getArmExtension());

    extensionMotor.set(ControlMode.PercentOutput, extensionInput);
    rotationMotor.set(ControlMode.PercentOutput, rotationInput);

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleOff(double timestamp) {
    extensionMotor.set(ControlMode.PercentOutput, 0);
    rotationMotor.set(ControlMode.PercentOutput, 0);

    return defaultStateTransfer(timestamp);
  }

  private SystemState defaultStateTransfer(double timestamp) {
    switch (wantedState) {
      case OPEN_LOOP:
        return SystemState.OPEN_LOOP;
      case OFF:
      default:
        return SystemState.OFF;
    }
  }

  @Override
  public void stop() {
    wantedState = WantedState.OFF;
  }

  @Override
  public void zeroSensors() {
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(loop);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void reset() {
    systemState = SystemState.OFF;
  }

  @Override
  public void test() {
  }

  public double getDistanceOutsideFramePerimeter() {
    double a = 0d;
    double b = 0d;
    double c = 0d;

    return 0d;
  }

  public void setOpenLoopInput(ArmInput armInput) {
    synchronized (openLoopInput) {
      openLoopInput = armInput;
    }
  }

  public SystemState getSystemState() {
    return systemState;
  }


  private double getArmAngle() {
    return (rotationPot.get() * MAX_ROTATION) - MIN_ROTATION;
  }

  private double getArmExtension() {
    return (extensionPot.get() * MAX_EXTENSION) - MIN_EXTENSION;
  }

}