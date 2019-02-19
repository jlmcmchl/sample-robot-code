package net.teamrush27.frc2019.managers;

import java.util.LinkedList;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Arm.ArmState;
import net.teamrush27.frc2019.subsystems.impl.Wrist;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.util.interpolate.InterpolatingDouble;
import net.teamrush27.frc2019.util.math.MathUtils;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class SuperstructureManager extends Subsystem {

  private static final String TAG = "SSMAN";
  private static SuperstructureManager INSTANCE = null;
  private static final double ROTATION_EPSILON = 10d;
  private static final double EXTENSION_EPSILON = 4d;
  private static final double ARM_BASE_LENGTH = 15.5d;
  private static final double ARM_MAX_EXTENSION = 47d;
  private static final double MAX_LATERAL_EXTENSION = 32d;

  private static final Logger LOG = LogManager.getLogger(SuperstructureManager.class);

  private static final double NUM_SEGMENTS = 5;

  public static SuperstructureManager getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SuperstructureManager();
    }

    return INSTANCE;
  }

  private Arm arm = Arm.getInstance();
  private Wrist wrist = Wrist.getInstance();

  public enum WantedState {
    CARGO_GROUND_PICKUP(new ArmInput(5d, 88d), 50d),
    HUMAN_LOAD(new ArmInput(10d, 32d), new ArmInput(5d, 90d), 52d, 0d),
    CARGO_SHIP(new ArmInput(12.5d, 27d), 63d),
    ROCKET_LEVEL_1(new ArmInput(7d, 70d), new ArmInput(5d, 90d), 21d, 0d),
    ROCKET_LEVEL_2(new ArmInput(26d, 32d), new ArmInput(18d, 39d), 57d, 48d),
    ROCKET_LEVEL_3(new ArmInput(47d, 18d), new ArmInput(44d, 22d), 45d, 67d),
    STOW(new ArmInput(5d, 25d), 0d),
    CLIMB(new ArmInput(5d, 45d), 0d),
    START(new ArmInput(0d, 0d), 0d);

    private final ArmInput defaultInput;
    private final ArmInput hatchInput;
    private Double defaultWristAngle;
    private Double hatchWristAngle;
    
    WantedState(ArmInput armInput, double wristAngle) {
      this(armInput, null, wristAngle, null);
    }

    WantedState(ArmInput armInput, ArmInput hatchInput, Double defaultWristAngle, Double hatchWristAngle) {
      this.defaultInput = armInput;
      this.hatchInput = hatchInput;
      this.defaultWristAngle = defaultWristAngle;
      this.hatchWristAngle = hatchWristAngle;
    }

    public ArmInput getDefaultInput() {
      return defaultInput;
    }

    public ArmInput getHatchInput() {
      return hatchInput;
    }

    public Double getDefaultWristAngle() {
      return defaultWristAngle;
    }
  
    public Double getHatchWristAngle() {
      return hatchWristAngle;
    }
  }

  private WantedState newWantedState = WantedState.STOW;
  private WantedState wantedState = WantedState.START;
  private Boolean newInvertedRotation = false;
  private Boolean invertedRotation = false;
  private Boolean hasHatch = false;
  private Command origin = new Command(0d, 0d, null);
  private LinkedList<Command> commands = new LinkedList<>();

  public synchronized void setWantedState(WantedState wantedState, Boolean invertedRotation, Boolean hasHatch) {
    this.newWantedState = wantedState;
    this.newInvertedRotation = invertedRotation;
    this.hasHatch = hasHatch;
  }

  @Override
  public void outputToSmartDashboard() {
    //LOG.info("rot: {} ext: {} wrist: {}", arm.getArmState().getRotationInDegrees(), arm.getArmState().getExtensionInInches(), wrist.getEncoderAngle());
  }

  @Override
  public void stop() {
    wantedState = WantedState.STOW;
  }

  @Override
  public void zeroSensors() {
    /*LOG.info("Package to Stow");
    origin = new Command(0d, 0d, null);
    wantedState = WantedState.STOW;
    recomputeOperations();

    LOG.info("Stow to CARGO_GROUND_PICKUP");
    origin = new Command(0d, 5d, null);
    wantedState = WantedState.CARGO_GROUND_PICKUP;
    recomputeOperations();

    LOG.info("Stow to CARGO_SHIP");
    origin = new Command(0d, 5d, null);
    wantedState = WantedState.CARGO_SHIP;
    recomputeOperations();

    LOG.info("Stow to HUMAN_LOAD");
    origin = new Command(0d, 5d, null);
    wantedState = WantedState.HUMAN_LOAD;
    recomputeOperations();*/

    /*LOG.info("Stow to ROCKET_LEVEL_1");
    origin = new Command(0d, 0d, null);
    wantedState = WantedState.ROCKET_LEVEL_1;
    recomputeOperations();
*/
    /*

    LOG.info("Stow to ROCKET_LEVEL_2");
    origin = new Command(0d, 5d, null);
    wantedState = WantedState.ROCKET_LEVEL_2;
    recomputeOperations();*/

    /*LOG.info("Stow to ROCKET_LEVEL_3");
    origin = new Command(0d, 5d, null);
    wantedState = WantedState.ROCKET_LEVEL_3;
    recomputeOperations();*/
/*
    LOG.info("Stow to CLIMB");
    origin = new Command(0d, 5d, null);
    wantedState = WantedState.CLIMB;
    recomputeOperations();*/
/*
    LOG.info("ROCKET_LEVEL_2 to ROCKET_LEVEL_3");
    origin = new Command(-6d, 22d, -84,null);
    wantedState = WantedState.ROCKET_LEVEL_3;
    recomputeOperations();*/
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(new Loop() {
      @Override
      public void onStart(double timestamp) {
      }

      @Override
      public void onLoop(double timestamp) {
        handlePosition(timestamp);
      }

      @Override
      public void onStop(double timestamp) {
        stop();
      }

      @Override
      public String id() {
        return TAG;
      }
    });
  }

  // Every Loop:
  //

  public void handlePosition(double timestamp) {
    if (wantedState != newWantedState || invertedRotation != newInvertedRotation) {
      LOG.info(String.format("%s : %s - %s : %s", wantedState, invertedRotation, newWantedState, newInvertedRotation));
      wantedState = newWantedState;
      invertedRotation = newInvertedRotation;
      recomputeOperations();
    }

    //System.out.println(String.format("Current: %s", wrist.getPWMAngle()));
    if (commands.isEmpty()) {
      return;
    }

    executeCommand(commands.peek());

    if (operationComplete(commands.peek())) {
      commands.poll();
    }
  }

  // TODO
  // I don't like how this handles arm extension while at 30" out.
  // If we're moving from one level to another while at 30", we risk violating our extension limit
  // if extension and rotation don't move in sync.
  // Possible solution to put setpoints at 25" out instead
  // and creep towards like we had kyle move the elevator last year

  // TODO HOW WERE ACTUALLY GOING TO DO IT
  // 2 INTERMEDIATE POINTS
  // FIRST IS ANGLE WHERE EXTENSION CAN REACH 5" FROM START POINT
  // SECOND IS WHERE EXTENSION CAN LEAVE FROM 5" AND REACH GOAL AT SAME TIME AS ROTATION

  // OR

  // 1 SETPOINT WHERE WE CAN GET TO AND THEN GET TO GOAL WITHIN ROTATION'S MOTION


  private void recomputeOperations() {
    commands.clear();
    ArmState armState = arm.getArmState();

    LOG.info(
        String.format("Origin: Rot: %s\tExt: %s\tWrs: %s", armState.getRotationInDegrees(),
            armState.getExtensionInInches(), wrist.getPWMAngle()));
    /*LOG.info(
        String.format("Origin: Rot: %s\tExt: %s\tWrs: %s", origin.getArmInput().getRotationInput(),
            origin.getArmInput().getExtensionInput(), origin.getDefaultWristAngle()));*/
    LOG.info(String.format("Wanted: Rot: %s\tExt: %s\tWrs: %s", getWantedRotation(),
        getWantedExtension(), getWantedWristAngle()));

    InterpolatingDouble initialRotation = new InterpolatingDouble(
        //    origin.getArmInput().getRotationInput());
        armState.getRotationInDegrees());
    InterpolatingDouble wantedRotation = new InterpolatingDouble(getWantedRotation());
    InterpolatingDouble initialExtension = new InterpolatingDouble(
        //    origin.getArmInput().getExtensionInput());
        armState.getExtensionInInches());
    InterpolatingDouble wantedExtension = new InterpolatingDouble(getWantedExtension());
    InterpolatingDouble initialWristAngle = new InterpolatingDouble(
        //    origin.getDefaultWristAngle());
        wrist.getPWMAngle());
    InterpolatingDouble wantedWristAngle = new InterpolatingDouble(getWantedWristAngle());

    boolean extension_bound = false;

    double initial_extension_bounded = boundExtensionForAngle(initialRotation.value,
        initialExtension.value);
    if (initial_extension_bounded != initialExtension.value) {
      initialExtension = new InterpolatingDouble(initial_extension_bounded);
      commands.push(new Command(initialRotation.value, initial_extension_bounded,
          initialWristAngle.value,
          initial_extension_bounded < initialExtension.value
              ? Limit.EXTENSION_MAXIMUM
              : Limit.EXTENSION_MINIMUM));

      if (initial_extension_bounded < initialExtension.value) {
        LOG.info(String.format("Command: XTNS_MAX Rot: %s\tExt: %s\tWrs: %s",
            initialRotation.value, initial_extension_bounded, initialWristAngle.value));
      } else {
        LOG.info(String.format("Command: XTNS_MIN Rot: %s\tExt: %s\tWrs: %s",
            initialRotation.value, initial_extension_bounded, initialWristAngle.value));
      }
    }

    // If angle diff is 0 then idgaf idontgiveafuck
    if (!MathUtils.epsilonEquals(initialExtension.value, wantedExtension.value, EXTENSION_EPSILON)) {
      for (int i = 1; i < NUM_SEGMENTS; i++) {
        InterpolatingDouble angle = initialRotation
            .interpolate(wantedRotation, i * 1.0 / NUM_SEGMENTS);

        InterpolatingDouble extension = initialExtension
            .interpolate(wantedExtension, i * 1.0 / NUM_SEGMENTS);

        InterpolatingDouble wristAngle = initialWristAngle
            .interpolate(wantedWristAngle, i * 1.0 / NUM_SEGMENTS);

        double bound_extension = boundExtensionForAngle(angle.value, extension.value);

        double bound_wristAngle = boundWristAngleForExtension(bound_extension, wristAngle.value);

        if (bound_extension != extension.value) {
          if (bound_extension < extension.value) {
            LOG.info(String.format("Command: XTNS_MAX Rot: %s\tExt: %s\tWrs: %s",
                angle.value, bound_extension, bound_wristAngle));
          } else {
            LOG.info(String.format("Command: XTNS_MIN Rot: %s\tExt: %s\tWrs: %s",
                angle.value, bound_extension, bound_wristAngle));
          }
          commands
              .add(new Command(angle.value, bound_extension, bound_wristAngle,
                  bound_extension < extension.value ? Limit.EXTENSION_MAXIMUM
                      : Limit.EXTENSION_MINIMUM));
        } else if (extension_bound && bound_extension == extension.value) {
          LOG.info(String.format("Command: ROTATION  Rot: %s\tExt: %s\tWrs: %s",
              angle.value, bound_extension, bound_wristAngle));

          commands.add(new Command(angle.value, bound_extension, bound_wristAngle, Limit.ROTATION));
        } else {
          LOG.info(String.format("Command: XTNS_ANY Rot: %s\tExt: %s\tWrs: %s",
              angle.value, bound_extension, bound_wristAngle));

          commands.add(new Command(angle.value, bound_extension, bound_wristAngle, Limit.EXTENSION_MAXIMUM));
        }

        extension_bound = bound_extension != extension.value;
      }
    }

    LOG.info(String.format("Command: NONE      Rot: %s\tExt: %s\tWrs: %s",
        wantedRotation.value, wantedExtension.value, wantedWristAngle.value));
    LOG.info("");

    commands.add(new Command(wantedRotation.value, wantedExtension.value, wantedWristAngle.value,
        Limit.NONE));
  }

  private boolean operationComplete(Command current) {
    double currentRotation = arm.getArmState().getRotationInDegrees();
    double goalRotation = current.getArmInput().getRotationInput();
    double goalExtension = current.getArmInput().getExtensionInput();
    double currentExtension = arm.getArmState().getExtensionInInches();

    switch (current.getLimitType()) {
      case ROTATION:
        return MathUtils.epsilonEquals(currentRotation, goalRotation, ROTATION_EPSILON);
      case EXTENSION_MINIMUM:
        return MathUtils.epsilonEquals(currentRotation, goalRotation, ROTATION_EPSILON)
            && (currentExtension > goalExtension
              || MathUtils.epsilonEquals(currentExtension, goalExtension, EXTENSION_EPSILON));
      case EXTENSION_MAXIMUM:
        return MathUtils.epsilonEquals(currentRotation, goalRotation, ROTATION_EPSILON)
            && (currentExtension < goalExtension
              || MathUtils.epsilonEquals(currentExtension, goalExtension, EXTENSION_EPSILON));
      default:
        return false;
    }
  }

  private void executeCommand(Command command) {
    arm.setClosedLoopInput(command.getArmInput());
    wrist.setClosedLoopInput(command.getWristAngle());
  }

  private double getWantedRotation() {
    if ((hasHatch && wantedState.getHatchInput() != null) || wantedState.getDefaultInput() == null) {
      return wantedState.getHatchInput().getRotationInput() * (invertedRotation ? -1 : 1);
    }
    return wantedState.getDefaultInput().getRotationInput() * (invertedRotation ? -1 : 1);
  }

  private double getWantedExtension() {
    if ((hasHatch && wantedState.getHatchInput() != null) || wantedState.getDefaultInput() == null) {
      return wantedState.getHatchInput().getExtensionInput();
    }
    return wantedState.getDefaultInput().getExtensionInput();
  }

  private double getWantedWristAngle() {
    if((hasHatch && wantedState.getHatchWristAngle() != null) || wantedState.getDefaultWristAngle() == null){
      return wantedState.getHatchWristAngle() * (invertedRotation ? -1 : 1);
    }
    return wantedState.getDefaultWristAngle() * (invertedRotation ? -1 : 1);
  }

  private double boundExtensionForAngle(final double angle, final double extension) {
    final double flooredExtension = Math.min(extension, getMaxExtensionForAngle(angle));
    final double ceiledExtension = Math.max(flooredExtension, getMinExtensionForAngle(angle));
    return ceiledExtension;
  }

  private double getMaxExtensionForAngle(double angle) {
    if (Math.abs(angle) <= 90) {
      double max_lateral_extension_for_angle =
          MAX_LATERAL_EXTENSION - getExtensionBufferForAngle(angle)
              - Math.sin(angle * Math.PI / 180) * ARM_BASE_LENGTH;
      double max_extension_for_angle =
          max_lateral_extension_for_angle / Math.sin(angle * Math.PI / 180);
      return Math.min(ARM_MAX_EXTENSION, Math.abs(max_extension_for_angle));
    } else {
      return MAX_LATERAL_EXTENSION - ARM_BASE_LENGTH;
    }
  }

  private double getMinExtensionForAngle(double angle) {
    return 0d;
  }

  private double getExtensionBufferForAngle(final double angle) {
    return 5d;
  }

  private double boundWristAngleForExtension(double extension, double wristAngle) {
    double max_wristAngle = MaxWristAngleForExtension(extension);
    return Math.signum(wristAngle) * Math.min(max_wristAngle, Math.abs(wristAngle));
  }

  private double MaxWristAngleForExtension(double extension) {
    if (extension < 4) {
      return 45;
    } else if (4 <= extension && extension < 12) {
      return 70;
    } else {
      return 90;
    }
  }

  @Override
  public void test() {

  }

  @Override
  public String id() {
    return TAG;
  }


  private enum Limit {
    ROTATION, EXTENSION_MINIMUM, EXTENSION_MAXIMUM, NONE
  }

  private class Command {

    private final ArmInput armInput;
    private final double wristAngle;
    private final Limit limitType;

    protected Command(double rotationDemand, double extensionDemand, double wristAngle,
        Limit limitType) {
      this.armInput = new ArmInput(extensionDemand, rotationDemand);
      this.wristAngle = wristAngle;
      this.limitType = limitType;
    }

    protected Command(double rotationDemand, double extensionDemand, Limit limitType) {
      this(rotationDemand, extensionDemand, 0, limitType);
    }

    public ArmInput getArmInput() {
      return armInput;
    }

    public double getWristAngle() {
      return wristAngle;
    }

    public Limit getLimitType() {
      return limitType;
    }

  }
}
