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
  private static final double WRIST_EPSILON = 20d;
  private static final double ARM_BASE_LENGTH = 15.5d;
  private static final double ARM_MIN_EXTENSION = 5;
  private static final double ARM_MAX_EXTENSION = 47d;
  private static final double MIN_LATERAL_EXTENSION = 2d;
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
    CARGO_GROUND_PICKUP(new ArmInput(5d, 93d), 50d),
    HUMAN_LOAD(new ArmInput(12.6d, 28.75d), new ArmInput(5d, 90d), 52d, 0d),
    CARGO_SHIP(new ArmInput(12.5d, 27d), 63d),
    ROCKET_LEVEL_1(new ArmInput(8.1d, 63.8d), new ArmInput(5d, 90d), 21d, 0d),
    ROCKET_LEVEL_2(new ArmInput(18d, 6.7d), new ArmInput(13d, 15.5d), 60d, 71d),
    ROCKET_LEVEL_3(new ArmInput(46d, 2.85d), new ArmInput(40d, 7.7d), 63d, 80d),
    STOW(new ArmInput(5d, 0d), 0d),
    CLIMB(new ArmInput(5d, 45d), 0d),
    START(new ArmInput(0d, 0d), 0d);

    private final ArmInput defaultInput;
    private final ArmInput hatchInput;
    private Double defaultWristAngle;
    private Double hatchWristAngle;

    WantedState(ArmInput armInput, double wristAngle) {
      this(armInput, null, wristAngle, null);
    }

    WantedState(ArmInput armInput, ArmInput hatchInput, Double defaultWristAngle,
        Double hatchWristAngle) {
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
  private Boolean force_recompute = true;
  //private Command origin = new Command(0d, 0d, null);
  private LinkedList<Command> commands = new LinkedList<>();

  private Double offset;

  public synchronized void setWantedState(WantedState wantedState, Boolean invertedRotation,
      Boolean hasHatch) {
    this.newWantedState = wantedState;
    this.newInvertedRotation = invertedRotation;
    this.hasHatch = hasHatch;
  }

  public void must_recompute() {
    force_recompute = true;
  }

  @Override
  public void outputToSmartDashboard() {
    //LOG.info("rot: {} ext: {} wrist: {}", arm.getArmState().getRotationInDegrees(), arm.getArmState().getExtensionInInches(), wrist.getEncoderAngle());
  }

  @Override
  public void stop() {
    wantedState = WantedState.STOW;
  }

  private void printCommands() {
    for (Command command : commands) {
      LOG.info(String.format("COMMAND ROT: %s\tEXT: %s\tWRS: %s\tLIM: %s",
          command.getArmInput().getRotationInput(), command.getArmInput().getExtensionInput(),
          command.getWristAngle(), command.getLimitType()));
    }
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
    if (wantedState != newWantedState || invertedRotation != newInvertedRotation
        || force_recompute) {
      LOG.info(String.format("%s : %s - %s : %s", wantedState, invertedRotation, newWantedState,
          newInvertedRotation));
      wantedState = newWantedState;
      invertedRotation = newInvertedRotation;
      force_recompute = false;
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

  private void recomputeOperations() {
    commands.clear();
    ArmState armState = arm.getArmState();

    LOG.info(
        String.format("Origin: Rot: %s\tExt: %s\tWrs: %s", armState.getRotationInDegrees(),
            armState.getExtensionInInches(), wrist.getPWMAngle()));
    /*LOG.info(
        String.format("Origin: Rot: %s\tExt: %s\tWrs: %s", origin.getArmInput().getRotationInput(),
            origin.getArmInput().getExtensionInput(), origin.getWristAngle()));*/
    LOG.info(String.format("Wanted: Rot: %s\tExt: %s\tWrs: %s", getWantedRotation(),
        getWantedExtension(), getWantedWristAngle()));

    commands.add(
        new Command(armState.getRotationInDegrees(), armState.getExtensionInInches(),
            0d, Limit.WRIST));

    double theta0 = armState.getRotationInDegrees();
    int startingQuadrant = theta0 < -45 ? 0 : theta0 < 0 ? 1 : theta0 < 45 ? 2 : 3;

    double theta1 = getWantedRotation();
    int finalQuadrant = theta1 < -45 ? 0 : theta1 < 0 ? 1 : theta1 < 45 ? 2 : 3;

    if (startingQuadrant != finalQuadrant) {
      commands.add(
          new Command(armState.getRotationInDegrees(), 5d,
              0d, Limit.EXTENSION));

      commands.add(
          new Command(getWantedRotation(), 5d,
              0d, Limit.ROTATION));
    } else {
      commands.add(
          new Command(getWantedRotation(), armState.getExtensionInInches(),
              0d, Limit.ROTATION));
    }

    commands.add(
        new Command(getWantedRotation(), getWantedExtension(),
            getWantedWristAngle(), Limit.NONE));

    printCommands();
  }

  private boolean operationComplete(Command current) {
    double currentRotation = arm.getArmState().getRotationInDegrees();
    double goalRotation = current.getArmInput().getRotationInput();
    double goalExtension = current.getArmInput().getExtensionInput();
    double currentExtension = arm.getArmState().getExtensionInInches();
    double currentWrist = wrist.getEncoderAngle();
    double goalWrist = current.getWristAngle();

    switch (current.getLimitType()) {
      case ROTATION:
        return MathUtils.epsilonEquals(currentRotation, goalRotation, ROTATION_EPSILON);
      case EXTENSION:
        return MathUtils.epsilonEquals(currentExtension, goalExtension, EXTENSION_EPSILON);
      case WRIST:
        return MathUtils.epsilonEquals(currentWrist, goalWrist, WRIST_EPSILON);
      default:
        return true;
    }
  }

  private void executeCommand(Command command) {
    LOG.info(String.format("Current Command: ROT: %s\tEXT: %s\tWRS: %s",
        command.getArmInput().getRotationInput(),
        command.getArmInput().getExtensionInput(),
        command.getWristAngle()));

    arm.setClosedLoopInput(command.getArmInput());
    wrist.setClosedLoopInput(command.getWristAngle());
  }

  private double getWantedRotation() {
    if ((hasHatch && wantedState.getHatchInput() != null)
        || wantedState.getDefaultInput() == null) {
      return wantedState.getHatchInput().getRotationInput() * (invertedRotation ? -1 : 1);
    }
    return wantedState.getDefaultInput().getRotationInput() * (invertedRotation ? -1 : 1);
  }

  private double getWantedExtension() {
    if ((hasHatch && wantedState.getHatchInput() != null)
        || wantedState.getDefaultInput() == null) {
      return wantedState.getHatchInput().getExtensionInput();
    }
    return wantedState.getDefaultInput().getExtensionInput();
  }

  private double getWantedWristAngle() {
    if ((hasHatch && wantedState.getHatchWristAngle() != null)
        || wantedState.getDefaultWristAngle() == null) {
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

  public void increaseOffset() {
    synchronized (this) {
      if (this.commands.isEmpty()) {
        Double[] state = new Double[]{arm.getArmState().getExtensionInInches(),
            arm.getArmState().getRotationInDegrees()};

        Command command = adjust(state, 1);
        if (command != null) {
          executeCommand(command);
        }
      }
    }
  }

  public void decreaseOffset() {
    synchronized (this) {
      if (this.commands.isEmpty()) {
        Double[] state = new Double[]{arm.getArmState().getExtensionInInches(),
            arm.getArmState().getRotationInDegrees()};

        Command command = adjust(state, -1);
        if (command != null) {
          executeCommand(command);
        }
      }
    }
  }

  private Command adjust(Double[] state, double deltaX) {
    Double[] coords = polarToCartesian(state);
    coords[0] += Math.signum(coords[0]) * deltaX;
    coords = clampCartesian(coords);
    Double[] polar = cartesianToPolar(coords);

    if (reachable(state, polar)) {
      double wristAngle = state[1] + wrist.getEncoderAngle() - polar[1];
      return new Command(polar[1], polar[0], wristAngle, Limit.NONE);
    }
    return null;
  }

  private Double[] polarToCartesian(Double[] polar) {
    double x = Math.sin(polar[1]) * (polar[0] + ARM_BASE_LENGTH);
    double y = Math.cos(polar[1]) * (polar[0] + ARM_BASE_LENGTH);

    return new Double[]{x, y};
  }

  private Double[] cartesianToPolar(Double[] cartesian) {
    double ext =
        Math.sqrt(cartesian[0] * cartesian[0] + cartesian[1] * cartesian[1]) - ARM_BASE_LENGTH;
    double rot = Math.atan2(cartesian[0], cartesian[1]);

    return new Double[]{ext, rot};
  }

  private Double[] clampCartesian(Double[] input) {
    double x = Math.signum(input[0]) * Math
        .min(Math.max(MIN_LATERAL_EXTENSION, Math.abs(input[0])), MAX_LATERAL_EXTENSION);
    return new Double[]{x, input[1]};
  }

  private Boolean reachable(Double[] start, Double[] goal) {
    return ARM_MIN_EXTENSION < goal[0]
        && goal[0] < ARM_MAX_EXTENSION
        && start[1] * goal[1] > 0
        && Math.abs(start[1]) < 90;
  }

  @Override
  public void test() {

  }

  @Override
  public String id() {
    return TAG;
  }


  private enum Limit {
    ROTATION, EXTENSION, WRIST, NONE
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
