package net.teamrush27.frc2019.managers;

import java.util.concurrent.ConcurrentLinkedQueue;
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
	private static final double ROTATION_EPSILON = 0.5d;
	private static final double EXTENSION_EPSILON = 0.25d;
	private static final double STINGER_INTERSECTION_ANGLE = -52.5d;
	private static final double ARM_BASE_LENGTH = 0d;
	private static final double ARM_MAX_EXTENSION = 0d;
	private static final double MAX_LATERAL_EXTENSION = 32d;
	
	private static final Logger LOG = LogManager.getLogger(SuperstructureManager.class);
	
	private static final double NUM_SEGMENTS = 200;
	
	public static SuperstructureManager getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new SuperstructureManager();
		}
		
		return INSTANCE;
	}
	
	private Arm arm = Arm.getInstance();
	private Wrist wrist = Wrist.getInstance();
	
	public enum WantedState {
		CARGO_GROUND_PICKUP(new ArmInput(5d, 88d)),
		HATCH_HUMAN_PICKUP(null, new ArmInput(5d, 86d)),
		CARGO_SHIP(new ArmInput(12.5d, 27d)),
		ROCKET_LEVEL_1(new ArmInput(5d, 81d), new ArmInput(5d, 86d)),
		ROCKET_LEVEL_2(new ArmInput(22d, 6d), new ArmInput(22d, 6d)),
		ROCKET_LEVEL_3(new ArmInput(45d, 6d), new ArmInput(45d, 6d)),
		STOW(new ArmInput(5d, 0d)),
		CLIMB(new ArmInput(5d, -45d));
		
		private final ArmInput defaultInput;
		private final ArmInput hatchInput;
		
		WantedState(ArmInput armInput) {
			this.defaultInput = armInput;
			this.hatchInput = null;
		}
		
		WantedState(ArmInput armInput, ArmInput hatchInput) {
			this.defaultInput = armInput;
			this.hatchInput = hatchInput;
		}
		
		
		public ArmInput getDefaultInput() {
			return defaultInput;
		}
		
		public ArmInput getHatchInput() {
			return hatchInput;
		}
	}
	
	private WantedState wantedState = WantedState.STOW;
	private boolean wantedStateChanged = false;
	private boolean invertedRotation = false;
	private ConcurrentLinkedQueue<Command> commands = new ConcurrentLinkedQueue<>();
	
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
	
	private void handlePosition(double timestamp) {
		if (wantedStateChanged) {
			recomputeOperations();
		}
		
		if (commands.isEmpty()) {
			return;
		}
		
		if (operationComplete(commands.peek())) {
			commands.poll();
		}
		
		if (commands.isEmpty()) {
			return;
		}
		
		executeCommand(commands.peek());
	}
	
	// TODO
	// I don't like how this handles arm extension while at 30" out.
	// If we're moving from one level to another while at 30", we risk violating our extension limit
	// if extension and rotation don't move in sync.
	// Possible solution to put setpoints at 25" out instead
	// and creep towards like we had kyle move the elevator last year
	
	
	private void recomputeOperations() {
		ArmState armState = arm.getArmState();
		InterpolatingDouble initialRotation = new InterpolatingDouble(
			armState.getRotationDemandInDegrees());
		InterpolatingDouble wantedRotation = new InterpolatingDouble(getWantedRotation());
		InterpolatingDouble initialExtension = new InterpolatingDouble(
			armState.getExtensionInInches());
		InterpolatingDouble wantedExtension = new InterpolatingDouble(wantedState.getExtension());
		
		// needs to travel towards rear of robot
		
		boolean extension_bound = false;
		
		for (int i = 0; i < NUM_SEGMENTS; i++) {
			InterpolatingDouble angle = initialRotation
				.interpolate(wantedRotation, i * 1.0 / NUM_SEGMENTS);
			
			InterpolatingDouble extension = initialExtension
				.interpolate(wantedExtension, i * 1.0 / NUM_SEGMENTS);
			
			double bound_extension = boundExtensionForAngle(angle.value, extension.value);
			
			if (bound_extension != extension.value) {
				commands.add(new Command(bound_extension, angle.value, Limit.EXTENSION));
				extension_bound = true;
			} else if (extension_bound) {
				commands.add(new Command(extension.value, angle.value, Limit.ROTATION));
			}
		}
		
		commands.add(new Command(wantedExtension.value, wantedRotation.value, Limit.NONE));
	}
	
	private boolean operationComplete(Command current) {
		boolean complete = true;
		
		if (current.getLimitType() == Limit.ROTATION || current.getLimitType() == Limit.NONE) {
			complete =
				complete && MathUtils.epsilonEquals(arm.getArmState().getRotationDemandInDegrees(),
					current.getRotationDemand(), ROTATION_EPSILON);
		}
		
		if (current.getLimitType() == Limit.EXTENSION || current.getLimitType() == Limit.NONE) {
			complete = complete && MathUtils.epsilonEquals(arm.getArmState().getExtensionInInches(),
				current.getExtensionDemand(), EXTENSION_EPSILON);
		}
		
		return complete;
	}
	
	private void executeCommand(Command command) {
//    ArmState state = arm.getArmState();
//    state.setRotationDemandInDegrees(command.getRotationDemand());
//    state.setExtensionDemand(command.getExtensionDemand());
	}
	
	private double getWantedRotation() {
		return wantedState.getRotation() * (invertedRotation ? -1 : 1);
	}
	
	private double boundExtensionForAngle(final double angle, final double extension) {
		final double flooredExtension = Math.min(extension, getMaxExtensionForAngle(angle));
		final double ceiledExtension = Math.max(flooredExtension, getMinExtensionForAngle(angle));
		return ceiledExtension;
	}
	
	private double getMaxExtensionForAngle(double angle) {
		double extension =
			MAX_LATERAL_EXTENSION / Math.cos(angle * Math.PI / 180) - ARM_BASE_LENGTH;
		return Math.min(ARM_MAX_EXTENSION, extension);
	}
	
	private double getMinExtensionForAngle(double angle) {
		if (MathUtils.epsilonEquals(angle, STINGER_INTERSECTION_ANGLE, 5)) {
			return 5;
		}
		return 0d;
	}
	
	@Override
	public void test() {
	
	}
	
	@Override
	public String id() {
		return TAG;
	}
	
	
	private enum Limit {
		ROTATION, EXTENSION, NONE
	}
	
	private class Command {
		
		private final double extensionDemand;
		private final double rotationDemand;
		private final Limit limitType;
		
		protected Command(double extensionDemand, double rotationDemand, Limit limitType) {
			this.extensionDemand = extensionDemand;
			this.rotationDemand = rotationDemand;
			this.limitType = limitType;
		}
		
		public double getExtensionDemand() {
			return extensionDemand;
		}
		
		public double getRotationDemand() {
			return rotationDemand;
		}
		
		public Limit getLimitType() {
			return limitType;
		}
		
	}
}
