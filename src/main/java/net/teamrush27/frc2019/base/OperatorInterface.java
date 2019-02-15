package net.teamrush27.frc2019.base;

import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public interface OperatorInterface {
    // DRIVER CONTROLS
    DriveCommand getTankCommand();
    boolean shouldShiftLowGear();
    boolean shouldShiftHighGear();
    
    // OPERATOR CONTROLS
	
	// gripper
	Boolean getWantManipulateHatch();
	Boolean getWantManipulateCargo();
	
	// arm
	Boolean wantsStow();
	Boolean wantsGroundPickup();
	Boolean wantsLevel1HumanLoad();
	Boolean wantsLevel2();
	Boolean wantsLevel3();
	Boolean getWantsInvert();
	
	// climb
	Boolean wantsPreClimb();
	Boolean wantsClimb();

	// open loop
	Double getWristInput();
	ArmInput getArmInput();
	
}
