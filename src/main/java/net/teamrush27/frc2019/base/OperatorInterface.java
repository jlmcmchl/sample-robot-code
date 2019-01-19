package net.teamrush27.frc2019.base;

import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public interface OperatorInterface {
    // DRIVER CONTROLS
    DriveCommand getTankCommand();
    
    // OPERATOR CONTROLS
	ArmInput getArmInput();

}
