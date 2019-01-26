package net.teamrush27.frc2019.auto.modes;

import java.util.ArrayList;
import java.util.List;
import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.CollectAccelerationData;
import net.teamrush27.frc2019.auto.actions.CollectVelocityData;
import net.teamrush27.frc2019.auto.actions.WaitAction;
import net.teamrush27.frc2019.util.physics.DriveCharacterization;
import net.teamrush27.frc2019.util.physics.DriveCharacterization.VelocityDataPoint;

public class CharacterizeDrivetrain extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    List<VelocityDataPoint> velocityData = new ArrayList<>();
    List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

    // runAction(new ShiftHighGearAction(false));
    // runAction(new WaitAction(10));

    runAction(new CollectVelocityData(velocityData, false, false));
    runAction(new WaitAction(10));
    runAction(new CollectAccelerationData(accelerationData, false, false));

    DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

    System.out.println("ks: " + constants.ks);
    System.out.println("kv: " + constants.kv);
    System.out.println("ka: " + constants.ka);
  }
}