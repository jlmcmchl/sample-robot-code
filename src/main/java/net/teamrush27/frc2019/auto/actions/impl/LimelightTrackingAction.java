package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.Limelights;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.math.CircularBuffer;

public class LimelightTrackingAction implements Action {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Limelights limelights = Limelights.getInstance();
 
  private Boolean isFront;
  private Double minLimit;
  
  private CircularBuffer buf = new CircularBuffer(10);
  
  public LimelightTrackingAction(boolean isFront, double minLimit) {
    this.isFront = isFront;
    this.minLimit = minLimit;
  }
  
  @Override
  public boolean isFinished() {
    if(buf.isFull() && Math.abs(buf.getMax() - buf.getMin()) < 50 && buf.getAverage() > minLimit){
      return true;
    }
    
    return false;
  }
  
  @Override
  public void update() {
    if(isFront){
      buf.addValue(drivetrain.getFrontDistance());
    } else {
      buf.addValue(drivetrain.getRearDistance());
    }
  }

  @Override
  public void done() {
    limelights.setTrackingEnabled(false);
  }

  @Override
  public void start() {
    limelights.setTrackingEnabled(true);
    drivetrain.setLimelightSteering(limelights.getSystemState());
  }
}
