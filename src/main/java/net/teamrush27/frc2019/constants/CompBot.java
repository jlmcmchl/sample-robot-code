package net.teamrush27.frc2019.constants;

public class CompBot implements RobotConfiguration {

  //1935
	@Override
	public double getArmHomePosition() {
		return 170.068359375;
	}
	
	@Override
	public Integer getJawIntakePosition() {
		return -470;
	}
	
	@Override
	public Integer getJawExhaustPosition() {
		return -429;
	}
	
	@Override
	public Integer getJawRetractPosition() {
		return -758;
	}
	
	@Override
	public Integer getWristHomePosition() {
		return 1893;
	}
	
	@Override
	public double getArmMaxExtension() {
		return 49d;
	}
	
	@Override
	public double getArmLevel3CargoExtension() {
		return 49d;
	}
	
	@Override
	public double getArmLevel3HatchExtension() {
		return 46d;
	}
	
	@Override
	public double getLimelightDriveForwardPercent() {
		return .3;
	}
	
	@Override
	public double getDriveKv() {
		return 0.12126489927598622;
	}
	
	@Override
	public double getDriveKa() {
		return 0.011954287296849787;
	}
	
	@Override
	public double getDriveVIntercept() {
		return 0.8518630469354613;
	}

	@Override
	public double getScrubFactor() {
		return  1.1709785539493969;
	}
}
