package net.teamrush27.frc2019.constants;

public class CompBot implements RobotConfiguration {
	
	@Override
	public double getArmHomePosition() {
		return 170.5;
	}
	
	@Override
	public Integer getJawIntakePosition() {
		return -455;
	}
	
	@Override
	public Integer getJawExhaustPosition() {
		return -415;
	}
	
	@Override
	public Integer getJawRetractPosition() {
		return -715;
	}
	
	@Override
	public Integer getWristHomePosition() {
		return 2122;
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
}
