package net.teamrush27.frc2019.constants;

public class CompBot implements RobotConfiguration {
	
	@Override
	public double getArmHomePosition() {
		return 170.5;
	}
	
	@Override
	public Integer getJawIntakePosition() {
		return -614;
	}
	
	@Override
	public Integer getJawExhaustPosition() {
		return -565;
	}
	
	@Override
	public Integer getJawRetractPosition() {
		return -902;
	}
	
	@Override
	public Integer getWristHomePosition() {
		return 2538;
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
