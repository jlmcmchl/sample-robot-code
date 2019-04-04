package net.teamrush27.frc2019.constants;

public class PracticeBot implements RobotConfiguration {
	
	@Override
	public double getArmHomePosition() {
		return 213.57;
	}
	
	@Override
	public Integer getJawIntakePosition() {
		return -737;
	}
	
	@Override
	public Integer getJawExhaustPosition() {
		return -696;
	}
	
	@Override
	public Integer getJawRetractPosition() {
		return -1000;
	}
	
	@Override
	public Integer getWristHomePosition() {
		return 1550;
	}
	
	@Override
	public double getLimelightDriveForwardPercent() {
		return .35;
	}
	
	@Override
	public double getDriveKv() {
		return 0.08194457874358892;
	}
	
	@Override
	public double getDriveKa() {
		return 0.02430825757642667;
	}
	
	@Override
	public double getDriveVIntercept() {
		return 1.9806873036972523;
	}
}
