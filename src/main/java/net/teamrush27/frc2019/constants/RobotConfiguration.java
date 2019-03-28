package net.teamrush27.frc2019.constants;

public interface RobotConfiguration {
	
	double getArmHomePosition();
	
	Integer getJawIntakePosition();
	Integer getJawExhaustPosition();
	Integer getJawRetractPosition();
	
	Integer getWristHomePosition();
	
	double getLimelightDriveForwardPercent();
	
	
	double getDriveKv();
	double getDriveKa();
	double getDriveVIntercept();
	
}
