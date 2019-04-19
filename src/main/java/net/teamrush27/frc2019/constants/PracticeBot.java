package net.teamrush27.frc2019.constants;

public class PracticeBot implements RobotConfiguration {
	
	@Override
	public double getArmHomePosition() {
		return 212.6953125;
	}
	
	@Override
	public Integer getJawIntakePosition() {
		return -70;
	}
	
	@Override
	public Integer getJawExhaustPosition() {
		return -29;
	}
	
	@Override
	public Integer getJawRetractPosition() {
		return -360;
	}
	
	@Override
	public Integer getWristHomePosition() {
		return 2170;
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
		return .40;
	}
	
	@Override
	public double getDriveKv() {
		return 0.20315821635941475;
	}
	
	@Override
	public double getDriveKa() {
		return 0.036565738498063036;
	}
	
	@Override
	public double getDriveVIntercept() {
		return 1.0191132753224497;
	}

	@Override
	public double getScrubFactor() {
		return 1.1525606838366464;
	}

}
