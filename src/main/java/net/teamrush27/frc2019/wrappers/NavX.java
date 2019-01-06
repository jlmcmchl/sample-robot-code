package net.teamrush27.frc2019.wrappers;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.SPI;
import net.teamrush27.frc2019.util.math.Rotation2d;

/**
 * @author team254
 * Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
 */
public class NavX {
    protected class Callback implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(long systemTimestamp, long sensorTimestamp, 
        		AHRSUpdateBase update, Object context) {
            synchronized (NavX.this) {
                // This handles the fact that the sensor is inverted from our coordinate conventions.
                if (lastSensorTimestampMs != INVALID_TIMESTAMP 
                		&& lastSensorTimestampMs < sensorTimestamp) {
                    yawRate = 1000.0 * (-yawDegrees - update.yaw)
                            / (double) (sensorTimestamp - lastSensorTimestampMs);
                }
                lastSensorTimestampMs = sensorTimestamp;
                yawDegrees = -update.yaw;
            }
        }
    }

    protected AHRS ahrs;

    protected Rotation2d angleAdjustment = Rotation2d.identity();
    protected double yawDegrees;
    protected double yawRate; // in Degrees per Second
    protected final static long INVALID_TIMESTAMP = -1;
    protected long lastSensorTimestampMs;

    public NavX(SPI.Port spiPortId) {
        ahrs = new AHRS(spiPortId, (byte) 200);
        resetState();
        ahrs.registerCallback(new Callback(), null);
    }

    public synchronized void reset() {
        ahrs.reset();
        resetState();
    }

    public synchronized void zeroYaw() {
        ahrs.zeroYaw();
        resetState();
    }

    private void resetState() {
        lastSensorTimestampMs = INVALID_TIMESTAMP;
        yawDegrees = 0.0;
        yawRate = 0.0;
    }

    public synchronized void setAngleAdjustment(Rotation2d adjustment) {
        angleAdjustment = adjustment;
    }

    public synchronized double getRawYawDegrees() {
        return yawDegrees;
    }

    public Rotation2d getYaw() {
        return angleAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
    }

    /**
     * @return the yaw rate in degrees per second
     */
    public double getYawRate() {
        return yawRate;
    }

    /**
     * @return the yaw rate in radians per second
     */
    public double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRate();
    }

    public double getRawAccelX() {
        return ahrs.getRawAccelX();
    }
    
    public double getRawAccelY(){
    	return ahrs.getRawAccelY();
	}
	
	public double getPitch(){
    	return ahrs.getPitch();
	}
}

