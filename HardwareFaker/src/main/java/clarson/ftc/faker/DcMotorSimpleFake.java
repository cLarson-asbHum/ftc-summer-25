package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.Range;

public class DcMotorSimpleFake implements DcMotorSimple {
    protected double power = 0;
    protected double maxTickSpeed;
    protected double ticksPerRev;
    protected double velocity = 0; // Ticks per second
    protected double currentPosition = 0;
    protected DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    public DcMotorSimpleFake(double rpm, double ticksPerRev) {
        this.ticksPerRev = ticksPerRev;
        this.maxTickSpeed = rpm / 60 * this.ticksPerRev;
    }

    public DcMotorSimpleFake(double rpm, double ticksPerRev, double position) {
        this.ticksPerRev = ticksPerRev;
        this.maxTickSpeed = rpm / 60 * this.ticksPerRev;
        this.currentPosition = position;
    }

    @Override
    public void setPower(double power) {
        this.power = power;
        this.velocity = Range.clip(power, -1.0, 1.0) * this.maxTickSpeed;

        if(this.direction == DcMotorSimple.Direction.REVERSE) {
            this.velocity = -this.velocity;
        }
    }

    @Override
    public double getPower() { 
        return power;
    }

    @Override
    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }

    @Override
    public DcMotorSimple.Direction getDirection() {
        return this.direction;
    }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.Unknown;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override 
    public String getConnectionInfo() {
        return "DcMotor Test Fake - No connection!";
    }

    @Override
    public String getDeviceName() {
        return "DcMotor Test Fake";
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.direction = DcMotorSimple.Direction.FORWARD;
    }

    @Override
    public void close() {
        // Nothing to do...
    }

    /**
     * Adds the given rotation to the velocity in the forward direction. The 
     * given velocity is in radians/second. This is designed to allow for unit
     * tests to simulate physics, such as the moment on an arm from gravity.
     * 
     * The position is not updated until the `update()` is called, but the added
     * angular velocity persists until the next call to `setPower()` or some 
     * other change to the velocity.
     * 
     * @param thetaPrime Added angular velocity, in radians per second.
     * @return The new velocity, in ticks per second
     */
    public double addAngularVel(double thetaPrime) {
        return this.velocity = thetaPrime / (2 * Math.PI) * this.ticksPerRev;
    }

    /**
     * Performs All calculations necessary for correct operation of this fake. 
     * This should be called by the test itself to simulate real life for an 
     * opmode, not by opmodes themselves. 
     * 
     * @param deltaSec Elapsed number of seconds since the last call to update()
     * @return The changed number of ticks.
     */
    public double update(double deltaSec) {
        final double deltaTick = this.velocity * deltaSec;
        this.currentPosition += deltaTick;
        return deltaTick;
    }
}