package clarson.ftc.faker;

import static com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;

public class CRServoImplExFake extends CRServoImplEx implements Updateable {
    private static ServoControllerEx fakeController = new ServoControllerEx() {
        public void pwmEnable() {}
        public void pwmDisable() {}
        public ServoController.PwmStatus getPwmStatus() { return null; }
        public void setServoPosition(int servo, double position) {}
        public double getServoPosition(int servo) { return 0.0; }
        public void setServoPwmRange(int servo, PwmControl.PwmRange range) {}
        public PwmControl.PwmRange getServoPwmRange(int servo) { return null; }
        public void setServoPwmEnable(int servo) {}
        public void setServoPwmDisable(int servo) {}
        public boolean isServoPwmEnabled(int servo) { return false; }
        public void setServoType(int servo, ServoConfigurationType servoType) {}
        public void close() {}
        public String getConnectionInfo() { return null; }
        public String getDeviceName() { return null; }
        public Manufacturer getManufacturer() { return null; }
        public int getVersion() { return 0; }
        public void resetDeviceConfigurationForOpMode() {}
        
    };

    protected double power = 0;
    protected double maxTickSpeed; // Revolutions per second
    protected double velocity = 0; // Revolutions per second
    protected double accumulatedVelOffset = 0; // Revolutions per second
    protected double currentPosition = 0; // In revolutions
    protected DcMotor.Direction direction = DcMotor.Direction.FORWARD;
    protected PwmRange pwmRange = PwmRange.defaultRange; // 600μs - 2400μs, according to Game Manual 0.
    protected boolean pwmEnabled = true;
    protected PwmRange maxPwmRange = new PwmRange(500, 2500); // For REV and GoBilda Servos

    public CRServoImplExFake(double rpm) {
        super(CRServoImplExFake.fakeController, -1, DcMotor.Direction.FORWARD, null);
        this.maxTickSpeed = rpm / 60;
    }

    public CRServoImplExFake(double rpm, double position) {
        super(CRServoImplExFake.fakeController, -1, DcMotor.Direction.FORWARD, null);
        this.maxTickSpeed = rpm / 60;
        this.currentPosition = position;
    }

    public CRServoImplExFake(double rpm, double position, PwmRange maxPwmRange) {
        super(CRServoImplExFake.fakeController, -1, DcMotor.Direction.FORWARD, null);
        this.maxTickSpeed = rpm / 60;
        this.currentPosition = position;
        this.maxPwmRange = maxPwmRange; // Values in μs 
    }

    @Override
    public void setPower(double power) {
        if(!pwmEnabled) {
            return;
        }

        this.power = power;
        this.velocity = Range.scale( // NOTE: PWM Frame width is not respected.
            // Original value
            Range.clip(power, -1.0, 1.0) * this.maxTickSpeed,

            // Original range
            -this.maxTickSpeed, 
            this.maxTickSpeed, 

            // New Range
            this.getLowerSpeed(), 
            this.getUpperSpeed()
        );
        this.velocity = Range.clip(this.velocity, this.getLowerSpeed(), this.getUpperSpeed());
        this.accumulatedVelOffset = 0;

        if(this.direction == DcMotor.Direction.REVERSE) {
            this.velocity = -this.velocity;
        }
    }

    @Override
    public double getPower() { 
        return power;
    }

    @Override
    public void setPwmRange(PwmRange pwmRange) {
        this.pwmRange = pwmRange;
    }

    @Override
    public PwmRange getPwmRange() {
        return pwmRange;
    }

    @Override
    public void setPwmEnable() {
        pwmEnabled = true;
    }

    @Override
    public void setPwmDisable() {
        pwmEnabled = false;
        this.power = 0;
        this.velocity = 0;
    }

    @Override
    public boolean isPwmEnabled() {
        return pwmEnabled;
    }

    @Override
    public void setDirection(DcMotor.Direction direction) {
        this.direction = direction;
    }

    @Override
    public DcMotor.Direction getDirection() {
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
        return "CRServo Test Fake";
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.direction = DcMotor.Direction.FORWARD;
        this.setPower(0);
    }

    @Override
    public void close() {
        // Nothing to do...
    }

    private double getPwmPower(double pwmMicros) {
        final double midpoint = 0.5 * (maxPwmRange.usPulseLower + maxPwmRange.usPulseUpper);
        final double length = maxPwmRange.usPulseUpper - maxPwmRange.usPulseLower;
        return 2 * (pwmMicros - midpoint) / length;
    }

    private double getUpperSpeed() {
        return maxTickSpeed * getPwmPower(pwmRange.usPulseUpper);
    }
    
    private double getLowerSpeed() {
        return maxTickSpeed * getPwmPower(pwmRange.usPulseLower);
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
     * NOTE: If the power is at 0, the motor will brake, preventing angular 
     * adjustment unless either PWM is disabled or the given velocity offset 
     * overpowers the motor (i.e. is greater than the velocity of setPower(1.0))
     * 
     * @param thetaPrime New angular velocity, in ticks per second.
     * @return The new velocity, in ticks per second
     */
    public double addAngularVel(double thetaPrime) {
        this.accumulatedVelOffset += thetaPrime;
        if(this.power == 0 && this.pwmEnabled) {
            return this.velocity = Range.clip(
                0,
                this.getLowerSpeed() + accumulatedVelOffset / (2 * Math.PI),
                this.getUpperSpeed() + accumulatedVelOffset / (2 * Math.PI)
            );
        }

        return this.velocity += thetaPrime / (2 * Math.PI);
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