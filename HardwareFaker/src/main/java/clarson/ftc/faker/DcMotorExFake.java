package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorExFake extends DcMotorSimpleFake implements DcMotorEx {
    private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private double currentAlertAmps = 0;
    private int tolerance = 3;
    private int target;
    private boolean enabled = true;
    private boolean busy = false;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    private double runToPower;
    private double runToFactor = 1;

    public DcMotorExFake(double rpm, double ticksPerRev) {
        super(rpm, ticksPerRev);
    }

    public DcMotorExFake(double rpm, double ticksPerRev, double currentPosition) {
        super(rpm, ticksPerRev, currentPosition);
    }

    @Override
    public boolean isOverCurrent() {
        return this.getCurrent(CurrentUnit.AMPS) > currentAlertAmps;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        this.currentAlertAmps = unit.toAmps(current);
    }

    @Override 
    public double getCurrentAlert(CurrentUnit unit) {
        return unit.convert(this.currentAlertAmps, CurrentUnit.AMPS);
    }
    
    @Override 
    public double getCurrent(CurrentUnit unit) {
        // Just pretending that resistance doesn't matter (it doesn't, so touch 
        // exposed wires, kids!) and so current = voltage
        return unit.convert(13 * this.power, CurrentUnit.AMPS);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        if(runMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            this.mode = runMode;
            this.currentPosition = 0;
            return;
        }

        if(!(this.mode == DcMotor.RunMode.RUN_TO_POSITION && runMode == DcMotor.RunMode.RUN_TO_POSITION)) {
            this.runToFactor = 1;
            this.busy = false;
        }

        if(runMode == DcMotor.RunMode.RUN_TO_POSITION) {
            this.busy = true;
        }

        // this.velocity = 0;
        this.mode = runMode;
    }

    @Override
    public DcMotor.RunMode getMode() {
        return this.mode;
    }

    @Override
    public void setTargetPosition(int target) {
        this.target = target;
    }

    @Override
    public int getTargetPosition() {
        return this.target;
    }

    @Override
    public boolean isBusy() {
        return busy;
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        this.zeroPowerBehavior = behavior;
    }

    @Override
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return this.zeroPowerBehavior;
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean getPowerFloat() {
        return this.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT;
    }

    @Override
    public void setPower(double power) {
        if(this.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            this.power = power;
            this.runToPower = power;
            return;
        }

        // Default: Just set the power (duh...)
        this.runToPower = power;
        super.setPower(power);
    }

    @Override
    public void setVelocity(double vel) {
        if(this.mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            this.velocity = Range.clip(vel, -this.maxTickSpeed, this.maxTickSpeed);
        }
    }

    @Override
    public void setVelocity(double vel, AngleUnit unit) {
        if(this.mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            this.velocity = Range.clip(
                unit.toDegrees(vel) / 180 * ticksPerRev, 
                -this.maxTickSpeed, 
                this.maxTickSpeed
            );
        }
    }

    @Override
    public double getVelocity() {
        return this.velocity;
    }
    
    @Override
    public double getVelocity(AngleUnit unit) {
        return unit.fromDegrees(this.velocity / this.ticksPerRev * 180);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of ignoring PIDF constants
        // Not doing anything cause the internal "PID" modes are not PIDs.
        return null;
    }
    
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of ignoring PIDF constants
        // Not doing anything cause the internal "PID" modes are not PIDs.
        return null;
    }
    
    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients constants) {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of ignoring PIDF constants
        // Not doing anything cause the internal "PID" modes are not PIDs.
    }
    
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients constants) {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of ignoring PIDF constants
        // Not doing anything cause the internal "PID" modes are not PIDs.
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of ignoring PIDF constants
        // Not doing anything cause the internal "PID" modes are not PIDs.
    }
    
    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of ignoring PIDF constants
        // Not doing anything cause the internal "PID" modes are not PIDs.
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return this.tolerance;
    }

    @Override
    public int getCurrentPosition() {
        // TODO: Decide whether reversing the motor affects this (hopefully not! 🤞)
        return (int) Math.round(this.currentPosition);
    }
    
    @Override 
    public boolean isMotorEnabled() {
        return this.enabled;
    }

    @Override
    public void setMotorEnable() {
        this.enabled = true;
        this.setPower(0);
        this.velocity = 0;
    }
    
    @Override
    public void setMotorDisable() {
        this.enabled = false;
        this.setPower(0);
        this.velocity = 0;
    }

    @Override
    public int getPortNumber() {
        return -1;
    }

    @Override
    public DcMotorController getController() {
        // ¯\_(ツ)_/¯  I'm not sure how to construct these or if it matters
        return null;
    }

    @Override 
    public void setMotorType(MotorConfigurationType config) {
        // If you relay on this method, there is something seriously wrong with you.
        throw new RuntimeException("Attempted to call setMotorType on DcMotorExFake");
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    private double updateRunToVelocity() {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of this
        // Getting the speed factor
        double reverseFactor  = 1; // Reverse at a lower speed if the target is missed.
        if((this.target - this.currentPosition) / this.velocity < 0 && this.runToFactor != -1) {
            reverseFactor *= -0.5; // Put it in reverse, Ter! ...and put half the previous speed
            // reverseFactor *= -1; // Put it in reverse, Ter! ...and put half the previous speed
        } else if(this.runToFactor == 1) {
            reverseFactor *= -1;
        }

        this.runToFactor *= reverseFactor;

        // Setting the velocity;
        this.velocity = this.runToPower * this.runToFactor * this.maxTickSpeed;
        return reverseFactor;
    }

    /**
     * Gets whether this motor is attempting to brake. This is meant to be used 
     * by a test method, not by an opmode, to apply less rotation than normal,
     * dependent on this motor's torque (something not stored in this class).
     * 
     * @return Whether the motor is attempting to brake.
     */
    public boolean isBraking() {
        return this.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.BRAKE && this.power == 0;
    }

    /**
     * Adds the given rotation to the velocity in the forward direction. The 
     * given velocity is in radians/second. This is designed to allow for unit
     * tests to simulate physics, such as the moment on an arm from gravity.
     * 
     * If the motor is in DcMotor.RunMode.RUN_USING_ENCODER or 
     * DcMotor.RunMode.RUN_TO_POSITION, this only clamps the velocity.
     * 
     * The position is not updated until the `update()` is called, but the added
     * angular velocity persists until the next call to `setPower()` or some 
     * other change to the velocity.
     * 
     * @param thetaPrime Added angular velocity, in radians per second.
     * @return The new velocity, in ticks per second
     */
    public double addAngularVel(double thetaPrime) {
        switch(this.mode) {
            case RUN_USING_ENCODER: 
            case RUN_TO_POSITION:
                final double ticksPrime = thetaPrime / (2 * Math.PI) * ticksPerRev;
                return this.velocity = Range.clip(
                    super.addAngularVel(thetaPrime), 
                    -this.maxTickSpeed + ticksPrime,
                    this.maxTickSpeed + ticksPrime
                ); 

            default:
                return super.addAngularVel(thetaPrime);
        } 
    }

    @Override
    public double update(double deltaSec) {
        final double delta = this.velocity * deltaSec;
        this.currentPosition += delta;

        // Ending run to position when within the target
        if(
            this.mode == DcMotor.RunMode.RUN_TO_POSITION
            && Math.abs(this.currentPosition - this.target) <= this.tolerance
        ) {
            this.busy = false;
            this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Updating the velocity if the postion is off
        if(this.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            this.busy = true;
            this.updateRunToVelocity();
        } 
        return delta;
    }
}
