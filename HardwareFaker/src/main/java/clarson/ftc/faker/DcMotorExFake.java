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
    private boolean targetSet = false; // Only used to valid that a target has been set before RUN_TO_POSITION
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
        return unit.convert(Math.abs(13 * this.power), CurrentUnit.AMPS);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        if(runMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            this.mode = runMode;
            this.currentPosition = 0;
            return;
        }

        if(runMode == DcMotor.RunMode.RUN_TO_POSITION && !targetSet) {
            // If no target has been set, throw an exception
            throw new RuntimeException("No target set before setting runMode to RUN_TO_POSITION");
        }
        
        if(this.mode != DcMotor.RunMode.RUN_TO_POSITION || runMode != DcMotor.RunMode.RUN_TO_POSITION) {
            // Cleanup for when exiting RUN_TO_POSITION. Going to and from RUN_TO_POSITIOM doesn't count.
            // NOTE: This also executes when going to RUN_TO_POSITION from a non-RUN_TO mode. 
            this.runToFactor = 1;
            this.busy = false;
            this.targetSet = false;
        }

        if(runMode == DcMotor.RunMode.RUN_TO_POSITION) {
            this.busy = true;
            this.velocity = 0;
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
        this.targetSet = true;
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

    @Override
    public boolean getPowerFloat() {
        return this.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT;
    }

    @Override
    public void setPower(double power) {
        if(!this.enabled) {
            return;
        }

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
        if(!this.enabled) {
            return;
        }

        if(this.mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            this.velocity = Range.clip(vel, -this.maxTickSpeed, this.maxTickSpeed);
        }
    }

    @Override
    public void setVelocity(double vel, AngleUnit unit) {
        setVelocity(unit.fromDegrees(vel / this.ticksPerRev * 360));
    }

    @Override
    public double getVelocity() {
        if(this.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            return 0;
        }

        return this.velocity;
    }
    
    @Override
    public double getVelocity(AngleUnit unit) {
        return unit.fromDegrees(this.velocity / this.ticksPerRev * 360);
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

    @Override
    public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        this.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setDirection(DcMotor.Direction.FORWARD);
        this.setPower(0);
    }

    private double updateRunToVelocity() {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of this
        // Getting the speed factor
        double reverseFactor  = 1; // Reverse at a lower speed if the target is missed.
        if((this.target - this.currentPosition) / this.velocity < 0 && this.runToFactor == 1) {
            // Reversing the power so wer alsways start in the correct direction
            reverseFactor *= -1;
        } else if((this.target - this.currentPosition) / this.velocity < 0) {
            reverseFactor *= -0.5; // Put it in reverse, Ter! ...and put half the previous speed
            // reverseFactor *= -1; // Put it in reverse, Ter! ...and put half the previous speed
        }

        this.runToFactor *= reverseFactor;

        // Setting the velocity;
        if(this.getDirection() == DcMotor.Direction.REVERSE) {
            System.out.println("[updateRunVelocity] maxTickSpeed: " + this.maxTickSpeed);
        }
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
     * DcMotor.RunMode.RUN_TO_POSITION, this only clamps the velocity. The new 
     * range is [tickPrime - |maxTickSpeed|,  tickPrime + |maxTickSpeed|], where
     * tickPrime is thetaPrime converted to ticks per second. This simulates the 
     * effect of the motor being overpowered and, therefore, not holding 
     * velocity. 
     * 
     * To illustrate the previous point, take for example setting a motor to 
     * power 1.0 sets the velocity to 100 ticks per sec. A load on the motor, 
     * however, is calculated to apply -20 ticks per sec to the motor, so the 
     * velocity of the motor goes to 80 ticks per sec. However, if the motor was 
     * originally set to 0.3 power andthe velocity to 30 ticks per second, the 
     * motor would stay at 30 ticks per second, because the motor can apply more
     * torque to overcome the counter-torque of the load.
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
                // Only if we are enabled do we actually keep velocity; otherwise, we go into the 
                // default block, which causes the motor vel to change.
                if(this.enabled) {
                    final double ticksPrime = thetaPrime / (2 * Math.PI) * ticksPerRev;
                    return this.velocity = Range.clip(
                        this.velocity, 
                        -this.maxTickSpeed + ticksPrime,
                        this.maxTickSpeed + ticksPrime
                    ); 
                }

                // No break statement. We only break in the if statement above

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
            this.targetSet = false;
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
