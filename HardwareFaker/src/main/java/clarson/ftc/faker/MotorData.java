package clarson.ftc.faker;

// import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MotorData {
    // Descriptor fields - These describe unchaning properties of the motor itself. 
    public final DcMotorImplEx actuator;
    public final double ticksPerRev;
    public final double rpm; // Can be negative

    // Basic Fields - These are the most essential parts to making the update logic work.
    public double position = 0; // Ticks
    public double velocity = 0; // Ticks / sec
    public double power = 0;
    public double accumulatedVelOffset = 0; // Ticks / sec. How much is added by addAngularVel.
    public DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER; 
    public DcMotor.ZeroPowerBehavior behavior = DcMotor.ZeroPowerBehavior.BRAKE;

    // RUN_TO_POSITION Fields. 
    public int tolerance = 5; // Ticks above or below the target, inclusive
    public int targetPosition = 0; // Ticks
    public boolean isEnabled = true;
    public boolean isBusy = false;
    public boolean isTargetSet = false;
    
    // Compatability Fields -  These are (currently) unused by the update logic
    public double currentAlertAmps = 0; // Amperes
    public final HashMap<DcMotor.RunMode, PIDCoefficients> pidCoefficients = new HashMap<>();
    public final HashMap<DcMotor.RunMode, PIDFCoefficients> pidfCoefficients = new HashMap<>();
    public MotorConfigurationType motorType = MotorConfigurationType.getUnspecifiedMotorType();

    public MotorData(DcMotorImplEx actuator, double rpm, double ticksPerRev) {
        this(actuator, rpm, ticksPerRev, 0);
    } 
    
    public MotorData(DcMotorImplEx actuator, double rpm, double ticksPerRev, double initialPosition) {
        // TODO: This should register the motor so that it can be updated and affected

        this.actuator = actuator;
        this.rpm = rpm;
        this.ticksPerRev = ticksPerRev;
        this.position = initialPosition;

        pidCoefficients.put(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients());
        pidCoefficients.put(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients());

        pidfCoefficients.put(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients());
        pidfCoefficients.put(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients());
    } 

    public double getAngle(AngleUnit unit) {
        return unit.fromDegrees(position * 360.0 / ticksPerRev);
    }

    public double getVelocity(AngleUnit unit) {
        return unit.fromDegrees(velocity * 360.0 / ticksPerRev);
    }
}