package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;

import static com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;

public class DcMotorControllerExFake implements DcMotorControllerEx {
    protected HashMap<Integer, MotorData> motors = new HashMap<>(4, 1.0f);

    /**
     * Attempts to connect the motor to this controller. The connection can fail 
     * if the motor's port (from `getPort()`) is already occupied. 
     * 
     * @param motorData Metadata for the motor to be connected. The `getPort()` 
     * method should return a value in the range 0-3 (inclusive), or the 
     * connection will fail. 
     * @return True if the the connection was succesfully completed. 
     */
    public boolean connect(MotorData motorData) {
        if(
            motors.containsKey(motorData.actuator.getPortNumber())
            || motorData.actuator.getPortNumber() < 0
            || motorData.actuator.getPortNumber() > 3
        ) {
            return false;
        }

        motors.put(motorData.actuator.getPortNumber(), motorData);
        return true;
    }

    /**
     * Gets the motor data at the given port number. If the port is not occupied 
     * or does not exists, the method throws an IllegalArgumentException.
     * 
     * @param port The port number at which the motor was connected.
     * @return The motor data at the given port.
     */
    protected MotorData getData(int port) {
        if(!motors.containsKey(port)) {
            throw new IllegalArgumentException("Attempted to access unconnected port <" + port + ">.");
        }

        return motors.get(port);
    }

    /**
     * Gets the motor at the given port number. If the port is not occupied or 
     * does not exists, the method throws an IllegalArgumentException.
     * 
     * @param port The port number at which the motor was connected.
     * @return The motor at the given port
     */
    protected DcMotorImplEx getMotor(int port) {
        if(!motors.containsKey(port)) {
            throw new IllegalArgumentException("Attempted to access unconnected port <" + port + ">.");
        }

        return motors.get(port).actuator;
    }

    @Override
    public void setMotorEnable(int port) {
        getData(port).isEnabled = true;
    }

    @Override
    public void setMotorDisable(int port) {
        getData(port).isEnabled = true;
    }

    @Override
    public boolean isMotorEnabled(int port) {
        return getData(port).isEnabled;
    }

    @Override
    public void setMotorVelocity(int port, double ticksPerSecond) {
        getData(port).velocity = ticksPerSecond;
    }

    @Override
    public void setMotorVelocity(int port, double velocity, AngleUnit unit) {
        setMotorVelocity(
            port,
            getData(port).ticksPerRev 
                * AngleUnit.DEGREES.fromUnit(unit, velocity) 
                / 360
        );
    }

    @Override
    public double getMotorVelocity(int port) {
        return getData(port).velocity;
    }

    @Override
    public double getMotorVelocity(int port, AngleUnit unit) {
        return unit.fromDegrees(getData(port).velocity);
    }

    @Override
    public void setPIDCoefficients(int port, DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
        if(!getData(port).pidCoefficients.containsKey(mode)) {
            return;
        }

        getData(port).pidCoefficients.put(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(int port, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        if(!getData(port).pidfCoefficients.containsKey(mode)) {
            return;
        }

        getData(port).pidfCoefficients.put(mode, pidfCoefficients);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(int port, DcMotor.RunMode mode) {
        return getData(port).pidCoefficients.get(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(int port, DcMotor.RunMode mode) {
        return getData(port).pidfCoefficients.get(mode);
    }

    @Override
    public void setMotorTargetPosition(int port, int position, int tolerance) {
        getData(port).targetPosition = position;
        getData(port).tolerance = tolerance;
        getData(port).isTargetSet = true;
    }

    @Override
    public double getMotorCurrent(int port, CurrentUnit unit) {
        // TODO: Make this calculate current drawn based off of velocity and added angular?
        // Just pretending that resistance doesn't matter (it doesn't, so touch 
        // exposed wires, kids!) and so current = voltage
        return unit.convert(Math.abs(13 * getData(port).power), CurrentUnit.AMPS);
    }

    @Override
    public double getMotorCurrentAlert(int port, CurrentUnit unit) {
        return unit.convert(getMotor(port).getPower(), CurrentUnit.AMPS);
    }

    @Override
    public void setMotorCurrentAlert(int port, double current, CurrentUnit unit) {
        getData(port).currentAlertAmps = unit.toAmps(current);
    }

    @Override
    public boolean isMotorOverCurrent(int port) {
        return getMotorCurrent(port, CurrentUnit.AMPS) > getData(port).currentAlertAmps;
    }

    @Override
    public void setMotorType(int port, MotorConfigurationType motorType) {
        // This is only useful to DcMotorImplEx, which uses this to determine operational 
        // direction, that is, whether the motor naturally rotates clockwise or counter.
        getData(port).motorType = motorType;
    }
    
    @Override
    public MotorConfigurationType getMotorType(int port) {
        return getData(port).motorType;
    }
    
    @Override
    public void setMotorMode(int port, DcMotor.RunMode mode) {
        getData(port).mode = mode;
    }
    
    @Override
    public DcMotor.RunMode getMotorMode(int port) {
        return getData(port).mode;
    }
    
    @Override
    public void setMotorPower(int port, double power) {
        getData(port).power = power;
    }
    
    @Override
    public double getMotorPower(int port) {
        return getData(port).power;
    }
    
    @Override
    public boolean isBusy(int port) {
        return getData(port).isBusy;
    }
    
    @Override
    public void setMotorZeroPowerBehavior(int port, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        getData(port).behavior = zeroPowerBehavior;
    }
    
    @Override
    public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int port) {
        return getData(port).behavior;
    }
    
    @Override
    public boolean getMotorPowerFloat(int port) {
        return getData(port).behavior == DcMotor.ZeroPowerBehavior.FLOAT;
    }
    
    @Override
    public void setMotorTargetPosition(int port, int position) {
        getData(port).targetPosition = position;
    }
    
    @Override
    public int getMotorTargetPosition(int port) {
        return getData(port).targetPosition;
    }
    
    @Override
    public int getMotorCurrentPosition(int port) {
        return (int) Math.round(getData(port).position);
    }
    
    @Override
    public void resetDeviceConfigurationForOpMode(int port) {
        final MotorData motor = getData(port); // May as well save the keyboard some stress.
        motor.velocity = 0;
        motor.power = 0;
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER; 
        motor.behavior = DcMotor.ZeroPowerBehavior.BRAKE;
        
        motor.tolerance = 5; // Ticks above or below the target, inclusive
        motor.targetPosition = 0; // Ticks
        motor.isEnabled = true;
        motor.isBusy = false;
        motor.isTargetSet = false;

        motor.currentAlertAmps = 0; // Amperes
        motor.motorType = null; // Not used by :HardwareFaker
        motor.pidCoefficients.clear();
        motor.pidCoefficients.put(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients());
        motor.pidCoefficients.put(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients());
        motor.pidfCoefficients.clear();
        motor.pidfCoefficients.put(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients());
        motor.pidfCoefficients.put(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients());
    }

    @Override
    public void close() {
        this.motors = null; // Allow the connections ot be garbage collected
    }

    private String safeGetDeviceName(DcMotorImplEx possiblyNull) {
        return possiblyNull == null ? "null" : possiblyNull.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return String.format(
              "DcMotorControllerExFake Connections:" +
            "\n    [1]: %s" +  
            "\n    [2]: %s" +  
            "\n    [3]: %s" +  
            "\n    [4]: %s",
            safeGetDeviceName(getMotor(0)),
            safeGetDeviceName(getMotor(1)),
            safeGetDeviceName(getMotor(2)),
            safeGetDeviceName(getMotor(3))
        );
    }

    @Override
    public String getDeviceName() {
        return "DcMotorControllerExFake. Hi youtube!";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.motors = new HashMap<>();
    }
}