package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import clarson.ftc.faker.updater.Rotateable;
import clarson.ftc.faker.updater.TwoWayUpdateable;
import clarson.ftc.faker.updater.Updater;

import java.util.HashSet;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class DcMotorImplExFake extends DcMotorImplEx implements Rotateable, TwoWayUpdateable {
    public final static MotorConfigurationType getFakeConfiguration(MotorData data) {
        final MotorConfigurationType result = new MotorConfigurationType();
        result.setTicksPerRev(data.ticksPerRev);
        result.setMaxRPM(data.maxTicksPerSec / data.ticksPerRev);
        result.setGearing(1); // TODO: Make gearing and torque a part of MotorData?
        result.setAchieveableMaxRPMFraction(1); // NOTE: waat? I don't know what to do with this property...
        result.setOrientation(Rotation.CW); // NOTE: This says CCW; however, position can be interpreted however
        return result;
    }
    
    public final static MotorConfigurationType getFakeConfiguration(double rpm, double ticksPerRev) {
        final MotorConfigurationType result = new MotorConfigurationType();
        result.setTicksPerRev(ticksPerRev);
        result.setMaxRPM(rpm);
        result.setGearing(1); // TODO: Make gearing and torque a part of MotorData?
        result.setAchieveableMaxRPMFraction(1); // NOTE: waat? I don't know what to do with this property...
        result.setOrientation(Rotation.CW); // NOTE: This says CCW; however, position can be interpreted however
        return result;
    }

    /**
     * Finds the lowest valued, unnoccupied port on the controller. If none are
     * found, -1 is returned.
     * 
     * @return The lowest avaiable port, or -1 if none exists.
     */
    private static int findAvaiablePort(DcMotorControllerExFake controller) {
        for(int i = 0; i < 4; i++) {
            if(controller.isPortAvailable(i)) {
                return i;
            }
        }

        // The method would've early returned if any was avaiable
        return -1;
    }

    private Set<Updater> updaters = new HashSet<>();

    /**
     * Constructs a new DcMotorImplExFake, isolated from all other hardware. 
     * This uses a newly construced DcMotorControllerExFake, meaning no other
     * hardware is present on the controller. Therefore, this is best used to 
     * do unit testing rather than offline end-to-end testing.
     * 
     * The motor will start at position 0.
     * 
     * @param rpm - Maximum speed the motor can turn, in reovlutions per second.
     * Negative values will rotate the opposite direction. The default direction
     * is counter clockwise.
     * @param ticksPerRev Number of ticks per revolution. These are the units 
     * returned by `getCurrentPosition()`.
     */
    public DcMotorImplExFake(double rpm, double ticksPerRev) {
        this(rpm, ticksPerRev, 0);
    }

    /**
     * Constructs a new DcMotorImplExFake, isolated from all other hardware. 
     * This uses a newly construced DcMotorControllerExFake, meaning no other
     * hardware is present on the controller. Therefore, this is best used to 
     * do unit testing rather than offline end-to-end testing.
     * 
     * @param rpm - Maximum speed the motor can turn, in reovlutions per second.
     * Negative values will rotate the opposite direction. The default direction
     * is counter clockwise.
     * @param ticksPerRev Number of ticks per revolution. These are the units 
     * returned by `getCurrentPosition()`.
     * @param initialPosition The position the motor will start at, in ticks. If
     * no movement has occurred, this is the first value that will be returned by 
     * `getCurrentPosition()`
     */
    public DcMotorImplExFake(double rpm, double ticksPerRev, double initialPosition) {
        super(
            DcMotorControllerExFake.createPossiblyWithNullModule(), 
            0, 
            DcMotor.Direction.FORWARD, 
            getFakeConfiguration(rpm, ticksPerRev)
        );

        final MotorData newData = new MotorData(this, rpm, ticksPerRev, initialPosition);
        ((DcMotorControllerExFake) this.getController()).connect(newData);
        this.getController().setMotorType(0, getFakeConfiguration(newData));
    }

    /**
     * Constructs a new `DcMotorImplExFake` connected to the given controller. 
     * The motor is connected to the controller at the given port during 
     * construction. If connection fails because the port is already occupied or 
     * is out of range (ports must be between 0-3 inclusive), an 
     * IllegalArumentException is thrown.
     * 
     * The motor referenced by the given `MotorData's` actuator field is 
     * ignored. While the value can be anything, it is recommended to be null
     * for unambiguity.
     * 
     * @param data Properties, including RPM and ticks per rev, that describe 
     * the motor and its current state.
     * @param controller What drives the motor. 
     * @param portNumber Where to connect the motor to on the controller. No 
     * advantage is given to any specific port(s).
     */
    public DcMotorImplExFake(MotorData data, DcMotorControllerExFake controller, int portNumber) {
        super(controller, portNumber, DcMotor.Direction.FORWARD, getFakeConfiguration(data));

        if(!controller.connect(MotorData.copyForMotor(this, data))) {
            throw new IllegalArgumentException("Port number <" + portNumber + "> is not available on controller");
        }

        controller.setMotorType(portNumber, getFakeConfiguration(data));
    }

    /**
     * Constructs a new `DcMotorImplExFake` connected to the given controller. 
     * The motor is connected to the controller at the lowest available port.
     * If no ports are avaiable, an IllegalArgumentException is thrown.
     * 
     * The motor referenced by the given `MotorData's` actuator field is 
     * ignored. While the value can be anything, it is recommended to be null
     * for unambiguity.
     * 
     * @param data Properties, including RPM and ticks per rev, that describe 
     * the motor and its current state.
     * @param controller What drives the motor. 
     */
    public DcMotorImplExFake(MotorData data, DcMotorControllerExFake controller) {
        this(data, controller, findAvaiablePort(controller));
        // super(
        //     controller, 
        // findAvaiablePort(controller), 
        // DcMotor.Direction.FORWARD, 
        // getFakeConfiguration(data));

        // if(!controller.connect(MotorData.copyForMotor(this, data))) {
        //     throw new IllegalArgumentException("Port number <" + getPortNumber() + "> is not available on controller");
        // }

        // controller.setMotorType(getPortNumber(), getFakeConfiguration(data));
    }

    @Override
    public double update(double deltaSec) {
        return this.getData().update(deltaSec);
    }

    @Override
    public double addAngularVelOffset(double thetaPrime) {
        return this.getData().addAngularVelOffset(thetaPrime);
    }
    
    @Override
    public double setAngularVelOffset(double thetaPrime) {
        return this.getData().setAngularVelOffset(thetaPrime);
    }

    public MotorData getData() {
        // final DcMotorControllerExFake controller = (DcMotorControllerExFake) this.getController();
        return ((DcMotorControllerExFake) this.getController()).getData(this.getPortNumber());
    }

    @Override
    public void remember(Updater updater) {
        this.updaters.add(updater);
    }

    @Override 
    public void forget(Updater updater) {
        this.updaters.remove(updater);
    }
}