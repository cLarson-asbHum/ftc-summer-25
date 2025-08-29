package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import clarson.ftc.faker.updater.Updater;
import clarson.ftc.faker.updater.Rotateable;
import clarson.ftc.faker.updater.TwoWayUpdateable;

import java.util.HashSet;
import java.util.Set;

public class ServoImplExFake extends ServoImplEx implements Rotateable, TwoWayUpdateable {
    public final static ServoConfigurationType getFakeConfiguration(PositionalServoData data) {
        return new ServoConfigurationType();
    }

    /**
     * Finds the lowest valued, unnoccupied port on the controller. If none are
     * found, -1 is returned.
     * 
     * @return The lowest avaiable port, or -1 if none exists.
     */
    private static int findAvaiablePort(ServoControllerExFake controller) {
        for(int i = 0; i < 4; i++) {
            controller.isPortAvailable(i);
        }

        // The method would've early returned if any was avaiable
        return -1;
    }

    private Set<Updater> updaters = new HashSet<>();

    public ServoImplExFake(double rpm, double maxRevolutions) {
        this(rpm, maxRevolutions, 0, PwmRange.defaultRange);
    }

    public ServoImplExFake(double rpm, double maxRevolutions, double initialPosition) {
        this(rpm, maxRevolutions, initialPosition, PwmRange.defaultRange);
    }

    public ServoImplExFake(
        double rpm, 
        double maxRevolutions,
        double initialPosition, 
        PwmRange maxRange
    ) {
        this(
            new PositionalServoData(rpm, maxRevolutions, initialPosition, maxRange), 
            new ServoControllerExFake(), 
            0
        );
    }

    public ServoImplExFake(PositionalServoData data, ServoControllerExFake controller, int portNumber) {
        super(
            controller, 
            portNumber, 
            ServoImplExFake.Direction.FORWARD, 
            getFakeConfiguration(data)
        );

        if(!controller.connect(PositionalServoData.copyForServo(this, data))) {
            throw new IllegalArgumentException("Port number <" + portNumber + "> is not avaiable on controller");
        }

        controller.setServoType(portNumber, getFakeConfiguration(data));
    }

    public ServoImplExFake(PositionalServoData data, ServoControllerExFake controller) {
        this(data, controller, findAvaiablePort(controller));
    }

    public PositionalServoData getData() {
        // What is this, Lisp?
        return (PositionalServoData) (
            ((ServoControllerExFake) this.getController())
                .getData(this.getPortNumber())
        );
    }

    @Override
    public double setAngularVelOffset(double thetaPrime) {
        return getData().setAngularVelOffset(thetaPrime);
    }
    
    @Override
    public double addAngularVelOffset(double thetaPrime) {
        return getData().addAngularVelOffset(thetaPrime);
    }
    
    @Override
    public double update(double deltaSec) {
        return getData().update(deltaSec);
    }

    @Override
    public boolean isUpdatingEnabled() {
        return this.getData().isUpdatingEnabled();
    }

    @Override
    public boolean setUpdatingEnabled(boolean newUpdatingEnabled) {
        return this.getData().setUpdatingEnabled(newUpdatingEnabled);
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