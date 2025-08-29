package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import clarson.ftc.faker.updater.Updater;
import clarson.ftc.faker.updater.Rotateable;
import clarson.ftc.faker.updater.TwoWayUpdateable;

import java.util.Set;
import java.util.HashSet;

public class CRServoImplExFake extends CRServoImplEx implements Rotateable, TwoWayUpdateable {
    public final static ServoConfigurationType getFakeConfiguration(ContinuousServoData data) {
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
    
    public CRServoImplExFake(double rpm) {
        this(rpm, 0, PwmRange.defaultRange);
    }

    public CRServoImplExFake(double rpm, double initialPosition) {
        this(rpm, initialPosition, PwmRange.defaultRange);
    }

    public CRServoImplExFake(double rpm, double initialPosition, PwmRange maxRange) {
        this(
            new ContinuousServoData(rpm, initialPosition, maxRange), 
            new ServoControllerExFake(), 
            0
        );
    }

    public CRServoImplExFake(ContinuousServoData data, ServoControllerExFake controller, int portNumber) {
        super(
            controller, 
            portNumber, 
            CRServoImplExFake.Direction.FORWARD, 
            getFakeConfiguration(data)
        );

        if(!controller.connect(ContinuousServoData.copyForServo(this, data))) {
            throw new IllegalArgumentException("Port number <" + portNumber + "> is not avaiable on controller");
        }

        controller.setServoType(portNumber, getFakeConfiguration(data));
    }

    public CRServoImplExFake(ContinuousServoData data, ServoControllerExFake controller) {
        this(data, controller, findAvaiablePort(controller));
    }

    @Override
    public double update(double deltaSec) {       
        return this.getData().update(deltaSec);
    }

    @Override
    public boolean setUpdatingEnabled(boolean newUpdatingEnabled) {
        return this.getData().setUpdatingEnabled(newUpdatingEnabled);
    }

    @Override
    public boolean isUpdatingEnabled() {
        return this.getData().isUpdatingEnabled();
    }

    @Override
    public double addAngularVelOffset(double thetaPrime) {
        return this.getData().addAngularVelOffset(thetaPrime);
    }
    
    @Override
    public double setAngularVelOffset(double thetaPrime) {
        return this.getData().setAngularVelOffset(thetaPrime);
    }

    public ContinuousServoData getData() {
        final ServoControllerExFake controller = (ServoControllerExFake) this.getController();
        return (ContinuousServoData) controller.getData(this.getPortNumber());
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