package clarson.ftc.faker.updater;

import static com.qualcomm.hardware.lynx.LynxModule.BulkData;

public interface Updater {
    /**
     * Stores the length of delay for automatic updates from the source of the 
     * update. All times are in seconds.
     */
    public enum UpdateDelaySource {
        /** DcMotor subclass or controller called `updateAll()` */   
        MOTOR(0.0025),

        /** Servo/CRServo subclass or controller called `updateAll()` */   
        SERVO(0.0025),

        /** Digital Device or controller called `updateAll()` */ 
        DIGITAL(0.0025),

        /** Analog Device or controller called `updateAll()` */  
        ANALOG(0.0025),

        /** 
         * I2C Device or controller called `updateAll()`. I2C devices ordinarily 
         * take multiple LynxMessages to read, causing a longer delay. 
         */     
        I2C(0.007);

        public double length; // Seconds

        private UpdateDelaySource(double lengthSec) {
            this.length = lengthSec;
        }
    }

    /**
     * Determines whether the given updateable has been registered. If the given
     * updateable has been unregistered after its last registration, this
     * method will return false.
     * 
     * @param updateable What to check for.
     * @return Whether the Updateable was registered and hasn't been 
     * unregistered since
     */
    public boolean hasRegistered(Updateable updateable);

    /**
     * Registers the given Updateable so that it can be updated by this `Updater`.
     * 
     * Circular references are handled on a case-by-case basis, depending on the
     * specific implementation. 
     * 
     * @param newUpdateable What to register. A weak reference will be created.
     * @return Whether the Updateable was now added. False if it was added 
     * previously without being unregistered.
     */
    public boolean register(Updateable newUpdateable);

    /**
     * Registers the given Updateable so that it can be updated by this `Updater`.
     * The Updateable then remembers the given Updater using its `remember` 
     * method.
     * 
     * Circular references are handled on a case-by-case basis, depending on the
     * specific implementation. 
     * 
     * @param newUpdateable What to register. A weak reference will be created.
     * This Updateable will also remember the Updater it was added to.
     * @return Whether the Updateable was now added. False if it was added 
     * previously without being unregistered.
     */
    public boolean register(TwoWayUpdateable newUpdateable);

    /**
     * Removes an updateable. After this is method, `hasRegistered()` for the 
     * given Updateable is guaranteed to return false, until it is registered
     * again, of course.
     * 
     * @param oldUpdateable What to find and unregister.
     * @return Whether the Updateable was now added. False if it was 
     * not registered (i.e. `hasRegistered()` returned false).
     */
    public boolean unregister(Updateable oldUpdateable);

    /**
     * Removes an updateable. After this is method, `hasRegistered()` for the 
     * given Updateable is guaranteed to return false, until it is registered
     * again, of course. The Updateable will forget that it was registered to
     * this Updater.
     * 
     * @param oldUpdateable What to find and unregister. Forgets this Updater
     * @return Whether the Updateable was now added. False if it was 
     * not registered (i.e. `hasRegistered()` returned false).
     */
    public boolean unregister(TwoWayUpdateable oldUpdateable);

    /**
     * Updates all Updateables registered with this updater. Management of bulk 
     * caching is not done with this method.
     * 
     * Any registered Updaters will not have their `updateAll()` method called,
     * unless it is called in its own `update()` method. This is, however, 
     * **not recommended** as registered updateables may call updateAll() on
     * the Updater, causing an unintended double update.
     * 
     * @param deltaSec How much time has elapsed since last call, in seconds.
     * @see #updateAll(UpdateDelaySource delay)
     */
    public void updateAll(double deltaSec);

    /**
     * Updates all Updateables registered with this updater using the specified
     * source of a delay length. Management of bulk caching is not done with 
     * this method.
     * 
     * Any registered Updaters will not have their `updateAll()` method called,
     * unless it is called in its own `update()` method. This is, however, 
     * **not recommended** as registered updateables may call updateAll() on
     * the Updater, causing an unintended double update.
     * 
     * @param delay Source of the delay length from who called this method.
     * @see #updateAll(double deltaSec)
     */
    default public void updateAll(UpdateDelaySource delay) {
        updateAll(delay.length);
        // return updateAll(delay.length);
    }
} 