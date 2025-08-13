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
     * Circular references are able to be added. `updateAll()` is expected to 
     * be called on any Updaters that were registered, but only if such updater
     * has not already been traversed; each registered Updater and Updateable
     * is updated only once.
     * 
     * NOTE: Implementations are **strongly** recommended to store registered 
     * `Updateable`s *weakly*, either with weak maps or sets.
     * 
     * @param newUpdateable What to register. A weak reference will be created.
     * @return Whether the Updateable was now added. False if it was added 
     * previously without being unregistered.
     */
    public boolean register(Updateable newUpdateable);

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
     * Updates all Updateables registered with this updater. Management of bulk 
     * caching is not done with this method.
     * 
     * If Updaters (that implement `Updateable`) are registered, they are 
     * expected to have their `updateAll()` method called. Consequently, circular
     * references can arise. However, all Updaters and Updateables should only be
     * updated once and if applicable have their `updateAll()` method called 
     * once, ensuring no runaway recursion or loops.
     * 
     * This method does not guard against multiple updates to the same object 
     * if Updateables update objects other than theirself. This might occur if 
     * an erroneous implementation of, for example, `DcMotorController` updated 
     * all connected motors when it was itself updated with `update()` instead 
     * of `updateAll()`, but all the motors were also connected. If the 
     * DcMotorController had instead been an Updater and updated the motors only
     * when `updateAll()` was called and instead updated only **itself** in 
     * `update()`, every object would only be updated once-- assuming no other
     * erroneous updating.
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
     * If Updaters (that implement `Updateable`) are registered, they are 
     * expected to have their `updateAll()` method called. Consequently, circular
     * references can arise. However, all Updaters and Updateables should only be
     * updated once and if applicable have their `updateAll()` method called 
     * once, ensuring no runaway recursion or loops.
     * 
     * This method does not guard against multiple updates to the same object 
     * if Updateables update objects other than theirself. This might occur if 
     * an erroneous implementation of, for example, `DcMotorController` updated 
     * all connected motors when it was itself updated with `update()` instead 
     * of `updateAll()`, but all the motors were also connected. If the 
     * DcMotorController had instead been an Updater and updated the motors only
     * when `updateAll()` was called and instead updated only **itself** in 
     * `update()`, every object would only be updated once-- assuming no other
     * erroneous updating.
     * 
     * @param delay Source of the delay length from who called this method.
     * @see #updateAll(double deltaSec)
     */
    default public void updateAll(UpdateDelaySource delay) {
        updateAll(delay.length);
        // return updateAll(delay.length);
    }
} 