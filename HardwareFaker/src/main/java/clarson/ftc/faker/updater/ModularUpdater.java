package clarson.ftc.faker.updater;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.Collections;
import java.util.Set;
import java.util.WeakHashMap;

/**
 * Calls `update()` to registered Updateables with respect to their
 * given LynxModule. The point of storing an Updateable's related
 * module is to allow for avoiding unnecessary calls to update if
 * a getter's data is in the module's bulk cache, simulating bulk 
 * caching on ordinary LynxModules.
 */
public class ModularUpdater implements Updater {

    protected final WeakHashMap<Updateable, LynxModule> registered = new WeakHashMap<>(52, 1.0f);

    /**
     * Updates all Updateables registered with this updater. If any Updaters
     * have been registered, their inner Updateables will not be updated.
     * 
     * @param deltaSec How much time has elapsed since last call, in seconds.
     */
    @Override
    public void updateAll(double deltaSec) {
        for(final Updateable updateable : registered.keySet()) {
            updateable.update(deltaSec);
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
    @Override
    public boolean hasRegistered(Updateable updateable) {
        return registered.containsKey(updateable);
    }
    
    /**
     * Registers the given Updateable so that it can be updated. Note that all
     * updateables registered are stored **weakly**, so failing to unregister a
     * forgotten updateable does **not** create a memory leak.
     * 
     * The module assigned to the given updateable is null; therefore, any 
     * attempted bulk cache checks will throw a `NullPointerException`. This 
     * does not allow the same Updateable to be added twice, by adding one with
     * a null LynxModule module and another with a correct module; all LynxModules
     * are ignored.
     * 
     * @param updateable What to register. A weak reference will be created.
     * @return Whether the Updateable was now added. False if it was added 
     * previously without being unregistered.
     */
    @Override
    public boolean register(Updateable updateable) {
        if(hasRegistered(updateable)) {
            return false;
        }

        registered.put(updateable, null);
        return true;
    }

    /**
     * Registers the given Updateable so that it can be updated. Note that all
     * updateables registered are stored **weakly**, so failing to unregister a
     * forgotten updateable does **not** create a memory leak.
     * 
     * The Updateable is assigned the given LynxModule, allowing the usage of 
     * bulk cache check methods. This does not allow the same Updateable to be 
     * added twice, by adding one witha null LynxModule module and another with 
     * a correct module; all LynxModules are ignored.
     * 
     * @param updateable What to register. A weak reference will be created.
     * @return Whether the Updateable was now added. False if it was added 
     * previously without being unregistered.
     */
    public boolean register(Updateable updateable, LynxModule module) {
        if(hasRegistered(updateable)) {
            return false;
        }

        registered.put(updateable, module);
        return true;
    }

    /**
     * Removes an updateable. After this is method, `hasRegistered()` for the 
     * given Updateable is guaranteed to return false, until it is registered
     * again, of course.
     * 
     * @param updateable What to unregister. 
     * @return Whether the Updateable was now added. False if it was 
     * not registered (i.e. `hasRegistered()` returned false).
     */
    @Override
    public boolean unregister(Updateable updateable) {
        if(!hasRegistered(updateable)) {
            return false;
        }

        registered.remove(updateable);
        return true;
    }
}