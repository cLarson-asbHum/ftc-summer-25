package clarson.ftc.faker.updater;

import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;

import clarson.ftc.faker.LynxModuleHardwareFake;

import java.util.Collections;
import java.util.NoSuchElementException;
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

    protected final WeakHashMap<Updateable, LynxModuleHardwareFake> registered = new WeakHashMap<>(52, 1.0f);

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
    public boolean register(Updateable updateable, LynxModuleHardwareFake module) {
        if(hasRegistered(updateable)) {
            return false;
        }

        registered.put(updateable, module);
        return true;
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
    public boolean register(TwoWayUpdateable updateable) {
        if(hasRegistered(updateable)) {
            return false;
        }

        updateable.remember(this);
        registered.put(updateable, null);
        return true;
    }

    /**
     * Registers the given Updateable so that it can be updated. Note that all
     * updateables registered are stored **weakly**, so failing to unregister a
     * forgotten updateable does **not** create a memory leak. 
     * 
     * This also causes the given TwoWayUpdateable to remember that it was 
     * registered with this ModularUpdater. The memory lasts an indefinite 
     * amount of time but is guaranteed to end when `forget()` is called, which
     * can happend through `unregister()`.
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
    public boolean register(TwoWayUpdateable updateable, LynxModuleHardwareFake module) {
        if(hasRegistered(updateable)) {
            return false;
        }

        updateable.remember(this);
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

    /**
     * Removes an updateable. After this is method, `hasRegistered()` for the 
     * given Updateable is guaranteed to return false, until it is registered
     * again, of course. The Updateable then forgets that it was ever 
     * registered with this Updater.
     * 
     * @param updateable What to unregister. 
     * @return Whether the Updateable was now added. False if it was 
     * not registered (i.e. `hasRegistered()` returned false).
     */
    @Override
    public boolean unregister(TwoWayUpdateable updateable) {
        if(!hasRegistered(updateable)) {
            return false;
        }

        updateable.forget(this);
        registered.remove(updateable);
        return true;
    }

    /**
     * Updates all registered Updateables only if the bulk cache for the given 
     * Updateable's module does not contain up-to-date data. This always updates
     * if the module is in `BulkCachingMode.OFF` and never updates if in 
     * `MANUAL`. `BulkCachingMode.AUTO` works as normal, updating if the cache 
     * is not clear and the same device (or, more accurately, Updateable), has 
     * called for the same data twice since the last obtained bulk data.
     * 
     * If the Updateable is not registered, its LynxModuleHardwareFake is null, 
     * or is itself null, then a `NullPointerException` is thrown. 
     * 
     * @param deltaSec How much time has elapsed since last call, in seconds.
     * @param updateable Who issued the command for bulk-able data. 
     * @param command Dictates how the data is to be requested. 
     * @param tag The bulk data "group". Only commands sent with the same group
     * can cause a module in `BulkCachingMode.AUTO` to get new BulkData
     * @return Whether all registered Updateables were updated.
     */
    public boolean updateAllIfCacheOutdated(
        double delaySec,
        Updateable updateable, 
        LynxDekaInterfaceCommand<?> command, 
        String tag
    ) {
        // Getting the module
        final LynxModuleHardwareFake module = registered.get(updateable);

        if(!hasRegistered(updateable)) {
            throw new NoSuchElementException("Updateable " + updateable + " has not been registered.");
        }

        if(module == null) {
            throw new NullPointerException("Cannot check bulk cache on null LynxModuleHardwareFake");
        }

        // Getting the new bulk data. The LynxUsbDeviceImplFake does not simulate delay upon transmission
        // (at least, as of Aug 23 2025) and so we test for whether the bulk would be reset by this command.
        if(module.wouldIssueBulkData(command, tag)) {
            updateAll(delaySec);
            return true;
        }

        // Bulk data did not change. No updating done.
        return false;
    }
    
    /**
     * Updates all registered Updateables only if the bulk cache for the given 
     * Updateable's module does not contain up-to-date data. This always updates
     * if the module is in `BulkCachingMode.OFF` and never updates if in 
     * `MANUAL`. `BulkCachingMode.AUTO` works as normal, updating if the cache 
     * is not clear and the same device (or, more accurately, Updateable), has 
     * called for the same data twice since the last obtained bulk data.
     * 
     * If the Updateable is not registered, its LynxModuleHardwareFake is null, 
     * or is itself null, then a `NullPointerException` is thrown. 
     * 
     * @param delay Source of the delay length from who called this method.
     * @param updateable Who issued the command for bulk-able data. 
     * @param command Dictates how the data is to be requested. 
     * @param tag The bulk data "group". Only commands sent with the same group
     * can cause a module in `BulkCachingMode.AUTO` to get new BulkData
     * @return Whether all registered Updateables were updated.
     */
    public boolean updateAllIfCacheOutdated(
        UpdateDelaySource delay,
        Updateable updateable, 
        LynxDekaInterfaceCommand<?> command, 
        String tag
    ) {
        return updateAllIfCacheOutdated(delay.length, updateable, command, tag);
    }
    
    /**
     * Updates all registered Updateables only if the bulk cache for the given 
     * Updateable's module does not contain up-to-date data. This always updates
     * if the module is in `BulkCachingMode.OFF` and never updates if in 
     * `MANUAL`. `BulkCachingMode.AUTO` works as normal, updating if the cache 
     * is not clear and the same device (or, more accurately, Updateable), has 
     * called for the same data twice since the last obtained bulk data.
     * 
     * If the Updateable is not registered, its LynxModuleHardwareFake is null, 
     * or is itself null, then a `NullPointerException` is thrown. 
     * 
     * @param delay Source of the delay length from who called this method.
     * @param updateable Who issued the command for bulk-able data. 
     * @param command Dictates how the data is to be requested. 
     * @return Whether all registered Updateables were updated.
     */
    public boolean updateAllIfCacheOutdated(
        UpdateDelaySource delay,
        Updateable updateable, 
        LynxDekaInterfaceCommand<?> command
    ) {
        return updateAllIfCacheOutdated(delay.length, updateable, command, "");
    }
    
    /**
     * Updates all registered Updateables only if the bulk cache for the given 
     * Updateable's module does not contain up-to-date data. This always updates
     * if the module is in `BulkCachingMode.OFF` and never updates if in 
     * `MANUAL`. `BulkCachingMode.AUTO` works as normal, updating if the cache 
     * is not clear and the same device (or, more accurately, Updateable), has 
     * called for the same data twice since the last obtained bulk data.
     * 
     * If the Updateable is not registered, its LynxModuleHardwareFake is null, 
     * or is itself null, then a `NullPointerException` is thrown. 
     * 
     * @param deltaSec How much time has elapsed since last call, in seconds.
     * @param updateable Who issued the command for bulk-able data. 
     * @param command Dictates how the data is to be requested. 
     * @return Whether all registered Updateables were updated.
     */
    public boolean updateAllIfCacheOutdated(
        double delaySec,
        Updateable updateable, 
        LynxDekaInterfaceCommand<?> command
    ) {
        return updateAllIfCacheOutdated(delaySec, updateable, command, "");
    }
}