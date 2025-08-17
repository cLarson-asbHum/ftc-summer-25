package clarson.ftc.faker.test.updater;

import clarson.ftc.faker.ContinuousServoData;
import clarson.ftc.faker.CRServoImplExFake;
import clarson.ftc.faker.DcMotorImplExFake;
import clarson.ftc.faker.LynxUsbDeviceImplFake;
import clarson.ftc.faker.MotorData;
import clarson.ftc.faker.ServoData;
import clarson.ftc.faker.updater.ModularUpdater;
import clarson.ftc.faker.updater.Updateable;

import static clarson.ftc.faker.test.TestUtil.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.LynxModuleDescription;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.junit.jupiter.api.AssertionFailureBuilder;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.params.Parameter;
import org.junit.jupiter.params.ParameterizedClass;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.*;

class ModularUpdaterUnitTest {
    @DisplayName("Can construct")
    @Test
    void canConstruct() {
        assertDoesNotThrow(() -> new ModularUpdater());
    }

    @DisplayName("Post Construct")
    @Nested
    class PostConstruct {
        final LynxModuleDescription controlDesc = new LynxModuleDescription.Builder(1, true)
            .setUserModule()
            .build();
        final LynxModuleDescription expansionDesc = new LynxModuleDescription.Builder(2, false)
            .setUserModule()
            .build();

        final LynxUsbDeviceImplFake device = new LynxUsbDeviceImplFake();
        LynxModule control;
        LynxModule expansion;

        ModularUpdater updater;

        @BeforeEach
        void construct() throws InterruptedException, RobotCoreException {
            updater = new ModularUpdater();
            control = device.getOrAddModule(controlDesc);
            expansion = device.getOrAddModule(expansionDesc);
        }

        @DisplayName("Can register")
        @Test
        void canRegister() throws InterruptedException, RobotCoreException {
            // No module registers
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(312, 576.6)));
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(1600, 100)));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(180)));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(60)));
            
            // Control Hub  registers
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(1, 2), control));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(1, 2), control));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(3, 4), control));
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(3, 4), control));
            
            // Expansion Hub  registers
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(1, 2), expansion));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(1, 2), expansion));
            assertDoesNotThrow(() -> updater.register(new CRServoImplExFake(3, 4), expansion));
            assertDoesNotThrow(() -> updater.register(new DcMotorImplExFake(3, 4), expansion));
        }

        @DisplayName("Register fails on duplicates and passes otherwise")
        @Test
        void duplicateRegistersFail() throws InterruptedException, RobotCoreException {

            // No module registers
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertTrue(updater.register(motorNone));
            assertTrue(updater.register(new DcMotorImplExFake(1600, 100)));
            assertTrue(updater.register(servoNone));
            assertTrue(updater.register(new CRServoImplExFake(180)));
            assertFalse(updater.register(motorNone));
            assertFalse(updater.register(servoNone));
            

            // Control hub module registers
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoCont = new CRServoImplExFake(60);
            assertTrue(updater.register(motorCont, control));
            assertTrue(updater.register(new DcMotorImplExFake(1600, 100), control));
            assertTrue(updater.register(servoCont, control));
            assertTrue(updater.register(new CRServoImplExFake(180), control));
            assertFalse(updater.register(motorCont, control));
            assertFalse(updater.register(servoCont, control));
            
            // Expansion hub registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoExp = new CRServoImplExFake(60);
            assertTrue(updater.register(motorExp, expansion));
            assertTrue(updater.register(new DcMotorImplExFake(1600, 100), expansion));
            assertTrue(updater.register(servoExp, expansion));
            assertTrue(updater.register(new CRServoImplExFake(180), expansion));
            assertFalse(updater.register(motorExp, expansion));
            assertFalse(updater.register(servoExp, expansion));

            // Duplicates, with different modules
            assertFalse(updater.register(motorCont)); // Equal to nul hub
            assertFalse(updater.register(servoCont)); // Equal to nul hub

            assertFalse(updater.register(motorCont, expansion));
            assertFalse(updater.register(servoCont, expansion));

            assertFalse(updater.register(motorExp)); // Equal to null hub
            assertFalse(updater.register(servoExp)); // Equal to null hub

            assertFalse(updater.register(motorExp, control));
            assertFalse(updater.register(servoExp, control));

            assertFalse(updater.register(motorNone, control));
            assertFalse(updater.register(servoNone, control));

            assertFalse(updater.register(motorNone, expansion));
            assertFalse(updater.register(servoNone, expansion));

        }

        @DisplayName("Can unregister")
        @Test
        void canUnregister() {
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));

            // Unregistering null hub  updateables.
            assertDoesNotThrow(() -> updater.unregister(motorNone));
            assertDoesNotThrow(() -> updater.unregister(servoNone));
            
            // Unregistering control hub updateables. 
            assertDoesNotThrow(() -> updater.unregister(motorCont));
            assertDoesNotThrow(() -> updater.unregister(servoCont));

            // Unregistering expansion hub updateables. 
            assertDoesNotThrow(() -> updater.unregister(motorExp));
            assertDoesNotThrow(() -> updater.unregister(servoExp));
        }

        @DisplayName("Unregistering something nonexistent fails; passes otherwise")
        @Test
        void unregisterNonexistentFails() {
            
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertFalse(() -> updater.unregister(motorNone));
            assertFalse(() -> updater.unregister(servoNone));
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertFalse(() -> updater.unregister(motorCont));
            assertFalse(() -> updater.unregister(servoCont));
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertFalse(() -> updater.unregister(motorExp));
            assertFalse(() -> updater.unregister(servoExp));
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));

            // Unregistering null hub  updateables.
            assertTrue(() -> updater.unregister(motorNone));
            assertFalse(() -> updater.unregister(motorNone));
            assertTrue(() -> updater.unregister(servoNone));
            assertFalse(() -> updater.unregister(servoNone));
            
            // Unregistering control hub updateables. 
            assertTrue(() -> updater.unregister(motorCont));
            assertFalse(() -> updater.unregister(motorCont));
            assertTrue(() -> updater.unregister(servoCont));
            assertFalse(() -> updater.unregister(servoCont));

            // Unregistering expansion hub updateables. 
            assertTrue(() -> updater.unregister(motorExp));
            assertFalse(() -> updater.unregister(motorExp));
            assertTrue(() -> updater.unregister(servoExp));
            assertFalse(() -> updater.unregister(servoExp));
        }

        @DisplayName("Has registered identifies registered")
        @Test
        void registerCanReturnTrue() {
            
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            assertTrue(updater.hasRegistered(motorNone));
            assertTrue(updater.hasRegistered(servoNone));
            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            assertTrue(updater.hasRegistered(motorCont));
            assertTrue(updater.hasRegistered(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));
            assertTrue(updater.hasRegistered(motorExp));
            assertTrue(updater.hasRegistered(servoExp));
        }

        @DisplayName("Has registered identifies unregistered")
        @Test
        void registerCanReturnFalse() {
            // Registering.
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            assertFalse(updater.hasRegistered(motorNone));
            assertFalse(updater.hasRegistered(servoNone));
            assertTrue(() -> updater.register(motorNone));
            assertTrue(() -> updater.register(servoNone));
            assertTrue(updater.unregister(servoNone));
            assertTrue(updater.unregister(motorNone));
            assertFalse(updater.hasRegistered(motorNone));
            assertFalse(updater.hasRegistered(servoNone));

            
            // Control Hub  registers
            // NOTE: The parameters shouldn't matter as they all have different hash() values,
            //      but if there was ever a change to that, no reason to fail.
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            assertFalse(updater.hasRegistered(motorCont));
            assertFalse(updater.hasRegistered(servoCont));
            assertTrue(() -> updater.register(motorCont));
            assertTrue(() -> updater.register(servoCont));
            assertTrue(updater.unregister(servoCont));
            assertTrue(updater.unregister(motorCont));
            assertFalse(updater.hasRegistered(motorCont));
            assertFalse(updater.hasRegistered(servoCont));
            
            // Expansion Hub  registers
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);
            assertFalse(updater.hasRegistered(motorExp));
            assertFalse(updater.hasRegistered(servoExp));
            assertTrue(() -> updater.register(motorExp));
            assertTrue(() -> updater.register(servoExp));
            assertTrue(updater.unregister(servoExp));
            assertTrue(updater.unregister(motorExp));
            assertFalse(updater.hasRegistered(motorExp));
            assertFalse(updater.hasRegistered(servoExp));
        }

        @DisplayName("Post Register")
        @Nested
        class PostRegister {
            final DcMotorImplExFake motorNone = new DcMotorImplExFake(312, 576.6);
            final CRServoImplExFake servoNone = new CRServoImplExFake(60);
            final DcMotorImplExFake motorCont = new DcMotorImplExFake(1, 2);
            final CRServoImplExFake servoCont = new CRServoImplExFake(120);
            final DcMotorImplExFake motorExp = new DcMotorImplExFake(2, 3);
            final CRServoImplExFake servoExp = new CRServoImplExFake(180);

            @BeforeEach
            void addHardware() {
                assertTrue(updater.register(motorNone, null));
                assertTrue(updater.register(servoNone, null));
                assertTrue(updater.register(motorCont, control));
                assertTrue(updater.register(servoCont, control));
                assertTrue(updater.register(motorExp, expansion));
                assertTrue(updater.register(servoExp, expansion));
            }

            @DisplayName("UpdateAll accurrately updates all added hardware")
            @Test
            void updateAllUpdatesAllHardware() {
                // Getting the original positions
                final MotorData motorNoneData = motorNone.getData();
                final ServoData servoNoneData = servoNone.getData();
                final MotorData motorContData = motorCont.getData();
                final ServoData servoContData = servoCont.getData();
                final MotorData motorExpData = motorExp.getData();
                final ContinuousServoData servoExpData = servoExp.getData();

                final double motorNoneOg = motorNoneData.position;
                final double servoNoneOg = servoNoneData.position;
                final double motorContOg = motorContData.position;
                final double servoContOg = servoContData.position;
                final double motorExpOg = motorExpData.position;
                final double servoExpOg = servoExpData.position;

                // Setting the powers
                motorNone.setPower(1.0);
                servoNone.setPower(1.0);

                motorCont.setPower(-1.0);
                servoCont.setPower(0.5);

                motorExp.setPower(0);
                servoExp.setDirection(DcMotorImplExFake.Direction.REVERSE);
                servoExp.setPower(1.0);
                servoExp.setAngularVelOffset(1.5 * 2 * Math.PI * servoExpData.maxRevsPerSec);

                // Updating and comparing
                updater.updateAll(1.0);
                
                final double eps = 1e-13;
                assertFloatEquals(motorNoneOg + motorNoneData.maxTicksPerSec, motorNoneData.position, eps);
                assertFloatEquals(servoNoneOg + servoNoneData.maxRevsPerSec, servoNoneData.position, eps);

                assertFloatEquals(motorContOg - motorContData.maxTicksPerSec, motorContData.position, eps);
                assertFloatEquals(servoContOg + 0.5 * servoContData.maxRevsPerSec, servoContData.position, eps);

                assertFloatEquals(motorExpOg, motorExpData.position, eps);
                assertFloatEquals(servoExpOg + 0.5 * servoExpData.maxRevsPerSec, servoExpData.position, eps);
            }

            @DisplayName("UpdateAll ignores unregistered hardware")
            @Test
            void updateAllIgnoresUnregisteredHardware() {
                // Setting powers and doing and checkinig that they can move upon update
                updateAllUpdatesAllHardware();

                // Getting the data
                final MotorData motorNoneData = motorNone.getData();
                final ServoData servoNoneData = servoNone.getData();
                final MotorData motorContData = motorCont.getData();
                final ServoData servoContData = servoCont.getData();
                final MotorData motorExpData = motorExp.getData();
                final ServoData servoExpData = servoExp.getData();

                final double motorNoneOg = motorNoneData.position;
                final double servoNoneOg = servoNoneData.position;
                final double motorContOg = motorContData.position;
                final double servoContOg = servoContData.position;
                final double motorExpOg = motorExpData.position;
                final double servoExpOg = servoExpData.position;

                // Unregistering
                assertTrue(() -> updater.unregister(motorNone));
                assertTrue(() -> updater.unregister(servoNone));
                assertTrue(() -> updater.unregister(motorCont));
                assertTrue(() -> updater.unregister(servoCont));
                assertTrue(() -> updater.unregister(motorExp));
                assertTrue(() -> updater.unregister(servoExp));

                // Updating and comparing
                updater.updateAll(1.0);
                
                final double eps = 1e-13;
                assertFloatEquals(motorNoneOg, motorNoneData.position, eps);
                assertFloatEquals(servoNoneOg, servoNoneData.position, eps);

                assertFloatEquals(motorContOg, motorContData.position, eps);
                assertFloatEquals(servoContOg, servoContData.position, eps);

                assertFloatEquals(motorExpOg, motorExpData.position, eps);
                assertFloatEquals(servoExpOg, servoExpData.position, eps);
            }
        }
    }
}