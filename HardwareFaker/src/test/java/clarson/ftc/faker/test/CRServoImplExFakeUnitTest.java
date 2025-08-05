package clarson.ftc.faker.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

import org.junit.jupiter.api.AssertionFailureBuilder;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.params.Parameter;
import org.junit.jupiter.params.ParameterizedClass;
import org.junit.jupiter.params.provider.ValueSource;

import clarson.ftc.faker.CRServoImplExFake;

import static com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.*;

class CRServoImplExFakeUnitTest {
    private static boolean doesThrow(Executable code) {
        try {
            code.execute();
            return false;
        } catch(Throwable err) {
            return true;
        } 
    }

    private static void assertFloatEquals(double expected, double actual, double eps){
        if(Math.abs(expected - actual) > eps) {
            // throw ("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">");
            // throw new RuntimeException("waaat? o_O");
            AssertionFailureBuilder
                .assertionFailure()
                .reason("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">")
                .message("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">")
                .actual(actual)
                .expected(expected)
                .buildAndThrow();
        }
    }

    @DisplayName("Can construct")
    @Test 
    void canConstruct() {
        assertDoesNotThrow(() -> new CRServoImplExFake(312));
        assertDoesNotThrow(() -> new CRServoImplExFake(-1600));
        assertDoesNotThrow(() -> new CRServoImplExFake(-1600));

        assertDoesNotThrow(() -> new CRServoImplExFake(312, 0));
        assertDoesNotThrow(() -> new CRServoImplExFake(312, 1200));
        assertDoesNotThrow(() -> new CRServoImplExFake(312, -1200));
        assertDoesNotThrow(() -> new CRServoImplExFake(-312, -1200));
        
        assertDoesNotThrow(() -> new CRServoImplExFake(312, 0, new PwmRange(0, 100)));
        assertDoesNotThrow(() -> new CRServoImplExFake(312, 1200, new PwmRange(0, 100, 40)));
        assertDoesNotThrow(() -> new CRServoImplExFake(-312, -1200, new PwmRange(100, 98300)));
        assertDoesNotThrow(() -> new CRServoImplExFake(312, -1200, new PwmRange(1000, 98300, 40)));
    }

    @DisplayName("Initial settings are FORWARD, enabled, and power 0")
    @Test 
    void checkInitialSettings() {
        final CRServoImplEx servo = new CRServoImplExFake(180);
        assertEquals(CRServo.Direction.FORWARD, servo.getDirection());
        assertEquals(0, servo.getPower());
        assertTrue(servo.isPwmEnabled());
    }

    @ParameterizedClass
    @ValueSource(strings = {"FORWARD", "REVERSE"})
    @DisplayName("Construction Dependent")
    @Nested 
    class ConsructionDependent {
        private CRServoImplExFake servo;
        private double rpm = 180;
        private double maxSpeed = rpm / 60;
        private PwmRange maxPwmRange = new PwmRange(1000, 2000);
        
        @Parameter
        private String directionName;

        @BeforeEach 
        void constructMotor() {
            servo = new CRServoImplExFake(rpm, 0, maxPwmRange);
            servo.setPwmRange(new PwmRange(1000, 2000));

            if(directionName.equals("FORWARD")) {
                servo.setDirection(DcMotor.Direction.FORWARD);
            } else if(directionName.equals("REVERSE")) {
                servo.setDirection(DcMotor.Direction.REVERSE);
            }
        }

        @DisplayName("DcMotorSimple Inherited")
        @Nested
        class DcMotorSimpleInherited {
            @DisplayName("Initial Power 0")
            @Test
            void initialPowerIsConstant() {
                assertEquals(0, servo.getPower());
            }

            @DisplayName("Set Power Arg = Get Power")
            @Test
            void setPowerArgumentStored() {
                final double[] powers = { 0, 1.0, 0.0, -1.0, 0.5, 0.25, -0.5, -0.25 };
                for(final double power : powers) {
                    servo.setPower(power);
                    assertEquals(power, servo.getPower());
                }
            }

            /* @Disabled("This is covered by test CRServoImplExFakeUnitTest.checkInitialSettings")
            @DisplayName("Initial Direction Forward")
            @Test 
            void initialDirectionForward() {
                assertEquals(DcMotor.Direction.FORWARD, servo.getDirection());
            }
            */
            
            @DisplayName("Set Direction Arg = Get Direction") 
            @Test
            void setDirectionArgumentStored() {
                final DcMotor.Direction[] dirs = DcMotor.Direction.values();
                for(final DcMotor.Direction dir : dirs) {
                    servo.setDirection(dir);
                    assertEquals(dir, servo.getDirection());
                }
            }

            @DisplayName("Update moves servo forward")
            @Test
            void updateMotorForward() {
                servo.setDirection(DcMotor.Direction.FORWARD);
                servo.setPower(1.0);
                
                // Seeing that the returned delta is as expected
                final double maxTickSpeed = rpm / 60;
                final double seconds = 0.016;
                assertEquals(maxTickSpeed * seconds, servo.update(seconds));

                // Seeing that the returned delta is, in fact, a delta and not the accumulated
                assertNotEquals(3 * maxTickSpeed * seconds, servo.update(seconds * 2));

                // Making sure the delta is accurate with varying input
                assertEquals(1.5 * maxTickSpeed * seconds, servo.update(seconds * 1.5));

                // Making sure the delta is still accurate after a power change
                servo.setPower(-0.33);
                assertEquals(-0.33 * maxTickSpeed * seconds, servo.update(seconds));
            }

            @DisplayName("Update moves servo backward")
            @Test
            void updateMotorBackward() {
                servo.setDirection(DcMotor.Direction.REVERSE);
                servo.setPower(1.0);

                // Seeing that the returned delta is as expected
                final double maxTickSpeed = -rpm / 60;
                final double seconds = 0.016;
                assertEquals(maxTickSpeed * seconds, servo.update(seconds));

                // Seeing that the returned delta is, in fact, a delta and not the accumulated
                assertNotEquals(3 * maxTickSpeed * seconds, servo.update(seconds * 2));

                // Making sure the delta is accurate with varying input
                assertEquals(1.5 * maxTickSpeed * seconds, servo.update(seconds * 1.5));

                // Making sure the delta is still accurate after a power change
                servo.setPower(-0.33);
                assertEquals(-0.33 * maxTickSpeed * seconds, servo.update(seconds));
            }
        
            @DisplayName("Add angular vel only ever adds (unless negative)")
            @Test
            void addVelAddsVelocotyCorrectly() {
                final double maxTickSpeed = rpm / 60;
                final double converson = 1 / (2 * Math.PI); // Radians to ticks

                // Positive Forward
                final double speed1 = 2 * Math.PI / 2;
                servo.setDirection(DcMotor.Direction.FORWARD);
                servo.setPower(1.0);
                servo.addAngularVel(speed1);
                assertEquals(maxTickSpeed + speed1 * converson, servo.update(1));
                
                // Negative Forward
                final double speed2 = -2 * Math.PI / 2;
                servo.setDirection(DcMotor.Direction.FORWARD);
                servo.setPower(1.0);
                servo.addAngularVel(speed2);
                assertEquals(maxTickSpeed + speed2 * converson, servo.update(1));
                
                // Positive Backward
                final double speed3 = 2 * Math.PI / 2;
                servo.setDirection(DcMotor.Direction.REVERSE);
                servo.setPower(1.0);
                servo.addAngularVel(speed3);
                assertEquals(-maxTickSpeed + speed3 * converson, servo.update(1));
                
                // Negative Backward
                final double speed4 = -2 * Math.PI / 2;
                servo.setDirection(DcMotor.Direction.REVERSE);
                servo.setPower(1.0);
                servo.addAngularVel(speed4);
                assertEquals(-maxTickSpeed + speed4 * converson, servo.update(1));
            }

            @DisplayName("Add angular vel persists until setPower")
            @Test
            void verifyAddAnguarVelPersistence() {
                final double maxTickSpeed = rpm / 60;
                
                servo.setDirection(DcMotor.Direction.FORWARD);
                servo.setPower(1.0);
                final double originalDeltaTick = servo.update(1);
                assertEquals(maxTickSpeed, originalDeltaTick);

                servo.addAngularVel(2 * Math.PI);
                final double firstTransformedDeltaTick = servo.update(1);
                assertNotEquals(maxTickSpeed, firstTransformedDeltaTick);
                assertEquals(firstTransformedDeltaTick, servo.update(1));

                servo.addAngularVel(2 * Math.PI);
                final double secondTransformedDeltaTick = servo.update(1);
                assertNotEquals(firstTransformedDeltaTick, secondTransformedDeltaTick);
                assertEquals(secondTransformedDeltaTick, servo.update(1));

                servo.setPower(1.0);
                final double resetDeltaTick = servo.update(1);
                assertNotEquals(firstTransformedDeltaTick, resetDeltaTick);
                assertNotEquals(secondTransformedDeltaTick, resetDeltaTick);
                assertEquals(originalDeltaTick, resetDeltaTick);
                assertEquals(resetDeltaTick, servo.update(1));
            }
        }
    
        @DisplayName("PWM")
        @Nested
        class Pwm {
            @DisplayName("Set Servo Enabled = Get Servo")
            @Test
            void setPwmEnabledArgumentStored() {
                servo.setPwmEnable();
                assertTrue(servo.isPwmEnabled());

                servo.setPwmDisable();
                assertFalse(servo.isPwmEnabled());
            }

            @DisplayName("Disabling servo prevents setting power")
            @Test
            void disableDisablesSetPower() {
                servo.setPower(1.0);
                final double unaffectedPowerDeltaTick = servo.update(1.0);

                servo.setPwmDisable();
                final double noPowerDiabledDelta = servo.update(1.0);
                servo.setPower(1.0);
                final double poweredDisabledDelta = servo.update(1.0);

                assertNotEquals(0, unaffectedPowerDeltaTick);
                assertNotEquals(unaffectedPowerDeltaTick, noPowerDiabledDelta);
                assertNotEquals(unaffectedPowerDeltaTick, poweredDisabledDelta);
                assertEquals(noPowerDiabledDelta, poweredDisabledDelta);

            }

            @DisplayName("Disabled servo affected by addAngularVel")
            @Test 
            void disabledAffectedByAddAngularVel() {
                // final SetVelocityAndPower nestedTested = new AddVelocity();
                // assumeTrue(!doesThrow(nestedTested::addAngularVelWithEncoder));

                servo.setPwmDisable();
                final double delatTick = servo.update(1.0); // Should be 0
                final double speed1 = 1;
                servo.addAngularVel(speed1);
                assertEquals(delatTick + speed1 / (2 * Math.PI), servo.update(1.0));

                final double delatTick2 = servo.update(1.0);
                final double speed2 = -1;
                servo.addAngularVel(speed2);
                assertEquals(delatTick2 + speed2 / (2 * Math.PI), servo.update(1.0));
            }
        
            @DisplayName("Renabled can set power")
            @Test
            void renabledStillWorks() {
                servo.setPower(1.0);

                // Assume it can be disabled in the frist place
                assumeFalse(doesThrow(this::disableDisablesSetPower));
                
                servo.setPwmEnable();
                assertTrue(servo.isPwmEnabled());
                servo.setPower(1.0);
                assertNotEquals(0, servo.update(1.0));
            }
        
            @DisplayName("Brake is stopped when servo is disabled")
            @Test
            void brakeIsStopped() {
                final AddVelocity nestedTested = new AddVelocity();
                assumeFalse(doesThrow(nestedTested::brakingPreventsMovement));
                servo.setPower(0.0);

                // Disabling...
                servo.setPwmDisable();
                final double delatTick = servo.update(1.0); // Should be 0
                final double speed1 = 1;
                servo.addAngularVel(speed1);
                assertEquals(delatTick + speed1 / (2 * Math.PI), servo.update(1.0));

                final double delatTick2 = servo.update(1.0);
                final double speed2 = -1;
                servo.addAngularVel(speed2);
                assertEquals(delatTick2 + speed2 / (2 * Math.PI), servo.update(1.0));
            }
        }

        @DisplayName("Add Velocity Misc.")
        @Nested
        class AddVelocity {
            @DisplayName("Braking prevents being offset")
            @Test
            void brakingPreventsMovement() { 
                servo.setPower(0.0);
                final double deltaRev = servo.update(1);
                assertEquals(0, Math.abs(deltaRev));

                servo.addAngularVel(-0.5 * maxSpeed * 2 * Math.PI);
                assertEquals(0, deltaRev + servo.update(1));
            }

            @DisplayName("Braking can be overpowered")
            @Test
            void brakingCanBeOverpowered() {
                servo.setPower(0.0);
                final double deltaRev = servo.update(1);
                assertEquals(0, Math.abs(deltaRev));

                servo.addAngularVel(2 * maxSpeed * 2 * Math.PI);
                final double affectedDeltaRev = servo.update(1);
                assertNotEquals(deltaRev, affectedDeltaRev);
                assertEquals(maxSpeed, affectedDeltaRev);
            }

            @DisplayName("Braking can be overpowered by two small offsets")
            @Test
            void sumOfPartsOverpowering() {
                servo.setPower(0.0);
                final double deltaRev = servo.update(1);
                assertFloatEquals(0, Math.abs(deltaRev), 1e-10);

                final double fraction = 0.7;
                servo.addAngularVel(fraction * maxSpeed * 2 * Math.PI);
                final double affectedDeltaRev = servo.update(1);
                assertFloatEquals(deltaRev, affectedDeltaRev, 1e-10);
                
                servo.addAngularVel(fraction * maxSpeed * 2 * Math.PI);
                final double affectedDeltaRev2 = servo.update(1);
                assertNotEquals(deltaRev, Math.abs(affectedDeltaRev2));
                assertFloatEquals(maxSpeed * (2 * fraction - 1), affectedDeltaRev2, 1e-1);
            }
        }
    
    }

}