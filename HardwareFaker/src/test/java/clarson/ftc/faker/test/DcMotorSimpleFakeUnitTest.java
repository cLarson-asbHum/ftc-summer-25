package clarson.ftc.faker.test;

import clarson.ftc.faker.DcMotorSimpleFake;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

@DisplayName("DcMotorSimpleFake")
class DcMotorSimpleFakeUnitTest {

    @DisplayName("Can construct")
    @Test 
    void canConstruct() {
        assertDoesNotThrow(() -> new DcMotorSimpleFake(312, 576.6));
        assertDoesNotThrow(() -> new DcMotorSimpleFake(-1600, 100));
        assertDoesNotThrow(() -> new DcMotorSimpleFake(-1600, 0));

        assertDoesNotThrow(() -> new DcMotorSimpleFake(312, 100, 0));
        assertDoesNotThrow(() -> new DcMotorSimpleFake(312, 100, 1200));
        assertDoesNotThrow(() -> new DcMotorSimpleFake(312, 100, -1200));
        assertDoesNotThrow(() -> new DcMotorSimpleFake(-312, 100, -1200));
    }

    @DisplayName("Post-Construct")
    @Nested
    class ConstructionDependent {
        private DcMotorSimpleFake motor;
        private double rpm = 312;
        private double ticksPerRev = 576.6;

        @BeforeEach 
        void constructMotor() {
            motor = new DcMotorSimpleFake(rpm, ticksPerRev);
        }

        @DisplayName("Initial Power 0")
        @Test
        void initialPowerIsConstant() {
            assertEquals(0, motor.getPower());
        }

        @DisplayName("Set Power Arg = Get Power")
        @Test
        void setPowerArgumentStored() {
            final double[] powers = { 0, 1.0, 0.0, -1.0, 0.5, 0.25, -0.5, -0.25 };
            for(final double power : powers) {
                motor.setPower(power);
                assertEquals(power, motor.getPower());
            }
        }

        @DisplayName("Initial Direction Forward")
        @Test 
        void initialDirectionForward() {
            assertEquals(DcMotor.Direction.FORWARD, motor.getDirection());
        }

        @DisplayName("Set Direction Arg = Get Direction") 
        @Test
        void setDirectionArgumentStored() {
            final DcMotor.Direction[] dirs = DcMotor.Direction.values();
            for(final DcMotor.Direction dir : dirs) {
                motor.setDirection(dir);
                assertEquals(dir, motor.getDirection());
            }
        }

        @DisplayName("Update moves motor forward")
        @Test
        void updateMotorForward() {
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setPower(1.0);
            
            // Seeing that the returned delta is as expected
            final double maxTickSpeed = rpm * ticksPerRev / 60;
            final double seconds = 0.016;
            assertEquals(maxTickSpeed * seconds, motor.update(seconds));

            // Seeing that the returned delta is, in fact, a delta and not the accumulated
            assertNotEquals(3 * maxTickSpeed * seconds, motor.update(seconds * 2));

            // Making sure the delta is accurate with varying input
            assertEquals(1.5 * maxTickSpeed * seconds, motor.update(seconds * 1.5));

            // Making sure the delta is still accurate after a power change
            motor.setPower(-0.33);
            assertEquals(-0.33 * maxTickSpeed * seconds, motor.update(seconds));
        }

        @DisplayName("Update moves motor backward")
        @Test
        void updateMotorBackward() {
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setPower(1.0);

            // Seeing that the returned delta is as expected
            final double maxTickSpeed = -rpm * ticksPerRev / 60;
            final double seconds = 0.016;
            assertEquals(maxTickSpeed * seconds, motor.update(seconds));

            // Seeing that the returned delta is, in fact, a delta and not the accumulated
            assertNotEquals(3 * maxTickSpeed * seconds, motor.update(seconds * 2));

            // Making sure the delta is accurate with varying input
            assertEquals(1.5 * maxTickSpeed * seconds, motor.update(seconds * 1.5));

            // Making sure the delta is still accurate after a power change
            motor.setPower(-0.33);
            assertEquals(-0.33 * maxTickSpeed * seconds, motor.update(seconds));
        }
    
        @DisplayName("Add angular vel only ever adds (unless negative)")
        @Test
        void addVelAddsVelocotyCorrectly() {
            final double maxTickSpeed = rpm / 60 * ticksPerRev;
            final double converson = ticksPerRev / (2 * Math.PI); // Radians to ticks

            // Positive Forward
            final double speed1 = 2 * Math.PI / 2;
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setPower(1.0);
            motor.addAngularVel(speed1);
            assertEquals(maxTickSpeed + speed1 * converson, motor.update(1));
            
            // Negative Forward
            final double speed2 = -2 * Math.PI / 2;
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setPower(1.0);
            motor.addAngularVel(speed2);
            assertEquals(maxTickSpeed + speed2 * converson, motor.update(1));
            
            // Positive Backward
            final double speed3 = 2 * Math.PI / 2;
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setPower(1.0);
            motor.addAngularVel(speed3);
            assertEquals(-maxTickSpeed + speed3 * converson, motor.update(1));
            
            // Negative Backward
            final double speed4 = -2 * Math.PI / 2;
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setPower(1.0);
            motor.addAngularVel(speed4);
            assertEquals(-maxTickSpeed + speed4 * converson, motor.update(1));
        }

        @DisplayName("Add angular vel persists until setPower")
        @Test
        void verifyAddAnguarVelPersistence() {
            final double maxTickSpeed = rpm / 60 * ticksPerRev;
            
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setPower(1.0);
            final double originalDeltaTick = motor.update(1);
            assertEquals(maxTickSpeed, originalDeltaTick);

            motor.addAngularVel(2 * Math.PI);
            final double firstTransformedDeltaTick = motor.update(1);
            assertNotEquals(maxTickSpeed, firstTransformedDeltaTick);
            assertEquals(firstTransformedDeltaTick, motor.update(1));

            motor.addAngularVel(2 * Math.PI);
            final double secondTransformedDeltaTick = motor.update(1);
            assertNotEquals(firstTransformedDeltaTick, secondTransformedDeltaTick);
            assertEquals(secondTransformedDeltaTick, motor.update(1));

            motor.setPower(1.0);
            final double resetDeltaTick = motor.update(1);
            assertNotEquals(firstTransformedDeltaTick, resetDeltaTick);
            assertNotEquals(secondTransformedDeltaTick, resetDeltaTick);
            assertEquals(originalDeltaTick, resetDeltaTick);
            assertEquals(resetDeltaTick, motor.update(1));
        }
    }
}