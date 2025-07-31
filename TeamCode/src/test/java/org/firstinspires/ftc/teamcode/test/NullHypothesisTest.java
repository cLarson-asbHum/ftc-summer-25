package org.firstinspires.ftc.teamcode.test;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class NullHypothesisTest {
    @Test
    public void obivouslyEqualsIsTrue() {
        assertEquals(1 + 1, 2, "1 + 1 == 2");
    }

    @Test 
    public void obviousNotEqualsIsFalse() {
        assertEquals(4 + 3, 2, "4 + 3 == 2");
    }
}