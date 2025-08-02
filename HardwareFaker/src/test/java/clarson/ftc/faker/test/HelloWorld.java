package clarson.ftc.faker.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

public class HelloWorld {
    @Test
    @DisplayName("Hello World")
    public void canConstructHelloWorld() {
        assertEquals("Hello World!", new String("Hello World!"));
    }


}