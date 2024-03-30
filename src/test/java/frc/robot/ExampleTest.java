package frc.robot;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ExampleTest
{
    // To learn more about how to write unit tests, see the
    // JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/

    @Test
    void twoPlusTwoShouldEqualFour()
    {
        assertEquals(4, 2 + 2);
    }

    public static void main(String[] args) {
        System.out.println(new Rotation3d(Math.PI, -Math.PI/4, Math.PI));
    System.out.println(new Rotation3d(new Quaternion(-0.13, 0,.99, 0)).getY());
    }
}
