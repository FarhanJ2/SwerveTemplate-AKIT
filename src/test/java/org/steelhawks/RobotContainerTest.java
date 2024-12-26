package org.steelhawks;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.fail;

public class RobotContainerTest {

    @Test
    public void createRobotContainer() {
        try {
            new RobotContainer();
        } catch (Exception e) {
            e.printStackTrace();
            fail("RobotContainer creation failed");
        }
    }
}
