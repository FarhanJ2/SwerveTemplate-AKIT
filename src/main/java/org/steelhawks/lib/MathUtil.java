package org.steelhawks.lib;

public class MathUtil {
    public static double continuous180To360(double angle) {
        return (angle + 360) % 360;
    }
}
