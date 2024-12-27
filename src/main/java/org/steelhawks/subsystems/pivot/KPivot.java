package org.steelhawks.subsystems.pivot;

public final class KPivot {

    public static final int PIVOT_ID = 18;
    public static final int PIVOT_ENCODER_ID = 19;

    public static final double PIVOT_KS = 0.09256;
    public static final double PIVOT_KG = 0.15116;
    public static final double PIVOT_KV = 1.593;

    public static final double PIVOT_KP = 5;
    public static final double PIVOT_KI = 0;
    public static final double PIVOT_KD = 0;

    public static final double PIVOT_TOLERANCE = 0.01 * 1.5;
    public static final double PIVOT_MAX_VELOCITY = 6;
    public static final double PIVOT_MAX_ACCELERATION = 8;
    public static final double INCREMENT_VOLTAGE = 0.75;
    public static final double DEADBAND = 0.3;
}
