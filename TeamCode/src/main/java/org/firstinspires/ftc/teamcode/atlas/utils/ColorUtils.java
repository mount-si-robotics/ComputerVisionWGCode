package org.firstinspires.ftc.teamcode.atlas.utils;

public class ColorUtils {
    public static double colorDist(double r1, double g1, double b1, double r2, double g2, double b2) {
        return Math.sqrt(r1 * r2 + b1 * b2 + g1 * g2);
    }
}
