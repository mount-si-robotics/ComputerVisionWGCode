package org.firstinspires.ftc.teamcode.atlas.utils;

public class Vector2 {
    public static final double DEG_TO_RAD = Math.PI / 180d;
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2() {
        this(0, 0);
    }

    public void normalize() {
        Vector2 out = Vector2.normalize(this);
        this.x = out.x;
        this.y = out.y;
    }
    public Vector2 normalized() {
        return Vector2.normalize(this);
    }

    public double magnitude() {
        return Vector2.magnitude(this);
    }

    public double distance(Vector2 other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static Vector2 normalize(Vector2 vector) {
        double magnitude = Vector2.magnitude(vector);
        return new Vector2(vector.x / magnitude, vector.y / magnitude);
    }
    public static double magnitude(Vector2 vector) {
        return Math.sqrt(vector.x * vector.x + vector.y * vector.y);
    }

    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }

    public void rotate(double degrees) {
        rotateRads(degrees * DEG_TO_RAD);
    }
    public void rotateRads(double radians) {
        double cs = Math.cos(radians);
        double sn = Math.sin(radians);
        x = x * cs - y * sn;
        y = x * sn + y * cs;
    }
}