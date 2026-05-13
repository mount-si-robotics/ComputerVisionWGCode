package org.firstinspires.ftc.teamcode.atlas.utils;

import java.util.ArrayList;

public class BezierUtils {
    public static final int SAMPLE_RESOLUTION = 50;
    public static ArrayList<Vector2> sampleCubicBezier(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, double spacing) {
        ArrayList<Vector2> result = new ArrayList<>();
        // Because bezier curves are dumb, the t value we plug into them isn't a perfect percentage.
        // We need to create a bunch of sample points and then interpolate to create evenly spaced points
        Vector2[] points = new Vector2[SAMPLE_RESOLUTION + 1];
        double[] lengths = new double[SAMPLE_RESOLUTION + 1];

        for (int i = 0; i <= SAMPLE_RESOLUTION; i++){
            double t = (double) i/SAMPLE_RESOLUTION;
            points[i] = getPointOnCubic(t, p0, p1, p2, p3);
        }
        for(int i = 1; i < points.length; i++){
            lengths[i] = lengths[i-1] + points[i].distance(points[i-1]);
        }
        double totalLength = lengths[lengths.length - 1];

        // Actual bezier curve point calculations here
        int count = (int)(totalLength / spacing) + 1;
        for(int i = 0; i < count; i++){
            double target = Math.min(totalLength, i * spacing);
            double t = solveT(lengths, target, points);
            result.add(getPointOnCubic(t, p0, p1, p2, p3));
        }
        return result;
    }

    private static double solveT(double[] lengths, double target, Vector2[] points) {
        double t;
        int searcher = 0;

        while (searcher < lengths.length && lengths[searcher] < target) searcher++;
        if (searcher == 0) t = 0;
        else if (searcher >= lengths.length) t = 1;
        else {
            // lerp
            double l0 = lengths[searcher - 1];
            double l1 = lengths[searcher];
            double ratio = (target - l0) / (l1 - l0);
            double t0 = (double) (searcher - 1) / (points.length - 1);
            double t1 = (double) (searcher) / (points.length - 1);
            t = t0 + (t1 - t0) * ratio;
        }
        return t;
    }

    // omg cubic bezier curves so cool so wow
    public static Vector2 getPointOnCubic(double t, Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3) {
        double u = 1 - t;
        double b0 = u * u * u;
        double b1 = 3 * u * u * t;
        double b2 = 3 * u * t * t;
        double b3 = t * t * t;
        return new Vector2(
            b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x,
            b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y
        );
    }
}