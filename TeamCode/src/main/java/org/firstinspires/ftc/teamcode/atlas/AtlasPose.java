package org.firstinspires.ftc.teamcode.atlas;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class AtlasPose {
    private final double metersPerTick;

    public static final long PAST_STATE_HOLD_TIME = 2000; // ms

    public double x = 0.0;
    public double y = 0.0;


    private final List<PastState> pastStates = new ArrayList<>();

    public AtlasPose(double metersPerTick) {
        this.metersPerTick = metersPerTick;
    }

    public void updateEncoders(int deltaFrontLeft, int deltaFrontRight,
                               int deltaBackLeft, int deltaBackRight,
                               double yawRads) {
        double dxLocal = (deltaFrontLeft + deltaFrontRight + deltaBackLeft + deltaBackRight) / 4.0;
        double dyLocal = (-deltaFrontLeft + deltaFrontRight + deltaBackLeft - deltaBackRight) / 4.0;

        double dx = sin(yawRads) * dxLocal + cos(yawRads) * dyLocal;
        double dy = cos(yawRads) * dxLocal - sin(yawRads) * dyLocal;

        x -= dx * metersPerTick;
        y -= dy * metersPerTick;

        long time = System.currentTimeMillis();
        pastStates.add(new PastState(time, dx, dy, x, y));
        while (!pastStates.isEmpty() &&
                time - pastStates.get(0).time > PAST_STATE_HOLD_TIME) {
            pastStates.remove(0);
        }
    }

    public void updateTruePosition(double x, double y, double time, double confidence) {
        confidence = Range.clip(confidence, -1, 1);
        double inverseConfidence = 1 - confidence;
        this.x = this.x * inverseConfidence + x * confidence;
        this.y = this.y * inverseConfidence + y * confidence;

        for (int i = pastStates.size() - 1; i >= 0; i--) {
            PastState state = pastStates.get(i);
            if (state.time > time) {
                x -= state.dx * metersPerTick;
                y += state.dy * metersPerTick;
            }
        }
    }

    public void updateTruePosition(double x, double y, double time) {
        updateTruePosition(x, y, time, 1);
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    public String toString() {
        return String.format("<AtlasPose %.2f, %.2f>", x, y);
    }

    public static class PastState {
        public final long time;
        public final double dx;
        public final double dy;
        public final double x;
        public final double y;

        public PastState(long time, double dx, double dy, double x, double y) {
            this.time = time;
            this.dx = dx;
            this.dy = dy;
            this.x = x;
            this.y = y;
        }
    }
}
