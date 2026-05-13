package org.firstinspires.ftc.teamcode.atlas.utils;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.concurrent.Callable;

public class Node {
    public double x, y;
    public double moveSpeed = 1;
    public double rotSpeed = 1;
    public boolean stop = false;
    public double r = Double.MAX_VALUE;

    public double h1x, h1y, h2x, h2y = Double.MAX_VALUE;

    public ArrayList<Runnable> runOnStart = new ArrayList<>();
    public ArrayList<Runnable> runOnTick = new ArrayList<>();
    public ArrayList<Callable<Boolean>> runBlocking = new ArrayList<>();

    public Node(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void zeroHandles() {
        h1x = x;
        h2x = x;
        h1y = y;
        h2y = y;
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("<Node (%.2f, %.2f) with handles h1=(%.2f, %.2f), h2=(%.2f, %.2f)>", x, y, h1x, h1y, h2x, h2y);
    }
}
