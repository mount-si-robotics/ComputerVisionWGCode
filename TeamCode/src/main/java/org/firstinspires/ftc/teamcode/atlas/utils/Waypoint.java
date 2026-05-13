package org.firstinspires.ftc.teamcode.atlas.utils;

import java.util.ArrayList;
import java.util.concurrent.Callable;

public class Waypoint {
    public double x;
    public double y;
    public double r;
    public double moveSpeed = 1;
    public double rotSpeed = 1;
    public boolean stop = false;
    public ArrayList<Runnable> onStart = new ArrayList<>();
    public ArrayList<Runnable> onTick = new ArrayList<>();
    public ArrayList<Callable<Boolean>> blocking = new ArrayList<>();

    public Waypoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
}