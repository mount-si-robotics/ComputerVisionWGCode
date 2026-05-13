package org.firstinspires.ftc.teamcode.atlas.atlasauto;


import org.firstinspires.ftc.teamcode.atlas.utils.BezierUtils;
import org.firstinspires.ftc.teamcode.atlas.utils.Node;
import org.firstinspires.ftc.teamcode.atlas.utils.Vector2;
import org.firstinspires.ftc.teamcode.atlas.utils.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Callable;

public class AtlasPathBuilder {
    private final ArrayList<Node> nodes = new ArrayList<>();
    private double rotation = 0;
    private final AtlasAutoOp opMode;
    double distancePerWaypoint;
    public AtlasPathBuilder(AtlasAutoOp opMode, double x, double y, double r, double d) {

        this.opMode = opMode;
        distancePerWaypoint = d;
        Node initial = new Node(x, y);
        initial.zeroHandles();
        rotation = r;
        nodes.add(initial);
    }

    public void perform() {
        if (nodes.size() < 2) return;
        debug("Starting performance");
        opMode.canSleep = false;

        opMode.telemetry.addLine(String.format("============= Atlas ============="));
        opMode.telemetry.addLine(String.format("Building waypoints from %d nodes", nodes.size()));
        opMode.telemetry.addLine(String.format("================================="));
        opMode.telemetry.update();
        long timer = System.currentTimeMillis();
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        Node first = nodes.get(0);
        Waypoint firstWaypoint = new Waypoint(first.x, first.y);
        firstWaypoint.onStart = first.runOnStart;
        firstWaypoint.onTick = first.runOnTick;
        firstWaypoint.blocking = first.runBlocking;

        double rotationState = firstWaypoint.r;

        waypoints.add(firstWaypoint);
        // Convert bezier path to list of waypoints for the follower to follow
        for (int i = 0; i < nodes.size() - 1; i++) {
            Node current = nodes.get(i);
            Node next = nodes.get(i + 1);
            Vector2 p0 = new Vector2(current.x, current.y);
            Vector2 p1 = new Vector2(current.h2x, current.h2y);
            Vector2 p3 = new Vector2(next.x, next.y);
            Vector2 p2 = new Vector2(next.h1x, next.h1y);
            AtlasPathBuilder.debug("Getting points from Bezier using", p0, p1, p2, p3, "and", distancePerWaypoint, "distance");
            ArrayList<Vector2> points = BezierUtils.sampleCubicBezier(p0, p1, p2, p3, distancePerWaypoint);
            AtlasPathBuilder.debug("Created a list of", points.size(), "waypoints and is now processing this");
            if (next.r != Double.MAX_VALUE) {
                rotationState = next.r;
            }

            boolean firstPoint = true;
            for (Vector2 point : points) {
                Waypoint waypoint = new Waypoint(point.x, point.y);
                if (firstPoint) {
                    firstPoint = false;
                    waypoint.onStart = current.runOnStart;
                }
                waypoint.onTick = current.runOnTick;
                waypoint.blocking = current.runBlocking;
                waypoint.moveSpeed = current.moveSpeed;
                waypoint.rotSpeed = current.rotSpeed;
                waypoint.r = rotationState;
                waypoints.add(waypoint);
            }
            if (next.stop) {
                waypoints.get(waypoints.size() - 1).stop = true;
            }
        }
        timer = System.currentTimeMillis() - timer;
        debug(String.format("Created path with %s waypoints", waypoints.size()));
        for (Waypoint waypoint : waypoints) {
            int index = waypoints.indexOf(waypoint);
            debug(String.format("%s: (%.2f, %.2f) r=%.2f", index, waypoint.x, waypoint.y, waypoint.r));
        }
        debug(String.format("From %s nodes", nodes.size()));
        for (Node node : nodes) {
            int index = nodes.indexOf(node);
            debug(String.format("%s: (%.2f, %.2f) r=%.2f", index, node.x, node.y, node.r));
        }
        opMode.follower.run(waypoints);
        opMode.canSleep = true;
    }

    private int getEnd() {
        return nodes.size() - 1;
    }

    public AtlasPathBuilder andMove(double x, double y) { return andMove(x, y, 1); }
    public AtlasPathBuilder andMove(double x, double y, double speed) {
        Node lastNode = nodes.get(getEnd());
        lastNode.x += x;
        lastNode.y += y;
        lastNode.moveSpeed = speed;
        lastNode.zeroHandles();
        return this;
    }
    public AtlasPathBuilder andMoveTo(double x, double y) { return andMoveTo(x, y, 1); }
    public AtlasPathBuilder andMoveTo(double x, double y, double speed) {
        Node lastNode = nodes.get(getEnd());
        lastNode.x = x;
        lastNode.y = y;
        lastNode.moveSpeed = speed;
        lastNode.zeroHandles();
        return this;
    }

    public AtlasPathBuilder andRotate(double degrees) { return andRotate(degrees, 1); }
    public AtlasPathBuilder andRotate(double degrees, double speed) {
        Node lastNode = nodes.get(getEnd());
        lastNode.r = rotation + degrees;
        rotation += degrees;
        lastNode.rotSpeed = speed;
        return this;
    }

    public AtlasPathBuilder andRotateTo(double degrees) { return andRotateTo(degrees, 1); }
    public AtlasPathBuilder andRotateTo(double degrees, double speed) {
        Node lastNode = nodes.get(getEnd());
        lastNode.r = degrees;
        rotation = degrees;
        lastNode.rotSpeed = speed;
        return this;
    }

    public AtlasPathBuilder andRun(Runnable func) {
        Node lastNode = nodes.get(getEnd());
        lastNode.runOnStart.add(func);

        return this;
    }

    public AtlasPathBuilder andRunTick(Runnable func) {
        Node lastNode = nodes.get(getEnd());
        lastNode.runOnStart.add(func);
        return this;
    }

    public AtlasPathBuilder andRunBlocking(Callable<Boolean> func) {
        Node lastNode = nodes.get(getEnd());
        lastNode.runBlocking.add(func);
        return this;
    }
    public AtlasPathBuilder andWaitForComplete() {
        Node lastNode = nodes.get(getEnd());
        lastNode.stop = true;
        return this;
    }

    public AtlasPathBuilder thenRun(Runnable func) {
        Node lastNode = nodes.get(getEnd());
        Node then = new Node(lastNode.x, lastNode.y);
        nodes.add(then);

        then.runOnStart.add(func);
        return this;
    }

    public AtlasPathBuilder thenMove(double x, double y) { return thenMove(x, y, 1); }
    public AtlasPathBuilder thenMove(double x, double y, double speed) {
        Node last = nodes.get(getEnd());
        nodes.add(new Node(last.x, last.y));
        return andMove(x, y, speed);
    }
    public AtlasPathBuilder thenMoveTo(double x, double y) { return thenMoveTo(x, y, 1); }
    public AtlasPathBuilder thenMoveTo(double x, double y, double speed) {
        nodes.add(new Node(0, 0));
        return andMoveTo(x, y, speed);
    }

    public AtlasPathBuilder thenRotate(double degrees) { return thenRotate(degrees, 1); }
    public AtlasPathBuilder thenRotate(double degrees, double speed) {
        Node lastNode = nodes.get(getEnd());
        Node then = new Node(lastNode.x, lastNode.y);
        then.r = rotation + degrees;
        rotation += degrees;
        then.rotSpeed = speed;

        nodes.add(then);
        return this;
    }

    public AtlasPathBuilder thenRotateTo(double degrees) { return thenRotateTo(degrees, 1); }
    public AtlasPathBuilder thenRotateTo(double degrees, double speed) {
        Node lastNode = nodes.get(getEnd());
        Node then = new Node(lastNode.x, lastNode.y);
        then.r = degrees;
        then.rotSpeed = speed;
        rotation = degrees;

        nodes.add(then);
        return this;
    }

    public AtlasPathBuilder thenBezierTo(double x, double y, double h1x, double h1y, double h2x, double h2y) {
        return thenBezierTo(x, y, h1x, h1y, h2x, h2y, 1.0);
    }
    public AtlasPathBuilder thenBezierTo(double x, double y, double h1x, double h1y, double h2x, double h2y, double speed) {
        Node lastNode = nodes.get(getEnd());
        lastNode.h2x = h1x;
        lastNode.h2y = h1y;
        lastNode.moveSpeed = speed;
        Node then = new Node(x, y);
        then.h1x = h2x;
        then.h1y = h2y;

        nodes.add(then);
        return this;
    }

    public AtlasPathBuilder thenSmoothMoveTo(double x, double y) { return thenSmoothMoveTo(x, y, 1.0); }
    public AtlasPathBuilder thenSmoothMoveTo(double x, double y, double speed) { return thenSmoothMoveTo(x, y, speed, 1/6); }
    public AtlasPathBuilder thenSmoothMoveTo(double x, double y, double speed, double tension) {
        if (nodes.size() < 2) throw new IllegalStateException("Cannot smoothmove, not enough nodes to auto handle!");
        Node left = nodes.get(getEnd() - 1);
        Node middle = nodes.get(getEnd());
        Node right = new Node(x, y);
        nodes.add(right);

        double tx = (right.x - left.x) * tension;
        double ty = (right.y - left.y) * tension;
        middle.h1x = middle.x - tx;
        middle.h1y = middle.y - ty;
        middle.h2x = middle.x + tx;
        middle.h2y = middle.y + ty;
        return this;
    }

    public static void debug(Object... args) {
        String output = Arrays.stream(args).reduce((a, b) -> a + " " + b.toString()).get().toString();
        System.out.println("[AtlasPathBuilder] " + output);
    }
}