package org.firstinspires.ftc.teamcode.atlas.atlasauto;
import org.firstinspires.ftc.teamcode.atlas.AtlasChassis;

public class AtlasParameters {
    public AtlasChassis chassis;
    public double tolerance;
    public double stopTolerance;

    public double distancePerWaypoint;
    public AtlasParameters(AtlasChassis chassis, double tolerance, double stopTolerance, double distancePerWaypoint) {
        this.chassis = chassis;
        this.tolerance = tolerance;
        this.stopTolerance = stopTolerance;
        this.distancePerWaypoint = distancePerWaypoint;
    }
}
