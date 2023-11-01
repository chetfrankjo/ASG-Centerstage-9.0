package org.firstinspires.ftc.teamcode.DataTypes;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

public class Trajectory {
    public ArrayList<CurvePoint> path;
    private double movespeed;
    private double followDistance;
    public Trajectory(double startX, double startY, double movespeed, double followDistance) {
        path = new ArrayList<>();
        path.add(new CurvePoint(startX, startY, movespeed, 0.7, Math.toRadians(0), followDistance, Math.toRadians(50), 1.0, false, 0));
        this.movespeed = movespeed;
        this.followDistance = followDistance;
    }

    /**
     *
     * @param x
     * @param y
     * @param targetAngle Degrees relative to line
     */
    public void addPoint(double x, double y, double targetAngle) {
        path.add(new CurvePoint(x, y, movespeed, 0.7, Math.toRadians(targetAngle), followDistance, Math.toRadians(50), 0.5, false, 0));
    }

    public void addAdvanced(CurvePoint point) {
        path.add(point);
    }

    public void build() {
        int size = path.size();

        //Make the last point the last point
        path.get(size-1).setEnd(true);

        // Calculate slope for extended virtual line (prevents drifting at end)
        Point p1 = new Point(path.get(size-2).x, path.get(size-2).y);
        Point p2 = new Point(path.get(size-1).x, path.get(size-1).y);
        double run = Range.clip(p2.x - p1.x, -followDistance-20, followDistance + 20);
        double rise = Range.clip(p2.y - p1.y, -followDistance-20, followDistance + 20);
        Point newpoint = new Point(p2.x + run, p2.y + rise);
        path.add(new CurvePoint(newpoint.x, newpoint.y, 1.0, 0.7, path.get(size-1).targetAngle, followDistance, Math.toRadians(50), 0.5, true, 0));
    }

}
