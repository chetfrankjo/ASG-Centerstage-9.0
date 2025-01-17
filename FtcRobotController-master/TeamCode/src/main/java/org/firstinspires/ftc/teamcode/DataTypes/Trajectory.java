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
    public Trajectory(double startX, double startY, double movespeed, double followDistance, double turnSpeed) {
        path = new ArrayList<>();
        path.add(new CurvePoint(startX, startY, movespeed, turnSpeed, Math.toRadians(0), followDistance, Math.toRadians(50), 1.0, false, 0));
        this.movespeed = movespeed;
        this.followDistance = followDistance;

    }

    /**
     *
     * @param x
     * @param y
     * @param targetAngle Degrees relative to line
     */
    public Trajectory addPoint(double x, double y, double targetAngle) {
        path.add(new CurvePoint(x, y, movespeed, 0.4, Math.toRadians(targetAngle), followDistance, Math.toRadians(50), 0.5, false, 0));
        return this;
    }

    public Trajectory addPointTurn(double x, double y, double targetAngle, double turnSpeed) {
        path.add(new CurvePoint(x, y, movespeed, turnSpeed, Math.toRadians(targetAngle), followDistance, Math.toRadians(50), 0.5, false, 0));
        return this;
    }

    public Trajectory addPointSpeed(double x, double y, double targetAngle, double movespeed) {
        path.add(new CurvePoint(x, y, movespeed, 0.4, Math.toRadians(targetAngle), followDistance, Math.toRadians(50), 0.5, false, 0));
        return this;
    }
    public Trajectory addPointFollow(double x, double y, double targetAngle, double followDistance) {
        path.add(new CurvePoint(x, y, movespeed, 0.4, Math.toRadians(targetAngle), followDistance, Math.toRadians(50), 0.5, false, 0));
        return this;
    }

    public void addAdvanced(CurvePoint point) {
        path.add(point);
    }

    public Trajectory build() {
        int size = path.size();

        //Make the last point the last point
        path.get(size-1).setEnd(true);

        // Calculate slope for extended virtual line (prevents drifting at end)
        Point p1 = new Point(path.get(size-2).x, path.get(size-2).y);
        Point p2 = new Point(path.get(size-1).x, path.get(size-1).y);
        double run = Range.clip(p2.x - p1.x, -followDistance-30, followDistance + 30);
        double rise = Range.clip(p2.y - p1.y, -followDistance-30, followDistance + 30);
        Point newpoint = new Point(p2.x + run*100, p2.y + rise*100);
        path.add(new CurvePoint(newpoint.x, newpoint.y, 1.0, 0.7, path.get(size-1).targetAngle, followDistance, Math.toRadians(50), 0.5, true, 0));
        return this;
    }

}
