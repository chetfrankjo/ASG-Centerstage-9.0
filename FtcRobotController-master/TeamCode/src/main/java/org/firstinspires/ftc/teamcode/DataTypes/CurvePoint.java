package org.firstinspires.ftc.teamcode.DataTypes;

import org.firstinspires.ftc.teamcode.DataTypes.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;
    public double targetAngle;
    public boolean end;
    public double slope;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double targetAngle, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount, boolean end, double slope
    ) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.targetAngle = targetAngle;
        this.end = end;
        this.slope = slope;
    }
    public CurvePoint(CurvePoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;;
        turnSpeed = thisPoint.turnSpeed;;
        followDistance = thisPoint.followDistance;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        pointLength = thisPoint.pointLength;
        targetAngle = thisPoint.targetAngle;
        slope = thisPoint.slope;
    }
    public Point toPoint() {
        return new Point(x, y);
    }

    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
    }

    public void setEnd(boolean end) {
        this.end = end;
    }
}
