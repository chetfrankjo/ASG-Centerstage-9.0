package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.Constants.DriveConstants;


public class Localizer {
    public int[] encoders, prevencoders;

    private final double COUNTS_PER_INCH = DriveConstants.COUNTS_PER_INCH;

    double dtheta = 0;
    private double x = 0, y = 0, angle = 0;
    public double startXOffset, startYOffset, startAngleOffset;
    double loopspeed = 0;
    double deltaX;
    double deltaY;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    private double leftChange = 0;
    private double rightChange = 0;
    double rawHorizontalChange = 0;

    Pose2d robotpos = new Pose2d(0, 0, 0);

    public Localizer() {
        robotEncoderWheelDistance = DriveConstants.wheelbaseseparation * COUNTS_PER_INCH; //
        this.horizontalEncoderTickPerDegreeOffset = DriveConstants.horizontalTickOffset;
        encoders = new int[3];
        prevencoders = new int[3];
    }
    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i] = encoders[i];
        }
    }

    public void update(double loopspeed){
        this.loopspeed=loopspeed;
        leftChange = encoders[0] - prevencoders[0];
        rightChange = encoders[1] - prevencoders[1];

        //Calculate Angle
        dtheta = (leftChange - rightChange) / (robotEncoderWheelDistance);
        angle = ((angle + dtheta));

        //Get the components of the motion
        rawHorizontalChange = encoders[2] - prevencoders[2];
        double horizontalChange = rawHorizontalChange - (dtheta*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;
        deltaX = (p*Math.sin(angle) + n*Math.cos(angle));
        deltaY = (p*Math.cos(angle) - n*Math.sin(angle));

        //Calculate and update the position values
        x = x + deltaX;
        y = y + deltaY;
        robotpos = new Pose2d((x/COUNTS_PER_INCH)+startXOffset, (y/COUNTS_PER_INCH)+startYOffset, Math.toDegrees(angle)+startAngleOffset);
        for (int i=0; i<prevencoders.length; i++) {
            prevencoders[i] = encoders[i];
        }

    }

    public double returnRawX(){ return x; }
    public double returnRawY(){ return y; }
    public double returnOrientation(){ return Math.toDegrees(angle) % 360; }
    public double xCoordinate() {return x / COUNTS_PER_INCH;}
    public double yCoordinate() {return y /COUNTS_PER_INCH;}
    public double angle() {return Math.toDegrees(angle) % 360;}
    public double wheelDistance() {return robotEncoderWheelDistance/COUNTS_PER_INCH;}
    public double tickOffset() {return horizontalEncoderTickPerDegreeOffset;}
    public double leftchange() {return leftChange;}
    public double rightchange() {return rightChange;}
    public Pose2d getPosEstimate() {
        return robotpos;
    }

    public Pose2d getvelocity() {
        return new Pose2d(((leftChange/COUNTS_PER_INCH)/loopspeed), ((rightChange/COUNTS_PER_INCH)/loopspeed), ((rawHorizontalChange/COUNTS_PER_INCH)/loopspeed));
    }

    /**
     *
     * @param x
     * @param y
     * @param angle degress, not radians
     */
    public void setEstimatePos(double x, double y, double angle) {
        startXOffset = x;
        startYOffset = y;
        startAngleOffset = angle;
    }

    public void resetPosWithEstimate(Pose2d pos) {
        startXOffset = pos.getX();
        startYOffset = pos.getY();
        startAngleOffset = pos.getHeading();;
        x = 0;
        y = 0;
        angle = 0;
    }
}
