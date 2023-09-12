package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Constants.FieldConstants.TAG_POSITIONS;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class ATLocalizer {
    private int numFramesWithoutDetection = 0;
    private final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = DriveConstants.THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION;
    private final float DECIMATION_LOW = DriveConstants.DECIMATION_LOW;
    private final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = DriveConstants.THRESHOLD_HIGH_DECIMATION_RANGE_METERS;
    private final float DECIMATION_HIGH = DriveConstants.DECIMATION_HIGH;
    private static final double FEET_PER_METER = DriveConstants.FEET_PER_METER;
    private Pose2d position;

    public ATLocalizer() {
        //weeeeee
    }

    public void update(AprilTagDetectionPipeline pipe) {
        // look for detections
        ArrayList<AprilTagDetection> detections = pipe.getDetectionsUpdate();
        //if there are new camera frames
        if (detections != null) {
            //if we don't see a tag
            if (detections.size() == 0) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    pipe.setDecimation(DECIMATION_LOW);
                }
            } else {
                // We do see a tag!
                numFramesWithoutDetection = 0;
                // set the dectimation to match the distance from the tag if it is very large
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    pipe.setDecimation(DECIMATION_HIGH);
                }
                // for each detection seen (there will likely only be one, but even if we see multiple the result should be the same)
                for (AprilTagDetection detection : detections) {
                    // not sure which values correspond to which robot pos value

                    // Determines the actual global heading based the location of which tag we see
                    double heading = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle + TAG_POSITIONS[detection.id].getHeading();

                    double rx = detection.pose.x*(FEET_PER_METER*12);
                    double ry = detection.pose.z*(FEET_PER_METER*12);

                    double h = Math.hypot(rx, ry);
                    double relAngleToTarget = Math.atan2(rx, ry);
                    double ax = h*Math.sin(relAngleToTarget + TAG_POSITIONS[detection.id].getHeading());
                    double ay = h*Math.cos(relAngleToTarget + TAG_POSITIONS[detection.id].getHeading());
                    double finalX = TAG_POSITIONS[detection.id].getX() + ax;
                    double finalY = TAG_POSITIONS[detection.id].getY() - ay;

                    position = new Pose2d(finalX, finalY, heading);
                }
            }
        }

    }

    public Pose2d getEstimatePosition() {
        return position;
    }

}
