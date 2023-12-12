package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Constants {



    public static class AssemblyConstants {
        public static PIDFCoefficients slidesPIDConstants = new PIDFCoefficients(0.12, 0.08, 0, 0.05);
        public static PIDFCoefficients gantryPIDConstants = new PIDFCoefficients(0, 0, 0, 0);

        public static final double slideTickToInch = -29.16666666666667;
        public static final double gantryAngleToInch = 0.00;


        public static final double defaultSlideLength = 0;


    }

    public static final class DriveConstants {
        // for FTCDashboard visualization
        public static final double ROBOT_RADIUS = 5;
        // Localization Constants
        public static final double wheelbaseseparation = 12.8125*1.001297978861487; //13.37 //
        public static final double COUNTS_PER_INCH = 1752.875;
        public static final double horizontalTickOffset = -5.15;

        public static double X_MULT = 1.025016554017347;
        public static double Y_MULT = 0.99781777;

        public static final float DECIMATION_HIGH = 3;
        public static final float DECIMATION_LOW = 2;
        public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        public static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        public static final double FEET_PER_METER = 3.28084;


        // Lens intrinsics
        // UNITS ARE PIXELS
        public static final double fx = 822.317;
        public static final double fy = 822.317;
        public static final double cx = 319.495;
        public static final double cy = 242.502;

        // UNITS ARE METERS
        public static final double tagsize = 0.155;

        public static final double CAMERA_X_OFFSET = 6; // UNITS ARE INCHES
        public static final double CAMERA_Y_OFFSET = 6;
    }

    public static final class VisionConstants {
        public static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100, 180);
        public static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(300, 170);
        public static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(480, 170);
        public static final int REGION_WIDTH = 30;
        public static final int REGION_HEIGHT = 30;
        public static final int[] ACCEPTED_IDS = {1, 2, 3, 4, 5, 6};
    }

    public static final class FieldConstants {

        public static final Pose2d[] TAG_FIELD_POSITIONS = new Pose2d[] {new Pose2d(100, 600, 0), new Pose2d(500, 600, 0),
                new Pose2d(600, 500, 90), new Pose2d(600, 100, 90), new Pose2d(500, 0, 180)
        };
        public static final Pose2d[] TAG_BACKDROP_POSITIONS = new Pose2d[] {new Pose2d(100, 600, 0), new Pose2d(500, 600, 0),
                new Pose2d(600, 500, 90), new Pose2d(600, 100, 90), new Pose2d(500, 0, 180)
        };
    }


} // 1.58


/*                    ** FIELD COORDINATES **

    y+                         0°
    |-------------------------------------------------------|
    |                                                       |
    |                                                       |
    |                                                       |
    |                                                       |
    |                                                       |
    |                                                       |
270°|                                                       | 90°
    |                                                       |
    |                                                       |
    |                                                       |
    |                                                       |
    | (0,0)                                                 |
    |  /                                                    |
    | \/                                                    |
    |-------------------------------------------------------| x+
                              180°


 */