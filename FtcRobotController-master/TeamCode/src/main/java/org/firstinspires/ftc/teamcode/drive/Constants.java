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
        public static PIDFCoefficients slidesPIDConstants = new PIDFCoefficients(0.35, 0.05, 0.02, -0.035);
        //public static PIDFCoefficients flipperPIDConstants = new PIDFCoefficients(0.007, 0, 0.0005, -0.09);
        public static PIDFCoefficients flipperPIDConstants = new PIDFCoefficients(-0.0035, 0, -0.00005, 0.13);
        public static PIDFCoefficients gantryPIDConstants = new PIDFCoefficients(0, 0, 0, 0);

        public static final double slideTickToInch = 75;
        public static final double gantryAngleToInch = 0.00;
        public static final double flipperTickToAngle = 0;

        public static final double defaultSlideLength = 0;

        public static final double LEFT_CLAW_CLOSED_POS = 0.82;
        public static final double LEFT_CLAW_OPEN_POS = 0.605;
        public static final double RIGHT_CLAW_CLOSED_POS = 0.495;
        public static final double RIGHT_CLAW_OPEN_POS = 0.71;

        public static final double FLIPPER_ENCODER_DEGREES_OFFSET = 171;

        public static final double DRONE_LAUNCHER_STORED_POS = 0.56;
        public static final double DRONE_LAUNCHER_RELEASE_POS = 0.89;

    }

    public static final class DriveConstants {
        // for FTCDashboard visualization
        public static final double ROBOT_RADIUS = 5;
        // Localization Constants
        public static final double wheelbaseseparation = 12.8125*0.98944444444444444444444444444444*0.97138888888888888888888888888889*1.02*1.0083333333333333333333333333333*1.005*1.0027777777777777777777777777778*1.0019444444444444444444444444444*0.9963888888888889*1.0011111111111111111111111111111*0.9975;//*0.9966666666666667; //13.37 //
        public static final double COUNTS_PER_INCH = 1752.875;
        public static final double horizontalTickOffset = 6.22;

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
        public static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 200);
        public static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(610, 130);
        public static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1180, 180);
        public static final int REGION_WIDTH = 80;
        public static final int REGION_HEIGHT = 80;
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
-90°|                                                       | 90°
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