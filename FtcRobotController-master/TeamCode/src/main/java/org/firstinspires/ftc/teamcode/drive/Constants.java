package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {

    public static class AssemblyConstants {
        public static PIDFCoefficients turretPIDConstants = new PIDFCoefficients(0, 0, 0, 0);
        public static PIDFCoefficients slidesPIDConstants = new PIDFCoefficients(0, 0, 0, 0);
        public static PIDFCoefficients v4barPIDConstants = new PIDFCoefficients(0, 0, 0, 0);

        public static final double slideTickToInch = 58.3;
        public static final double vbarTickToInch = 8.055556;
        public static final double zeroV4barAngle = -45;
        public static final double v4barLength = 12;
    }

    public static final class DriveConstants {
        // for FTCDashboard visualization
        public static final double ROBOT_RADIUS = 5;
        // Localization Constants
        public static final double wheelbaseseparation = 7.6616107;
        public static final double COUNTS_PER_INCH = 1752.875;
        public static final double horizontalTickOffset = 7.75; //TODO:  RR says 5

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

        public static final class Drivetrain {
            public static final int MECANUM = 1;
            public static final int SIXWD = 2;
            public static final int SWERVE = 3;
        }
    }

    public static final class FieldConstants {
        @Deprecated
        public static final double[] relTagAngles = new double[] {0, 0, 90, 90, 180, 180, -90, -90};

        public static final Pose2d[] TAG_POSITIONS = new Pose2d[] {new Pose2d(100, 600, 0), new Pose2d(500, 600, 0),
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