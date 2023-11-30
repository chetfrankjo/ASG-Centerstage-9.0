package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Constants {



    public static class AutoPaths {

        /*public static Trajectory approach_1 = new Trajectory(132, 84,0.4, 25).addPoint(120, 84, 0).addPoint(108, 72, 0).build();
        public static Trajectory backup = new Trajectory(108, 72, 0.4, 25).addPoint(120, 84, 180).build();
        public static Trajectory approach_2 = new Trajectory(120, 84, 0.4, 25).addPoint(108, 84, 0).addPoint(108, 120, 0).build();
        public static Trajectory park = new Trajectory(108, 120, 0.4, 25).addPoint(84, 120, 90).addPoint(84, 132, 0).build();

         */
        public static ArrayList<Trajectory> generateAutoPaths(General.ParkLocation parkLocation, General.SpikePosition spikePosition, General.AllianceLocation allianceLocation) {
            ArrayList<Trajectory> paths = new ArrayList<>();
            switch (spikePosition) {

                case LEFT:
                    switch (allianceLocation) {
                        case RED_SOUTH:
                            paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 20, 0).addPoint(-6, 29, 0).build());
                            paths.add(new Trajectory(-5, 29, 0.4, 5).addPoint(0, 4, 180).build());
                            paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(53, 4, -90).addPoint(60, 43, 0).addPoint(72, 43, 0).build());
                            switch (parkLocation) {

                                case LEFT:
                                    paths.add(new Trajectory(72, 43, 0.4, 10).addPoint(65, 43, 180).addPoint(65, 59, 90).build());
                                    paths.add(new Trajectory(65, 59, 0.3, 20).addPoint(87, 59, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(70, 44, 0.4, 10).addPoint(65, 44, 180).addPoint(65, 8, -90).build());
                                    paths.add(new Trajectory(65, 8, 0.3, 20).addPoint(87, 8, 0).build());
                                    break;
                            }
                            break;
                        case RED_NORTH:
                            paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 23, 0).addPoint(-6, 29, 0).build());
                            paths.add(new Trajectory(-5, 29, 0.6, 5).addPoint(0, 18, 180).build());
                            paths.add(new Trajectory(0, 12, 0.4, 15).addPoint(0, 30, 0).addPoint(12, 44, 0).addPoint(25, 44, 0).build());
                            switch (parkLocation) {

                                case LEFT:
                                    paths.add(new Trajectory(25, 44, 0.4, 10).addPoint(20, 44, 180).addPoint(20, 59, 90).build());
                                    paths.add(new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(25, 44, 0.4, 10).addPoint(20, 44, 180).addPoint(20, 8, -90).build());
                                    paths.add(new Trajectory(20, 8, 0.3, 20).addPoint(42, 8, 0).build());
                                    break;
                            }
                            break;
                        case BLUE_SOUTH:
                            paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 23, 0).addPoint(-4, 28, 0).build());
                            paths.add(new Trajectory(0, 28, 0.3, 5).addPoint(0, 4, 180).build());
                            paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(-55, 4, 90).addPoint(-62, 21, 0).addPoint(-72, 21, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(-72, 21, 0.4, 10).addPoint(-65, 21, 180).addPoint(-65, 5, 90).build());
                                    paths.add(new Trajectory(-65, 5, 0.3, 20).addPoint(-87, 5, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(-72, 27, 0.4, 10).addPoint(-65, 27, 180).addPoint(-65, 56, -90).build());
                                    paths.add(new Trajectory(-65, 56, 0.3, 20).addPoint(-87, 56, 0).build());
                                    break;
                            }
                            break;
                        case BLUE_NORTH:
                            paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 20, 0).addPoint(-6, 29, 0).build());
                            paths.add(new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build());
                            paths.add(new Trajectory(0, 22, 0.4, 15).addPoint(-12, 22, 90).addPoint(-12, 27, 0).addPoint(-25, 27, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(-25, 27, 0.4, 10).addPoint(-20, 27, 180).addPoint(-20, 8, 90).build());
                                    paths.add(new Trajectory(-20, 8, 0.3, 20).addPoint(-42, 8, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(-25, 27, 0.4, 10).addPoint(-20, 27, 180).addPoint(-20, 56, -90).build());
                                    paths.add(new Trajectory(-20, 56, 0.3, 20).addPoint(-42, 56, 0).build());
                                    break;
                            }
                            break;
                    }
                    break;
                case CENTER:
                    switch (allianceLocation) {
                        case RED_SOUTH:
                            paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                            paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 4, 180).build());
                            paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(55, 4, -90).addPoint(60, 32, 0).addPoint(72, 32, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(72, 34, 0.4, 10).addPoint(65, 34, 180).addPoint(65, 59, 90).build());
                                    paths.add(new Trajectory(65, 59, 0.3, 10).addPoint(87, 59, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(72, 34, 0.4, 10).addPoint(65, 34, 180).addPoint(65, 8, -90).build());
                                    paths.add(new Trajectory(65, 8, 0.3, 10).addPoint(87, 8, 0).build());
                                    break;
                            }
                            break;
                        case RED_NORTH:
                            paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                            paths.add(new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build());
                            paths.add(new Trajectory(0, 10, 0.4, 15).addPoint(0, 34, 0).addPoint(25, 34, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(25, 34, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 59, 90).build());
                                    paths.add(new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(25, 34, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 8, -90).build());
                                    paths.add(new Trajectory(20, 8, 0.3, 20).addPoint(42, 8, 0).build());
                                    break;
                            }
                            break;
                        case BLUE_SOUTH:
                            paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                            paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 4, 180).build());
                            paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(-55, 4, 90).addPoint(-63, 30, 0).addPoint(-72, 30, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(-72, 30, 0.4, 10).addPoint(-65, 30, 180).addPoint(-65, 5, 90).build());
                                    paths.add(new Trajectory(-65, 5, 0.3, 10).addPoint(-87, 5, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(-72, 34, 0.4, 10).addPoint(-65, 34, 180).addPoint(-65, 56, -90).build());
                                    paths.add(new Trajectory(-65, 56, 0.3, 10).addPoint(-87, 56, 0).build());
                                    break;
                            }
                            break;
                        case BLUE_NORTH:
                            paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                            paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 22, 180).build());
                            paths.add(new Trajectory(0, 10, 0.4, 15).addPoint(0, 34, 0).addPoint(-25, 34, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(-25, 34, 0.4, 10).addPoint(-20, 34, -180).addPoint(-20, 5, 90).build());
                                    paths.add(new Trajectory(-20, 5, 0.3, 20).addPoint(-42, 5, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(-25, 34, 0.4, 10).addPoint(-20, 34, 180).addPoint(-20, 56, -90).build());
                                    paths.add(new Trajectory(-20, 56, 0.3, 20).addPoint(-42, 56, 0).build());
                                    break;
                            }
                            break;
                    }
                    break;
                case RIGHT:
                    switch (allianceLocation) {
                        case RED_SOUTH:
                            paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 23, 0).addPoint(4, 28, 0).build());
                            paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 4, 180).build());
                            paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(53, 4, -90).addPoint(60, 27, 0).addPoint(72, 27, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(72, 30, 0.4, 20).addPoint(65, 34, -180).addPoint(65, 59, 90).build());
                                    paths.add(new Trajectory(65, 59, 0.3, 20).addPoint(87, 59, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(72, 30, 0.4, 10).addPoint(65, 34, 180).addPoint(65, 8, -90).build());
                                    paths.add(new Trajectory(65, 8, 0.3, 20).addPoint(87, 8, 0).build());
                                    break;
                            }
                            break;
                        case RED_NORTH:
                            paths.add(new Trajectory(0, 0,0.3, 10).addPoint(0, 10, 0).addPoint(6, 28, 0).build());
                            paths.add(new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build());
                            paths.add(new Trajectory(0, 22, 0.4, 15).addPoint(12, 22, -90).addPoint(12, 30, 0).addPoint(25, 30, 0).build());
                            switch (parkLocation) {
                                case LEFT:
                                    paths.add(new Trajectory(25, 30, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 59, 90).build());
                                    paths.add(new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(25, 30, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 8, -90).build());
                                    paths.add(new Trajectory(20, 8, 0.3, 20).addPoint(42, 8, 0).build());
                                    break;
                            }
                            break;
                        case BLUE_SOUTH:
                            paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 20, 0).addPoint(6, 29, 0).build());
                            paths.add(new Trajectory(5, 29, 0.4, 5).addPoint(0, 4, 180).build());
                            paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(-55, 4, 90).addPoint(-63, 38, 0).addPoint(-72, 38, 0).build());
                            switch (parkLocation) {

                                case LEFT:
                                    paths.add(new Trajectory(-70, 38, 0.4, 10).addPoint(-65, 38, -180).addPoint(-65, 5, 90).build());
                                    paths.add(new Trajectory(-65, 5, 0.3, 20).addPoint(-87, 5, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(-72, 38, 0.4, 10).addPoint(-65, 38, 180).addPoint(-65, 56, -90).build());
                                    paths.add(new Trajectory(-65, 56, 0.3, 20).addPoint(-87, 56, 0).build());
                                    break;
                            }
                            break;
                        case BLUE_NORTH:
                            paths.add(new Trajectory(0, 0,0.3, 10).addPoint(0, 20, 0).addPoint(5, 28, 0).build());
                            paths.add(new Trajectory(5, 28, 0.3, 5).addPoint(0, 22, 180).build());
                            paths.add(new Trajectory(0, 12, 0.4, 15).addPoint(0, 30, 0).addPoint(-12, 41, 0).addPoint(-25, 41, 0).build());
                            switch (parkLocation) {

                                case LEFT:
                                    paths.add(new Trajectory(-25, 44, 0.4, 10).addPoint(-20, 44, 180).addPoint(-20, 8, 90).build());
                                    paths.add(new Trajectory(-20, 8, 0.3, 20).addPoint(-42, 8, 0).build());
                                    break;
                                case RIGHT:
                                    paths.add(new Trajectory(-25, 44, 0.4, 10).addPoint(-20, 44, 180).addPoint(-20, 56, -90).build());
                                    paths.add(new Trajectory(-20, 56, 0.3, 20).addPoint(-42, 56, 0).build());
                                    break;
                            }
                            break;
                    }
                    break;

            }

        return paths;
        }


        public static Trajectory approach_1 = new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build();
        public static Trajectory approach_1_2 = new Trajectory(0, 0,0.3, 10).addPoint(0, 10, 0).addPoint(6, 28, 0).build();

        public static Trajectory approach_1_1 = new Trajectory(0, 0,0.3, 3).addPoint(0, 22, 0).addPointTurn(-4, 27, 0, 0.05).build();
        public static Trajectory backup = new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build();
        public static Trajectory approach_2 = new Trajectory(0, 10, 0.4, 15).addPoint(0, 34, 0).addPoint(25, 34, 0).build();
        public static Trajectory approach_2_2 = new Trajectory(0, 22, 0.4, 15).addPoint(12, 22, 0).addPoint(12, 34, 0).addPoint(25, 34, 0).build();
        public static Trajectory park_1 = new Trajectory(25, 34, 0.4, 20).addPoint(20, 34, 0).addPoint(20, 59, 90).build();
        public static Trajectory park_2 = new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build();

    }



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
        public static final double wheelbaseseparation = 13.72449; //13.37
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
        public static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70, 160);
        public static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(300, 150);
        public static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(560, 150);
        public static final int REGION_WIDTH = 50;
        public static final int REGION_HEIGHT = 50;
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