package org.firstinspires.ftc.teamcode.vision;


import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.DataTypes.General.SpikePosition;
import org.firstinspires.ftc.teamcode.drive.Constants.VisionConstants;

public class ThreeZonePropDetectionPipeline extends OpenCvPipeline {
    boolean overlay;
    Point REGION1_TOPLEFT_ANCHOR_POINT, REGION2_TOPLEFT_ANCHOR_POINT, REGION3_TOPLEFT_ANCHOR_POINT, region1_pointA, region1_pointB, region2_pointA, region2_pointB, region3_pointA, region3_pointB;

    General.AllianceLocation color = General.AllianceLocation.RED_NORTH;
    public ThreeZonePropDetectionPipeline(boolean enableOverlay, General.AllianceLocation color) {
        overlay = enableOverlay;
        this.color=color;

        if (color == General.AllianceLocation.RED_NORTH | color == General.AllianceLocation.RED_SOUTH) {
            REGION1_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION1_TOPLEFT_ANCHOR_POINT_RED;
            REGION2_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION2_TOPLEFT_ANCHOR_POINT_RED;
            REGION3_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION3_TOPLEFT_ANCHOR_POINT_RED;
        } else {
            REGION1_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION1_TOPLEFT_ANCHOR_POINT_BLUE;
            REGION2_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION2_TOPLEFT_ANCHOR_POINT_BLUE;
            REGION3_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION3_TOPLEFT_ANCHOR_POINT_BLUE;
        }

        region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);




    /*
     * The core values which define the location and size of the sample regions
     */
    static final int REGION_WIDTH = VisionConstants.REGION_WIDTH;
    static final int REGION_HEIGHT = VisionConstants.REGION_HEIGHT;
    
    /*
     * Working variables
     */
    Mat region1_Cb, region1_green, region2_Cb, region2_green, region3_Cb, region3_green;
    Mat YCrCb = new Mat();
    Mat primaryColor = new Mat();
    Mat green_mat = new Mat();
    Mat calibration = new Mat();

    private int avg1, avg1g, avg2, avg2g, avg3, avg3g;

    private SpikePosition position = SpikePosition.CENTER;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2BGR);
        if (color == General.AllianceLocation.RED_NORTH || color == General.AllianceLocation.RED_SOUTH) {
            Core.extractChannel(YCrCb, primaryColor, 2);
            Core.extractChannel(YCrCb, calibration, 2);
        } else {
            Core.extractChannel(YCrCb, primaryColor, 0);
            Core.extractChannel(YCrCb, calibration, 0);
        }
        Core.extractChannel(YCrCb, green_mat, 1);

    }

    @Override
    public void init(Mat firstFrame) {

        inputToCb(firstFrame);

        region1_Cb = primaryColor.submat(new Rect(region1_pointA, region1_pointB));
        region1_green = green_mat.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = primaryColor.submat(new Rect(region2_pointA, region2_pointB));
        region2_green = green_mat.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = primaryColor.submat(new Rect(region3_pointA, region3_pointB));
        region3_green = green_mat.submat(new Rect(region3_pointA, region3_pointB));

    }

    @Override
    public Mat processFrame(Mat input) {

        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg1g = (int) Core.mean(region1_green).val[0];
        avg2g = (int) Core.mean(region2_green).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];
        avg3g = (int) Core.mean(region3_green).val[0];


        if (overlay) {
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

        }


        double max = Math.max(Math.max(Math.abs(avg1-avg1g), Math.abs(avg2-avg2g)), Math.abs(avg3-avg3g));
        //TODO: YOU WANT THE GREATEST DIFFERENCE BETWEEN COLOR VALUE AND GREEN
        if (max == Math.abs(avg1-avg1g)) {
            position = SpikePosition.LEFT;
            if (overlay) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
        } else if (max == Math.abs(avg2-avg2g)) {
            position = SpikePosition.CENTER; // Record our analysis
            if (overlay) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
        } else if (max == Math.abs(avg3-avg3g)) {
            position = SpikePosition.RIGHT;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }


        return input;
    }

    public SpikePosition getAnalysis() {
        return position;
    }


    public int[] getReadings() {return new int[] {avg1, avg2, avg3, avg1g, avg2g, avg3g};}
}