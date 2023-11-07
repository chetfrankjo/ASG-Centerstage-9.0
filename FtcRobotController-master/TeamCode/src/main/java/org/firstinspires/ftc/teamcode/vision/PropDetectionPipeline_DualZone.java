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

public class PropDetectionPipeline_DualZone extends OpenCvPipeline {
    boolean overlay;
    public static int MIN_THRESH;
    General.AllianceLocation color = General.AllianceLocation.RED_NORTH;
    public PropDetectionPipeline_DualZone(boolean enableOverlay, General.AllianceLocation color) {
        overlay = enableOverlay;
        if (color == General.AllianceLocation.RED_NORTH || color == General.AllianceLocation.RED_SOUTH) {
            MIN_THRESH = 120;
        } else {
            MIN_THRESH = 120;
        }
        this.color=color;
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final int EXTREME_MIN_THRESH = 100;


    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION1_TOPLEFT_ANCHOR_POINT;
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = VisionConstants.REGION2_TOPLEFT_ANCHOR_POINT;
    static final int REGION_WIDTH = VisionConstants.REGION_WIDTH;
    static final int REGION_HEIGHT = VisionConstants.REGION_HEIGHT;

    Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb, region1_green, region2_green, calibration_region;
    Mat YCrCb = new Mat();
    Mat primaryColor = new Mat();
    Mat green_mat = new Mat();
    Mat calibration = new Mat();

    private int avg1, avg2, avg1g, avg2g, avgcal;

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
        calibration_region = calibration.submat(new Rect(new Point(120, 200), new Point(140, 240)));
    }

    @Override
    public Mat processFrame(Mat input) {

        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg1g = (int) Core.mean(region1_green).val[0];
        avg2g = (int) Core.mean(region2_green).val[0];
        avgcal = (int) Core.mean(calibration_region).val[0];

        if (color == General.AllianceLocation.BLUE_NORTH) {
            avg1 = avg1 + 30;
        }

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

        }
        int max = Math.max(avg1, avg2);


        if (max == avg1 && max > MIN_THRESH && avg1g < 120) // Was it from region 1?
        {

            position = SpikePosition.LEFT; // Record our analysis

            if (overlay) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
        } else if ((max == avg2 || avg1g >= 120) && max > MIN_THRESH && avg2g < 120) // Was it from region 2?
        {
            position = SpikePosition.CENTER; // Record our analysis
            if (overlay) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
        } else {
            position = SpikePosition.RIGHT; // Record our analysis
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    new Point(0,0), // First point which defines the rectangle
                    new Point(15, 15), // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        return input;
    }

    public SpikePosition getAnalysis() {
        return position;
    }
    public int[] getReadings() {return new int[] {avg1, avg2, avg1g, avg2g, avgcal};}
}