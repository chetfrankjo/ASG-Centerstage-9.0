package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.drive.Constants.VisionConstants.REGION3_TOPLEFT_ANCHOR_POINT_RED;

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

public class PDP_DualCamera extends OpenCvPipeline {
    boolean overlay;

    General.AllianceLocation color = General.AllianceLocation.RED_NORTH;
    public PDP_DualCamera(boolean enableOverlay, General.AllianceLocation color) {
        overlay = enableOverlay;
        this.color=color;
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

    Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT_RED.x, REGION3_TOPLEFT_ANCHOR_POINT_RED.y);
    Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT_RED.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT_RED.y + REGION_HEIGHT);


    /*
     * Working variables
     */
    Mat region3_Cb, region3_green;
    Mat YCrCb = new Mat();
    Mat primaryColor = new Mat();
    Mat green_mat = new Mat();
    Mat calibration = new Mat();

    private int avg3, avg3g;

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

        region3_Cb = primaryColor.submat(new Rect(region3_pointA, region3_pointB));
        region3_green = green_mat.submat(new Rect(region3_pointA, region3_pointB));

    }

    @Override
    public Mat processFrame(Mat input) {

        inputToCb(input);

        avg3 = (int) Core.mean(region3_Cb).val[0];
        avg3g = (int) Core.mean(region3_green).val[0];


        if (overlay) {
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

        }

        return input;
    }

    public SpikePosition getAnalysis(int[] readings) {

        double max = Math.max(Math.max(Math.abs(readings[0]-readings[2]), Math.abs(readings[1]-readings[3])), Math.abs(avg3-avg3g));
        //TODO: YOU WANT THE GREATEST DIFFERENCE BETWEEN COLOR VALUE AND GREEN
        if (max == Math.abs(readings[0]-readings[2])) {
            position = SpikePosition.LEFT;
        } else if (max == Math.abs(readings[1]-readings[3])) {
            position = SpikePosition.CENTER;
        } else if (max == Math.abs(avg3-avg3g)) {
            position = SpikePosition.RIGHT;
        }

        return position;
    }
    public int[] getReadings() {return new int[] {avg3, avg3g};}
}