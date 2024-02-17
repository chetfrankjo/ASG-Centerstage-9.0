package org.firstinspires.ftc.teamcode.drive.CenterStage;
import org.firstinspires.ftc.teamcode.drive.Constants.DriveConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.Localizer;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(group = "a")
public class CalesFirstProgram_whoLetBroCode extends LinearOpMode{
    //double Xcurrent_time, Xprevious_time;
    //double Xcurrent_error, Xprevious_error;
    //double Xp, Xi, Xd, Xk_p, Xk_i, Xmax_i, Xk_d;
    double Tcurrent_time, Tprevious_time;
    double Xcurrent_error, Xprevious_error;
    double Tcurrent_error, Tprevious_error;
    double Tp, Ti, Td, Tmax_i, Ttotal;
    double Xp, Xi, Xd, Xmax_i, Xtotal;

    long Tlast_time;

    public static double Tk_p = 0.0132;
    public static double Tk_i = 0.00000001;
    public static double Tk_d = 0.0008;

    public static double Xk_p = 0.5;
    public static double Xk_i = 0;
    public static double Xk_d = 0;

    public static double offpos = 0;

    double Tangle = 0;

    double Xpos = 100;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    FtcDashboard dashboard;

    private AprilTagProcessor aprilTag;
    Pose2d position;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    @Override
    public void runOpMode(){
        dashboard = FtcDashboard.getInstance();

        //Xtimer = new ElapsedTime();
        initAprilTag();
        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.getSensors();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        Tmax_i = 1;
        Xmax_i = .4;
        waitForStart();
        if (opModeIsActive()){
            driver.resetIMUHeading();
            while (opModeIsActive()){


                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections){
                    if (detection.metadata != null) {
                        if (detection.id == 2 && Xpos == 100){
                            Xpos = -detection.ftcPose.x;
                        }
                    }
                }
                //driver.getSensors();
                position = driver.getCurrentPos();
                telemetry.addData("thing", position.getX());
                TelemetryPacket packet = new TelemetryPacket();
                dashboard.sendTelemetryPacket(packet);          // Send field overlay and telemetry data to FTCDashboard


                telemetry.addData("Tag position", Xpos);
                Tcurrent_time = System.nanoTime();
                Tcurrent_error = Tangle - driver.pullIMUHeading();

                Xcurrent_error = Xpos+offpos-position.getX();
                packet.put("difference", Xpos-position.getX());
                telemetry.update();


                Tp = Tk_p * Tcurrent_error;
                Ti += Tk_i * (Tcurrent_error * (Tcurrent_time / 1000000000));
                if (Ti > Tmax_i) {
                    Ti = Tmax_i;
                } else if (Ti < -Tmax_i) {
                    Ti = -Tmax_i;
                }
                Td = Tk_d * (Tcurrent_error - Tprevious_error) / (Tcurrent_time);
                Ttotal = Tp + Ti + Td;


                Xp = Xk_p * Xcurrent_error;
                Xi += Xk_i * (Xcurrent_error * (Tcurrent_time / 1000000000));
                if (Xi > Xmax_i) {
                    Xi = Xmax_i;
                } else if (Xi < -Xmax_i) {
                    Xi = -Xmax_i;
                }
                Xd = Xk_d * (Xcurrent_error - Xprevious_error) / (Tcurrent_time);
                Xtotal = Xp + Xi + Xd;

                driver.drive(Xtotal, -gamepad1.left_stick_y, Ttotal, false);
                Xprevious_error = Xcurrent_error;
                Tprevious_error = Tcurrent_error;
                Tprevious_time = Tcurrent_time;
                if(gamepad1.a){
                    Tangle = 90;
                    Tprevious_error = 0;
                    while (gamepad1.a){
                        sleep(10);
                    }
                } else if (gamepad1.b){
                    Tangle = 0;
                    Tprevious_error = 0;
                    while (gamepad1.b){
                        sleep(10);
                    }
                }
                driver.update();
            }

        }
        visionPortal.close();

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360)); //TODO:  640x480, 800x600, 640x360, 1920x1080, 800x448, 864x480 <- supported resolutions

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        //visionPortal.stopStreaming();

        //visionPortal.resumeStreaming();

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addData("field position", detection.metadata.fieldPosition.toString());
                telemetry.addData("corrected field position y", detection.metadata.fieldPosition.get(0)+72);
                telemetry.addData("corrected field position x", 72-detection.metadata.fieldPosition.get(1));

                telemetry.addData("global position x", 72-detection.metadata.fieldPosition.get(1)-detection.ftcPose.x);
                telemetry.addData("global position y", detection.metadata.fieldPosition.get(0)+72-detection.ftcPose.y);
                //detection.metadata.fi

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class

