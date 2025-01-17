package org.firstinspires.ftc.teamcode.drive.CenterStage;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.AutoStorage;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;
import org.firstinspires.ftc.teamcode.vision.ThreeZonePropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "a")
public class Auto extends LinearOpMode {
    General.AutoState autoState = General.AutoState.VISION;
    General.SpikePosition position = General.SpikePosition.LEFT;
    General.AllianceLocation allianceLocation;
    General.ParkLocation parkLocation;
    ElapsedTime timer, stateOverrideTimer;
    OpenCvCamera PropCameraR, PropCameraL, cameraOfInterest;
    ThreeZonePropDetectionPipeline propPipeline;
    double timerOffset1, timerOffset2, timerOffset3;
    boolean weaponsExtended = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static double offpos = 0;


    double Xpos = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.storeAll();
        driver.resetFlipperEncoder();
        driver.resetSlidesEncoder();
        driver.resetIMUHeading();
        driver.setWeaponsState(General.WeaponsState.INTAKING);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.resetSlidesEncoder();
        timerOffset1 = driver.loadTimerPreset1();
        timerOffset2 = driver.loadTimerPreset2();
        timerOffset3 = driver.loadTimerPreset3();

        ArrayList<Trajectory> trajectories = AutoStorage.generateAutoPaths(General.ParkLocation.RIGHT, General.SpikePosition.CENTER, General.AllianceLocation.RED_NORTH); // Default loaded paths. Are changed later in code once vision estimate is received
        allianceLocation = driver.loadAlliancePreset();
        parkLocation = driver.loadParkPreset();

        telemetry.addLine("---------- Check Auto Presets -----------");

        if (allianceLocation == General.AllianceLocation.NONE) {
            telemetry.addLine("WARNING - ALLIANCE LOCATION NOT LOADED. RUNNING RED_NORTH AS DEFAULT");
            allianceLocation = General.AllianceLocation.RED_NORTH;
        } else {
            telemetry.addData("Alliance Location", allianceLocation.toString());
        }
        if (parkLocation == General.ParkLocation.NONE) {
            telemetry.addLine("WARNING - PARK LOCATION NOT LOADED. RUNNING RIGHT AS DEFAULT");
            parkLocation = General.ParkLocation.RIGHT;
        } else {
            telemetry.addData("Park Location", parkLocation.toString());
        }

        switch (allianceLocation) {

            case RED_SOUTH:
                driver.localizer.setEstimatePos(135, 34, 0);
                driver.setPurpleSouthRelease(false);
                break;
            case RED_NORTH:
                driver.localizer.setEstimatePos(135, 84, 0);
                driver.setPurpleNorthRelease(false);
                break;
            case BLUE_SOUTH:
                driver.localizer.setEstimatePos(9, 34, 0);
                driver.setPurpleSouthRelease(false);
                break;
            case BLUE_NORTH:
                driver.localizer.setEstimatePos(9, 84, 0);
                driver.setPurpleNorthRelease(false);
                break;
        }

        initCameras(allianceLocation); // initialize the correct camera
        initAprilTag();
        driver.setWeaponsState(General.WeaponsState.HOLDING);
        driver.setClawLiftPos(false);
        driver.setUseIMUForLocalization(true);

        while (!isStarted() && !isStopRequested()) { // warm up the camera as long as init goes on
            telemetry.addLine("Running Detection");
            telemetry.addData("analysis", propPipeline.getAnalysis());
            telemetry.update();
        }

        timer = new ElapsedTime();
        stateOverrideTimer = new ElapsedTime();
        timer.reset();

        if (driver.loadSlidesUpPreset()) {
            driver.setSlidesDepositTarget(11);
        } else {
            driver.setSlidesDepositTarget(9);
        }



        while (opModeIsActive()) {
            driver.update();
            telemetry.addData("x", driver.getCurrentPos().getX());
            telemetry.addData("y", driver.getCurrentPos().getY());
            telemetry.addData("head", driver.getCurrentPos().getHeading());
            telemetry.addData("Spike Position", position);
            telemetry.update();

            switch (autoState) {

                case VISION:
                    timer.reset();
                    driver.setClawMode(General.ClawMode.BOTH);
                    while (timer.time() < 0.1+ timerOffset1 && opModeIsActive()) { // Grab the final camera estimate
                        position = propPipeline.getAnalysis();
                        telemetry.addData("analysis", position);
                        telemetry.update();
                    }
                    autoState = General.AutoState.PURPLE_APPROACH;
                    cameraOfInterest.stopStreaming();
                    cameraOfInterest.closeCameraDevice();
                    trajectories = AutoStorage.generateAutoPaths(parkLocation, position, allianceLocation); // Load all auto paths
                    stateOverrideTimer.reset();
                    break;
                case PURPLE_APPROACH:
                    boolean result = driver.runAutoPath(trajectories.get(0).path); // Follow the path
                    if (driver.getIntakePos() < 20) {
                        driver.setIntakePower(-0.2);
                    } else {
                        driver.setIntakePower(0);
                    }
                    if (result) { // if the path is complete, move on
                        timer.reset();
                        while (timer.time() < 0.35 && opModeIsActive()) { // release the purple pixel on the spike mark
                            driver.drive(0, 0, 0, false);
                            //TODO: RELEASE PIXEL WITH SERVO

                            if (allianceLocation == General.AllianceLocation.BLUE_SOUTH || allianceLocation == General.AllianceLocation.RED_SOUTH) {
                                driver.setPurpleSouthRelease(true);
                            } else {
                                driver.setPurpleNorthRelease(true);

                            }
                            // release pixel
                            driver.update();
                        }
                        autoState = General.AutoState.APPROACH_2; // advance to the next state
                        timer.reset();
                        while (timer.time() < timerOffset2 && opModeIsActive()) { // wait the commanded period of time
                            telemetry.addLine("Waiting for Preset Timer to expire");
                            telemetry.addData("Time Remaining", timerOffset2-timer.time());
                            telemetry.update();
                            driver.update();
                            driver.drive(0, 0, 0, false);
                        }
                        timer.reset();
                        stateOverrideTimer.reset();
                    }
                    if (stateOverrideTimer.time() > 7) { // If the robot stalls, move on to the next state
                        autoState = General.AutoState.APPROACH_2;
                        driver.setPurpleNorthRelease(true);
                        driver.setPurpleSouthRelease(true);
                        System.out.println("State Skipped due to timeout");
                        timer.reset();
                        while (timer.time() + 7 < timerOffset2 && opModeIsActive()) { // offset the timer based off of the time already spent in stall
                            telemetry.addLine("Waiting for Preset Timer to expire");
                            telemetry.addData("Time Remaining", timerOffset2-timer.time());
                            telemetry.update();
                            driver.update();
                            driver.drive(0, 0, 0, false);
                        }
                        timer.reset();
                        stateOverrideTimer.reset();
                    }
                    break;
                case APPROACH_2: // approach the backdrop
                    result = driver.runAutoPath(trajectories.get(1).path);
                    if (allianceLocation == General.AllianceLocation.RED_SOUTH || allianceLocation == General.AllianceLocation.BLUE_SOUTH) {
                        if (driver.getCurrentPos().getY() > 85 && !weaponsExtended) { // extend all subsystems after you have cleared the stage door
                            driver.setWeaponsState(General.WeaponsState.EXTEND);
                            weaponsExtended = true;
                        }
                    } else {
                        if (!weaponsExtended) {
                            driver.setWeaponsState(General.WeaponsState.EXTEND); // extend all subsystems
                            weaponsExtended = true;
                        }
                    }
                    if (result) {
                        autoState = General.AutoState.APPROACH_3; // move on to the next state
                    }
                    if (stateOverrideTimer.time() > 9) { // if the robot stalls, skip to the next state
                        autoState = General.AutoState.APPROACH_3;
                        System.out.println("State Skipped due to timeout");
                        stateOverrideTimer.reset();
                    }
                    break;
                case APPROACH_3: // finalize backdrop position and deposit

                    //TODO: INSERT APRILTAG CODE
                    visionPortal.resumeStreaming();
                    timer.reset();
                    while (timer.time() < 2 && opModeIsActive()) {

                        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                        for (AprilTagDetection detection : currentDetections){
                            if (detection.metadata != null) {
                                if (detection.id == 2 && Xpos == 100){
                                    Xpos = -detection.ftcPose.x;
                                }
                            }
                        }

                        double Xcurrent_error = Xpos+offpos-driver.getCurrentPos().getX();

                        driver.goToAnotherPosition(new Pose2d(Xcurrent_error, 0, driver.getIMUHeading()), 0, 0, 0.5, Math.signum(Xcurrent_error)*-90, 0.3, 1, false, 1);

                    }
                    visionPortal.stopStreaming();

                    //---------------------------

                    timer.reset();
                    while (timer.time() < 2 && opModeIsActive() && !driver.getFSRPressed()) { // drive into the backdrop until the FSR is pressed
                        driver.drive(0, 0.2, -driver.getCurrentPos().getHeading()/8, false);
                        driver.update();
                    }
                    driver.setWeaponsState(General.WeaponsState.DEPOSIT); // drop the pixel, systems automatically fold up
                    timer.reset();
                    while (timer.time() < 0.5 && opModeIsActive()) {
                        driver.drive(0, 0, 0, false);
                        // release pixel
                        driver.update();
                    }
                    timer.reset();
                    while (timer.time() < timerOffset3 && opModeIsActive()) { // offset the timer based off of the time already spent in stall
                        telemetry.addLine("Waiting for Preset Timer to expire");
                        telemetry.addData("Time Remaining", timerOffset3-timer.time());
                        telemetry.update();
                        driver.update();
                        driver.drive(0, 0, 0, false);
                    }
                    timer.reset();
                    autoState = General.AutoState.PARK_1;
                case PARK_1:
                    if (parkLocation != General.ParkLocation.CENTER) { // Follow the corresponding path for the park
                        result = driver.runAutoPath(trajectories.get(2).path);
                        if (result) {
                            autoState = General.AutoState.PARK2;
                            timer.reset();
                        }
                    } else { // If you are doing a center park, just back up slowly for a second
                        if (timer.time() > 0.5) {
                            driver.drive(0,0,0,false);
                        } else {
                            driver.drive(0, -0.4, 0, false);
                        }
                    }
                    break;
                case PARK2:
                    if (driver.loadParkOnWall()) {
                        if (timer.time() < 0.6) {
                            driver.drive(0, 0.5, 0, false);
                        } else {
                            driver.drive(0, 0, 0, false);
                            driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
                        }
                    } else {
                        driver.followCurve(trajectories.get(3).path); // do the final parking move
                    }
                    break;
            }
        }
    }

    public void initCameras(General.AllianceLocation allianceLocation) { // handles camera intialization and selection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        if (allianceLocation == General.AllianceLocation.BLUE_NORTH | allianceLocation == General.AllianceLocation.BLUE_SOUTH) {
            PropCameraR = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "PropCamR"), viewportContainerIds[1]);
            propPipeline = new ThreeZonePropDetectionPipeline(true, allianceLocation);
            PropCameraR.setPipeline(propPipeline);
        } else {
            PropCameraL = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "PropCamL"), viewportContainerIds[0]);
            propPipeline = new ThreeZonePropDetectionPipeline(true, allianceLocation);
            PropCameraL.setPipeline(propPipeline);
        }

        if (allianceLocation == General.AllianceLocation.BLUE_NORTH | allianceLocation == General.AllianceLocation.BLUE_SOUTH) {
            cameraOfInterest = PropCameraR;
        } else {
            cameraOfInterest = PropCameraL;
        }

        cameraOfInterest.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cameraOfInterest.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
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

        visionPortal.stopStreaming();

        //visionPortal.resumeStreaming();

    }   // end method initAprilTag()

}
