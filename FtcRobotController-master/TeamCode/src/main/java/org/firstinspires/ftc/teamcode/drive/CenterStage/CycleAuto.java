package org.firstinspires.ftc.teamcode.drive.CenterStage;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.AutoStorage;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "a", name = "2+1 - SOUTH ONLY")
public class CycleAuto extends LinearOpMode {
    General.AutoState autoState = General.AutoState.VISION;
    General.SpikePosition position = General.SpikePosition.LEFT;
    General.AllianceLocation allianceLocation;
    General.ParkLocation parkLocation;
    General.AutoMode autoMode = General.AutoMode.STANDARD;
    ElapsedTime timer;
    double timerOffset;
    boolean didthething = false;


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    ArrayList<Trajectory> paths = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {
        paths.add(new Trajectory(135, 34, 0.6, 12, 0.2).addPoint(99, 30, 90+6.52+2).build());
        paths.add(new Trajectory(100, 30, 0.25, 17).addPoint(107, 24, -90-40.6).addPoint(107, 16, 180).build());
        paths.add(new Trajectory(107, 13, 0.7, 16).addPoint(107, 18, 0).addPoint(80, 18, 90).addPoint(80, 104, 0).addPoint(103, 104, -90).addPoint(102, 115, 0).build());
        paths.add(new Trajectory(102, 120, 0.4, 6).addPoint(102, 123, 0).build());
        paths.add(new Trajectory(100, 123, 0.5, 12).addPoint(100, 100, 180).addPoint(80, 100, 90).addPoint(80, 50, 180).addPoint(85, 14, 180).build()); // cycle intake/approach
        paths.add(new Trajectory(85, 20, 0.6, 12).addPoint(73, 40, 0).addPoint(73, 95, 0).addPoint(101, 95, -90).addPoint(101, 108, 0).build()); // cycle backdrop apprach
        paths.add(new Trajectory(101, 108, 0.4, 12).addPoint(96, 108, 90).build()); //cycle backdrop go to perfect position
        paths.add(new Trajectory(95, 115, 0.4, 8, 0).addPoint(95, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
        paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 115, 0).build());



        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.storeAll();
        driver.resetFlipperEncoder();
        driver.resetSlidesEncoder();
        driver.resetIMUHeading();
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.resetSlidesEncoder();
        driver.setPurpleRelease(false);
        timerOffset = driver.loadTimerPreset();

        timer = new ElapsedTime();
        timer.reset();

        ArrayList<Trajectory> trajectories = paths;
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


        /*switch (allianceLocation) {

            case RED_SOUTH:
                driver.localizer.setEstimatePos(135, 34, 0);
                break;
            case RED_NORTH:
                telemetry.addLine("DO NOT RUN. CYCLE IS ONLY FOR SOUTH");
                driver.localizer.setEstimatePos(135, 84, -90);
                break;
            case BLUE_SOUTH:
                driver.localizer.setEstimatePos(9, 34, 0);
                break;
            case BLUE_NORTH:
                telemetry.addLine("DO NOT RUN. CYCLE IS ONLY FOR SOUTH");
                driver.localizer.setEstimatePos(9, 84, 90);
                break;
        }

         */




        /*aprilTag = new AprilTagProcessor.Builder()
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
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 3"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

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

         */


        driver.localizer.setEstimatePos(135, 34, 0);
        driver.setWeaponsState(General.WeaponsState.HOLDING);
        telemetry.update();
        driver.setClawLiftPos(false);
        int loops = 0;
        /*while (opModeInInit() && loops<500) {
            driver.setCameraMode(General.CameraMode.PROP);
            driver.getCameraEstimate();
            driver.update();
            position = driver.propLocation;
            loops++;
        }

         */
        /*while (!isStarted() && !isStopRequested()) { // TODO: try with no camera, try just !opModeIsActive or just !isStarted
            driver.setCameraMode(General.CameraMode.PROP);
            driver.getCameraEstimate();
            driver.update();
            position = driver.propLocation;
            sleep(50);
        }

         */
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            driver.setSlidesDepositTarget(11);
            driver.update();
            telemetry.addData("CurrentPos", driver.getCurrentPos().getHeading());
            telemetry.addData("Spike Position", position);
            telemetry.update();

            switch (autoState) {

                case VISION: // Read camera, no movement
                    timer.reset();
                    driver.setClawMode(General.ClawMode.BOTH);
                    while (timer.time() < 2.5+timerOffset && opModeIsActive()) { // Read camera for set amount of time
                        driver.setCameraMode(General.CameraMode.PROP);
                        driver.getCameraEstimate();
                        driver.update();
                        position = driver.propLocation;
                        telemetry.addData("Estimate", position.toString());
                        telemetry.update();
                    }
                    driver.setCameraMode(General.CameraMode.IDLE);
                    autoState = General.AutoState.PURPLE_APPROACH;
                    trajectories = AutoStorage.generateCycleAutoPaths(parkLocation, position, allianceLocation); // load up the paths
                    break;
                case PURPLE_APPROACH: // Drop the purple pixel on the spike mark
                    boolean result = driver.runAutoPath(trajectories.get(0).path);
                    telemetry.addLine("Running Approach 1");
                    //telemetry.update();
                    if (result) {
                        // the path is ready to move on
                        timer.reset();

                        while (timer.time() < 0.3 && opModeIsActive()) {
                            driver.drive(0, 0, 0, false);
                            driver.setPurpleRelease(true); // release pixel
                            driver.update();
                        }
                        timer.reset();
                        autoState = General.AutoState.SPIKE; // advance to next state
                    }
                    break;
                case SPIKE:
                    driver.setWeaponsState(General.WeaponsState.INTAKING); // effectively unused state
                    autoState = General.AutoState.BACKUP;
                    break;
                case BACKUP: // drive to the stack, pick one pixel up
                    //driver.setWeaponsState(General.WeaponsState.INTAKING);
                    driver.setClawMode(General.ClawMode.RIGHT);
                    result = driver.runAutoPath(trajectories.get(1).path);
                    telemetry.addData("current pos", driver.getCurrentPos().toString());
                    //telemetry.update();
                    if (result) {
                        timer.reset();
                        driver.setWeaponsState(General.WeaponsState.INTAKING);
                        while (driver.getIntakeCurrent() < 4 && opModeIsActive() && timer.time() < 3) { // drive forwards until you hit the stack
                            driver.drive(0, -0.2, 0, false);
                            driver.setClawMode(General.ClawMode.RIGHT);
                            driver.update();
                        }
                        while ((timer.time() < 4 && !driver.getLeftHasPixel()) && opModeIsActive()) {
                            driver.drive(0, 0.1, 0, false); // drive backwards slowly to take in your pixel
                            driver.update();
                        }
                        timer.reset();
                        while (timer.time() < 0.5 && opModeIsActive()) { // drive more away
                            driver.drive(0, 0.25, 0, false);
                            driver.setClawMode(General.ClawMode.BOTH);
                            driver.update();
                        }
                        driver.setWeaponsState(General.WeaponsState.HOLDING);
                        driver.update();
                        driver.setIntakeMode(General.IntakeMode.INTAKE);
                        autoState = General.AutoState.APPROACH_2;
                    }
                    break;
                case APPROACH_2:
                    result = driver.runAutoPath(trajectories.get(2).path);
                    if (driver.getCurrentPos().getY() > 85 && !didthething) {
                        driver.setWeaponsState(General.WeaponsState.EXTEND);
                        didthething = true;
                    } else if (driver.getCurrentPos().getY() < 30 && driver.getCurrentPos().getY() > 18) {
                        driver.setIntakePower(-0.80);
                    } else if (driver.getCurrentPos().getY() >= 30) {
                        driver.setIntakeMode(General.IntakeMode.LOCK);
                    }
                    if (result) {
                        //driver.drive(0, 0, 0, false);
                        timer.reset();


                        /*while (timer.time() < 1.5 && opModeIsActive()) {
                            driver.update();
                            Pose2d pos = driver.getAprilTagPosition();
                            driver.setCameraMode(General.CameraMode.APRILTAG);
                            driver.setTagOfInterest(5);
                            driver.getCameraEstimate();
                            if (pos != null) {
                                if (pos.getX() < -2 && driver.getCurrentPos().getX() > 100 && driver.getCurrentPos().getX() < 117) {
                                    driver.drive(0.4, 0, -driver.getCurrentPos().getHeading()/20, false);
                                } else if (pos.getX() > 2 && driver.getCurrentPos().getX() > 100 && driver.getCurrentPos().getX() < 117) {
                                    driver.drive(-0.4, 0, -driver.getCurrentPos().getHeading()/20, false);
                                } else {
                                    driver.drive(0, 0, -driver.getCurrentPos().getHeading()/20, false);
                                }
                            } else {
                                driver.drive(0.2, 0, -driver.getCurrentPos().getHeading()/20, false);
                            }
                            telemetry.addData("tiemr", timer.time());
                            if (pos != null) {
                                telemetry.addData("pos", pos.toString());
                            } else {
                                telemetry.addLine("pos is null");
                            }
                            telemetry.update();
                        }

                         */





                        timer.reset();
                        while (opModeIsActive() && (!driver.getFSRPressed() && timer.time()<2)) {
                            /*Pose2d pos = driver.getAprilTagPosition();
                            driver.setCameraMode(General.CameraMode.APRILTAG);
                            driver.setTagOfInterest(5);
                            driver.getCameraEstimate();
                            if (pos != null) {
                                if (pos.getX() < -2 && driver.getCurrentPos().getX() > 100 && driver.getCurrentPos().getX() < 117) {
                                    driver.drive(0.4, 0.2, -driver.getCurrentPos().getHeading()/20, false);
                                } else if (pos.getX() > 2 && driver.getCurrentPos().getX() > 100 && driver.getCurrentPos().getX() < 117) {
                                    driver.drive(-0.4, 0.2, -driver.getCurrentPos().getHeading()/20, false);
                                } else {
                                    driver.drive(0, 0.2, -driver.getCurrentPos().getHeading()/20, false);
                                }
                            } else {
                                driver.drive(0, 0.2, -driver.getCurrentPos().getHeading()/20, false);
                            }

                             */
                            driver.drive(0, 0.2, -driver.getCurrentPos().getHeading()/15, false);
                            driver.update();
                            telemetry.addData("cur pos", driver.getCurrentPos().toString());
                            telemetry.update();
                        }
                        //driver.stopStreamingVP();

                        autoState = General.AutoState.APPROACH_3;
                    }
                    break;
                case APPROACH_3:
                    timer.reset();
                    driver.setClawLiftPos(true);
                    while (timer.time() < 0.8 && opModeIsActive() && position != General.SpikePosition.CENTER) {
                        driver.followCurve(trajectories.get(3).path);
                        //driver.drive(0, 0.2, 0, false);
                        driver.update();
                    }//TODO: Move FSR and AprilTag stuff to here
                    timer.reset();
                    while (opModeIsActive() && (!driver.getFSRPressed() && timer.time()<2)) {
                        driver.drive(0, 0.2, -driver.getCurrentPos().getHeading()/15, false);
                        driver.update();
                        telemetry.addData("cur pos", driver.getCurrentPos().toString());
                        telemetry.update();
                    }
                    driver.setClawMode(General.ClawMode.LEFT); // release the yellow pixel (right claw)
                    while (timer.time() < 1 && opModeIsActive()) { // wait for it to drop
                        driver.drive(0, 0, 0, false);
                        //TODO: Drop one pixel at a time
                        driver.update();
                    }

                    while (timer.time() < 1.2 && opModeIsActive()) { // drive out of the way to dump your extra pixel somewhere else
                        switch (position) {
                            case LEFT:
                                driver.drive(0.55, 0, 0, false);
                                break;
                            case CENTER:
                                driver.drive(-0.55, 0, 0, false);
                                break;
                            case RIGHT:
                                driver.drive(-0.55, 0, 0, false);
                                break;
                        }

                        driver.update();
                    }
                    driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                    timer.reset();
                    /*while (opModeIsActive()) {
                        driver.drive(0, 0, 0, false);
                        driver.update();
                    }

                     */
                    //autoState = General.AutoState.CYCLE_INTAKE; // TODO: Add another cycle
                    autoState = General.AutoState.PARK_1;
                    break;
                case CYCLE_INTAKE:
                    result = driver.runAutoPath(trajectories.get(4).path);
                    if (driver.getCurrentPos().getY() < 30 && driver.getIntakeMode() != General.IntakeMode.INTAKE) {
                        driver.setWeaponsState(General.WeaponsState.INTAKING);
                    }
                    if (result) {
                        timer.reset();
                        while (timer.time() < 3 && opModeIsActive()) {
                            //grab pixel
                            //driver.setWeaponsState(General.WeaponsState.PRIMED);
                            driver.followCurve(trajectories.get(4).path);
                            driver.update();
                        }
                        driver.setWeaponsState(General.WeaponsState.HOLDING);
                        autoState = General.AutoState.CYCLE_APPROACH;
                        timer.reset();
                        //while (opModeIsActive()) {driver.drive(0, 0, 0, false);}
                    }

                    driver.update();
                    break;
                case CYCLE_APPROACH:
                    result = driver.runAutoPath(trajectories.get(5).path);
                    if (result) {
                        autoState = General.AutoState.CYCLE_APPROACH_2;
                        timer.reset();
                    }
                    if (driver.getCurrentPos().getY() > 85) {
                        //driver.setSlidesTarget(4);
                        driver.setWeaponsState(General.WeaponsState.EXTEND);
                    }
                    driver.update();
                    break;
                case CYCLE_APPROACH_2:
                    timer.reset();
                    while (timer.time() < 0.8 && opModeIsActive()) {
                        driver.followCurve(trajectories.get(6).path);
                        //driver.drive(0, 0.2, 0, false);
                        driver.update();
                    }
                    while (timer.time() < 2 && opModeIsActive()) {
                        driver.drive(0, 0.2, 0, false);
                        driver.update();
                    }
                    while (timer.time() < 3.5 && opModeIsActive()) {
                        driver.drive(0, 0, 0, false);
                        driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                        // release pixel
                        driver.update();
                    }
                    timer.reset();
                    autoState = General.AutoState.PARK_1;
                    break;
                case PARK_1:
                    if (parkLocation != General.ParkLocation.CENTER) {
                        telemetry.addData("x", driver.getCurrentPos().getX());
                        telemetry.addData("y", driver.getCurrentPos().getY());
                        telemetry.addData("head", driver.getCurrentPos().getHeading());
                        telemetry.update();
                        result = driver.runAutoPath(trajectories.get(4).path);
                        if (result) {
                            autoState = General.AutoState.PARK2;
                        }
                    } else {
                        if (timer.time() > 1) {
                            driver.drive(0,0,0,false);
                        } else {
                            driver.drive(0, -0.2, 0, false);
                        }
                    }
                    break;
                case PARK2:
                    telemetry.addData("x", driver.getCurrentPos().getX());
                    telemetry.addData("y", driver.getCurrentPos().getY());
                    telemetry.addData("head", driver.getCurrentPos().getHeading());
                    telemetry.update();
                    driver.followCurve(trajectories.get(5).path);
                    break;
            }



        }
    }
}
