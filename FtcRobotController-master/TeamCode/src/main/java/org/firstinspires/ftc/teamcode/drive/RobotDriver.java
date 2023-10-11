package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.MathFunctions.AngleWrap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Point;
import org.firstinspires.ftc.teamcode.drive.Constants.AssemblyConstants;
import org.firstinspires.ftc.teamcode.drive.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline_DualZone;
import org.firstinspires.ftc.teamcode.DataTypes.General.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
public class RobotDriver {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront, verticalLeft, verticalRight, horizontal, slides, slides2;
    private List<DcMotorEx> driveMotors, odometryEncoders;
    private IMU imu;
    private VoltageSensor batterylevel;
    String verticalLeftEncoderName = "fl", horizontalEncoderName = "br", verticalRightEncoderName = "bl";
    public int[] encoders;
    public Localizer localizer;
    public Pose2d CurrentVelocities = new Pose2d();
    private Pose2d PreviousVelocities  = new Pose2d();
    static Pose2d currentPos = new Pose2d(0, 0, 0);
    private RevTouchSensor touch;
    private List<LynxModule> allHubs;
    private double slidesLength, touchVal, imuheading;
    double slidesTarget, slidesPower;
    static double frp=0, flp=0, brp=0, blp=0;
    public int[] colorLeft, colorRight;
    public boolean slidesPID = true;
    final double slideTickToInch = AssemblyConstants.slideTickToInch;
    public int loops = 0;
    long lastLoopTime = System.nanoTime();
    public double loopSpeed = 0;
    public static double ROBOT_RADIUS = Constants.DriveConstants.ROBOT_RADIUS;
    public Canvas overlay;
    public boolean useIMU = false;
    private boolean detectProp = true;
    private CameraMode cameraMode = CameraMode.IDLE;
    boolean cameraReady = false;
    boolean getCameraEstimate = false;
    private static final double CAMERA_X_OFFSET = DriveConstants.CAMERA_X_OFFSET;
    private static final double CAMERA_Y_OFFSET = DriveConstants.CAMERA_Y_OFFSET;
    SpikePosition propLocation;

    OpenCvCamera OpenCvCam;
    PropDetectionPipeline_DualZone propPipeline;

    private double tagsize = DriveConstants.tagsize;
    private double fx = DriveConstants.fx;
    private double fy = DriveConstants.fy;
    private double cx = DriveConstants.cx;
    private double cy = DriveConstants.cy;

    final FtcDashboard dashboard;
    PIDFCoefficients slidesPIDConstants = AssemblyConstants.slidesPIDConstants;


    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public RobotDriver(HardwareMap hardwareMap, boolean useIMU) {


        frp = 0;
        flp = 0;
        brp = 0;
        blp = 0;

        localizer = new Localizer();
        encoders = new int[localizer.encoders.length];

        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "fr");
        rightFront = hardwareMap.get(DcMotorEx.class, "br");

        verticalLeft = hardwareMap.get(DcMotorEx.class, verticalLeftEncoderName);
        verticalRight = hardwareMap.get(DcMotorEx.class, verticalRightEncoderName);
        horizontal = hardwareMap.get(DcMotorEx.class, horizontalEncoderName);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveMotors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        odometryEncoders = Arrays.asList(verticalLeft, verticalRight, horizontal);

        if (useIMU) {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
            imu.resetYaw();
        }

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        touch = hardwareMap.get(RevTouchSensor.class, "touch");

        batterylevel = hardwareMap.get(VoltageSensor.class, "Control Hub");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        propPipeline = new PropDetectionPipeline_DualZone(true);
        OpenCvCam.setPipeline(propPipeline);

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
        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));

        //builder.setCameraResolution(new Size(640, 480));

        // Set and enable the processor.
        builder.addProcessors(aprilTag, tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        colorLeft = new int[3];
        colorRight = new int[3];

        dashboard = FtcDashboard.getInstance();


    }

    /**
     * Function for quickly defining certain parameters for RobotDriver
     * @param useIMU
     */
    public void setKeyParameters(boolean useIMU) {
        this.useIMU = useIMU;
    }

    public void update() {              // Updates all motors and sensors. Nothing will happen if this function is not called
        loops++;
        long currentTime = System.nanoTime();
        if (loops == 1){
            lastLoopTime = currentTime;
        }
        loopSpeed = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime;
        updateEstimate();           // Updates localization data and motor encoders
        updateDriveMotors();        // Updates drive motor power
        updateCamera();             // Updates current requested camera operation (if any)

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("x", currentPos.getY());
        packet.put("y", currentPos.getX());
        packet.put("heading (deg)", currentPos.getHeading());
        if (useIMU) {
            packet.put("heading (imu)", imuheading);
        }
        packet.put("slides", slidesLength);
        packet.put("color right", colorRight);
        packet.put("color left", colorLeft);
        packet.put("loop speed", loopSpeed);
        packet.put("velocity left", CurrentVelocities.getX());
        packet.put("velocity right", CurrentVelocities.getY());
        packet.put("velocity hor", CurrentVelocities.getHeading());

        overlay = packet.fieldOverlay();

        drawRobot(overlay, new Pose2d(currentPos.getY(), currentPos.getX(), Math.toRadians(currentPos.getHeading())));     //Draw the robot's position on the diagram

        dashboard.sendTelemetryPacket(packet);          // Send field overlay and telemetry data to FTCDashboard
    }

    public void updateEstimate(){           // Updates all encoders and re-estimates localization
        getSensors();
        localizer.updateEncoders(encoders);
        localizer.update(loopSpeed);
        currentPos = localizer.getPosEstimate();
        PreviousVelocities = CurrentVelocities;
        CurrentVelocities = localizer.getvelocity();

    }

    public Pose2d getCurrentPos() {
        return currentPos;
    }
    public Pose2d getCurrentVelocities() {return CurrentVelocities;}

    public double leftchange() {return localizer.leftchange();}
    public double rightchange() {return localizer.rightchange();}

    public void getSensors() {             // Retrieves all encoder data
        encoders[0] = verticalLeft.getCurrentPosition();
        encoders[2] = -horizontal.getCurrentPosition();
        encoders[1] = verticalRight.getCurrentPosition();
        slidesLength = -slides2.getCurrentPosition()/slideTickToInch;
        //turretHeading = turret.getCurrentPosition()*25;
        //vbarHeading = (v4bar.getCurrentPosition()/vbarTickToInch)+zeroV4barAngle;
        //distRight = (distright.getVoltage()*1000)/3.2;
        //distLeft = (distleft.getVoltage()*1000)/3.2;

        touchVal = touch.getValue();
        if (useIMU) {
            imuheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }

    /**
     * Handles ALL camera functions. Switching between detection states is done with a variable
     */
    public void updateCamera() {
        if (cameraMode == CameraMode.PROP) {
            if (!cameraReady) {
                OpenCvCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        OpenCvCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        cameraReady = true;
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                    }
                });
                if (getCameraEstimate) {
                    propLocation = propPipeline.getAnalysis(); //get the estimate
                    getCameraEstimate = false; //close down all camera functions (we won't need them again)
                    OpenCvCam.stopStreaming();
                    OpenCvCam.closeCameraDevice();
                    cameraMode = CameraMode.IDLE;
                }
            }

        } else if (cameraMode == CameraMode.APRILTAG) {
            visionPortal.setProcessorEnabled(aprilTag, true);
            visionPortal.resumeStreaming();
            if (getCameraEstimate) {
                Pose2d result = getAprilTagEstimate();
                if (result != null) { // We got a result
                    localizer.resetPosWithEstimate(result); // apply the new estimate to odometry and shut down the camera
                    currentPos = result;
                    getCameraEstimate = false;
                    visionPortal.setProcessorEnabled(aprilTag, false);
                    if (!visionPortal.getProcessorEnabled(tfod)) { // if we aren't actively running tensorflow, shut down the camera service (save loop time)
                        visionPortal.stopStreaming();
                        cameraMode = CameraMode.IDLE;
                    }
                } else {
                    //continue looping until we have an estimate
                }
            }
        } else if (cameraMode == CameraMode.APLS) {
            visionPortal.setProcessorEnabled(tfod, true);
            visionPortal.resumeStreaming();
            if (getCameraEstimate) {
                // do apls stuff
            }
        }

    }


    public void setCameraMode(CameraMode mode) {
        cameraMode = mode;
    }

    public void getCameraEstimate() {
        getCameraEstimate = true;
    }

    public double getSlidesLength() {
        return slides.getCurrentPosition() * slideTickToInch;
    }
    public void updateSlides() {

    }




    //1 Set internal transfer servos/motor power

    //set gantry power
    //2 Update gantry (gantry movement using pid stuff)


    //3 set slides power
    //4 Update slides (slides movement using pid stuff)

    //5 Either set lift power, or set intake stuff
    //6 Update lift


    //7 Set angles for all subsystems (used to be here)
    //8 Set ZeroPower, mode, and pos for all subsystems



    public double[] getAssemblyCoordinate() {                //Returns location of subsystems in 3D coordinate space

        double relativeXToPoint = Math.cos(Math.toRadians(90)-Math.toRadians(currentPos.getHeading()));
        double relativeYToPoint = Math.sin(Math.toRadians(90)-Math.toRadians(currentPos.getHeading()));
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        //double x = (gantrypos);
        //double z = (slidesLength * Math.sin(Math.toRadians(60))) + someOffset;
        double y = (slidesLength * Math.cos(Math.toRadians(60)));
        //return new double[] {(movementXPower*y)+(movementYPower*x)+ currentPos.getX(), (movementYPower*y)+(movementXPower*x) + currentPos.getY(), z};
        return new double[] {9, 3};
    }

    public void setAssemblyCoordinates(double x, double y, double z, double v4barAngle) {       //Sets subsystems to best possible 3D location
        double deltaX = x - currentPos.getX();
        double deltaY = y - currentPos.getY();
        double deltaTheta = Math.toDegrees(Math.atan(deltaY/deltaX));
        slidesTarget = 0;
    }


    public double getBatteryVoltage() {return batterylevel.getVoltage();}

    public void setDriveMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : driveMotors) {
            motor.setMode(runMode);
        }
    }

    public void setDriveZeroPower(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }


    public void updateDriveMotors() {
        leftFront.setPower(flp);
        leftRear.setPower(blp);
        rightRear.setPower(brp);
        rightFront.setPower(frp);
    }

    public static void driveXY(double x, double y, double t) {
        double frontLeftPower = (y + x + t);
        double backLeftPower = (y - x + t);
        double frontRightPower = (y + x - t);
        double backRightPower = (y - x - t);
        flp = -frontLeftPower;
        blp = -backLeftPower;
        frp = frontRightPower;
        brp = backRightPower;
    }

    public void referenceDrive(double x, double y, double t, boolean IMU) {
        double head;
        if (IMU) {
            head = getIMUHeading();
        } else {
            head = currentPos.getHeading();
        }
        double relativeXToPoint = Math.cos(Math.toRadians(90)-Math.toRadians(head));
        double relativeYToPoint = Math.sin(Math.toRadians(90)-Math.toRadians(head));

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        driveXY((movementXPower*y)+(movementYPower*x), (movementYPower*-y)+(movementXPower*x), t);
    }

    public void resetOdometry() {
        for (DcMotorEx motor : odometryEncoders) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double verticalPos() {return verticalLeft.getCurrentPosition();}
    public double horizontalPos() {return horizontal.getCurrentPosition();}



    public double getIMUHeading() {
        return imuheading;
    }

    public void resetIMUHeading() {
        imu.resetYaw();
    }



    public void waitAndUpdate(long time){
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time){
            update();
        }
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), -pose.getY(), ROBOT_RADIUS);

        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, -y1, x2, -y2);
    }


    /**
     * Runs calculations for the robots pose based on detected apriltags.
     */
    public Pose2d getAprilTagEstimate() {
        //TODO: the position estimate needs to account for the camera's offset on the robot.
        //      This will require some complex trig to account for the possible change in the robots angle

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double head;
        if (useIMU) {
            head = getIMUHeading();
        } else {
            head = currentPos.getHeading();
        }
        Pose2d offsetPose = new Pose2d((CAMERA_X_OFFSET*Math.cos(Math.toRadians(head)) + CAMERA_Y_OFFSET*Math.sin(Math.toRadians(head))),
                CAMERA_X_OFFSET*Math.sin(Math.toRadians(head)) + CAMERA_Y_OFFSET*Math.cos(Math.toRadians(head)),
                head
                );
        Pose2d tagEstimate = null;

        // Step through the list of detections
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                for (int i : Constants.VisionConstants.ACCEPTED_IDS) {
                    if (i==detection.id) {
                        if (tagEstimate == null) {
                            tagEstimate = new Pose2d(detection.ftcPose.x, detection.ftcPose.y, head);
                        } else { // if multiple tags are detected, take the average of the estiamtes
                            tagEstimate = new Pose2d((tagEstimate.getX()+detection.ftcPose.x)/2, (tagEstimate.getY()+detection.ftcPose.y)/2, head);
                        }
                    }
                }

            } else {
                // unknown tag ID
            }
        }
        if (tagEstimate == null) {
            return null;
        } else {
            return new Pose2d(tagEstimate.getX() - offsetPose.getX(), tagEstimate.getY() - offsetPose.getY(), head);
        }
    }





    // PURE PURSUIT FUNCTIONS //
    // robot angle is converted to radians

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

        for (int i = 0; i < allPoints.size() - 1; i++) {
            // Generate a field visualization of the path

            //overlay.strokeLine(allPoints.get(i).x, allPoints.get(i).y, allPoints.get(i + 1).x, allPoints.get(i + 1).y);
            overlay.strokeLine(allPoints.get(i).y, allPoints.get(i).x, allPoints.get(i + 1).y, allPoints.get(i + 1).x);
        }
        // Find the best point to follow
        CurvePoint followMe = getFollowPointPath(allPoints, currentPos, allPoints.get(0).followDistance);

        // Log follow point
        overlay.strokeCircle(followMe.x, followMe.y, 2);

        // Follow the optimal point
        goToAnotherPosition(currentPos, followMe.x, followMe.y, followMe.moveSpeed, followMe.targetAngle, followMe.turnSpeed, 2, followMe.end, followMe.slope);

    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Pose2d robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {       // For each individual point

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            // Look for intersections between the robot's projected circle and the path
            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(new Point(robotLocation.getX(),robotLocation.getY()), followRadius, startLine.toPoint(), endLine.toPoint());   // Do the pure pursuit math and develop target points


            double closestAngle = 10000000;
            // Look at every intersection to find which is the best on to follow
            for(Point thisIntersection : intersections) {
                double angle =  Math.atan2(thisIntersection.x - robotLocation.getX(), thisIntersection.y - robotLocation.getY());
                double deltaAngle = Math.abs(AngleWrap(angle - Math.toRadians(robotLocation.getHeading()) + pathPoints.get(i+1).targetAngle));
                // The intersection who's angle is closest to our relative target angle is the optimal point
                if(deltaAngle < closestAngle) {
                    if (pathPoints.get(i).end) {
                        followMe.setPoint(pathPoints.get(pathPoints.size()-2).toPoint());
                        followMe.moveSpeed = 0.4;
                    } else {
                        closestAngle = deltaAngle;
                        followMe.setPoint(thisIntersection);
                    }
                }
                // Create a point for the robot to follow
                followMe.end = pathPoints.get(i).end;
                followMe.targetAngle = pathPoints.get(i+1).targetAngle;
                followMe.slope = Math.atan2(pathPoints.get(i+1).x-pathPoints.get(i).x, pathPoints.get(i+1).y - pathPoints.get(i).y);
            }
        }
        return followMe;
    }

    /**
     * @param currentPos in degrees
     * @param x
     * @param y
     * @param movementSpeed
     * @param preferredAngle
     * @param turnSpeed
     * @param cutoffAtDistance
     */
    public static void goToAnotherPosition(Pose2d currentPos, double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, double cutoffAtDistance, boolean end, double slope) {

        double movement_x;
        double movement_y;
        double movement_turn;

        double currenthead_rad = Math.toRadians(currentPos.getHeading());
        double deltaX = x - currentPos.getX();
        double deltaY = y - currentPos.getY();
        double distanceToTarget = Math.hypot(deltaX, deltaY);
        double absoluteAngleToTarget = Math.atan2(deltaX , deltaY);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - currenthead_rad);

        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        // Strafing is hard (especially on a heavy robot). Help it out!
        if (movement_x > 0.1) {
            movement_x = Range.clip(movement_x + 0.2, movement_x, 1.0);
        } else if (movement_x < -0.1) {
            movement_x = Range.clip(movement_x-0.2, -1.0, movement_x);
        }

        double relativeTurnAngle;
        // If you are perfectly aligned (angle-wise) with the path, let strafing do the work
        // This prevents additional unneccesary turning when you are off the path in the x or y axis that could be done by strafing
        if (preferredAngle + slope == 0) {
            relativeTurnAngle = AngleWrap(-currenthead_rad);
        } else {   // If your angle isn't correct, compensate for it
            relativeTurnAngle = AngleWrap(relativeAngleToPoint + preferredAngle);
        }

        // If you are decently close to your final target point, stop moving
        if (distanceToTarget < cutoffAtDistance) {
            movement_x = 0;
            movement_y = 0;
            //movement_turn = 0;
        }

        // If you are approaching the final point, you know the final angle better than the trig does
        // Fix onto that final angle and make strafing do the work to approach the final point
        if (end) {
            movement_turn = Range.clip(AngleWrap((slope+preferredAngle) - currenthead_rad) / Math.toRadians(30), -1, 1) * turnSpeed;
        } else {    // Otherwise, follow the trig
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        }

        driveXY(movement_x, movement_y, movement_turn);
    }

}
