package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.MathFunctions.AngleWrap;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.Point;
import org.firstinspires.ftc.teamcode.drive.Constants.AssemblyConstants;
import org.firstinspires.ftc.teamcode.drive.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.vision.PDP_DualCamera;
import org.firstinspires.ftc.teamcode.vision.PDP_LeftCam;
import org.firstinspires.ftc.teamcode.DataTypes.General.*;
import org.firstinspires.ftc.teamcode.vision.ThreeZonePropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
public class RobotDriver {

    private DcMotorEx fl, bl, fr, br, verticalLeft, verticalRight, horizontal, slidesL, slidesR, intake, leftSlidesEnc;
    private List<DcMotorEx> driveMotors, odometryEncoders, slides;
    private IMU imu;
    private VoltageSensor batterylevel;
    private CRServoImplEx gantry;
    private Servo plunger, clawL, clawR, launcher, hangReleaseLeft, hangReleaseRight, clawLift, intakeLift, purpleRelease, purpleReleaseNorth;
    CRServo clawFlipper;
    private AnalogInput flipperEnc;
    private AnalogInput gantryEnc;
    private ColorSensor colorLeft;
    private RevColorSensorV3 colorRight;
    private AnalogInput fsr, distLeft, distRight, distFrontLeft, distFrontRight;
    //private ContinousAnalogAxon gantyEncoder;
    ServoImplEx flipperimpl;
    public int[] encoders;
    public Localizer localizer;
    public Pose2d CurrentVelocities = new Pose2d(), PreviousVelocities = new Pose2d();
    public static double[] globalCoords;
    static Pose2d currentPos = new Pose2d(0, 0, 0);
    private RevTouchSensor touch;
    private List<LynxModule> allHubs;
    private double slidesLength, touchVal, imuheading, gantryPos;
    double slidesTarget, slidesPower, intakePower, flipperTarget, flipperPower, flipperAngle;
    static double frp=0, flp=0, brp=0, blp=0;
    public boolean useIMU = false, slidesDisable, flipperDisable;
    final double slideTickToInch = AssemblyConstants.slideTickToInch;
    public int loops = 0;
    long lastLoopTime = System.nanoTime();
    public double loopSpeed = 0;
    public static double ROBOT_RADIUS = Constants.DriveConstants.ROBOT_RADIUS;
    public static double CONSTANT = DriveConstants.horizontalTickOffset;
    public Canvas overlay;
    private CameraMode cameraMode = CameraMode.IDLE;
    boolean cameraReady = false, getCameraEstimate = false;
    private static final double CAMERA_X_OFFSET = DriveConstants.CAMERA_X_OFFSET, CAMERA_Y_OFFSET = DriveConstants.CAMERA_Y_OFFSET;
    public double slidesPrimeTarget = AssemblyConstants.defaultSlideLength, slidesDepositTarget = 12;
    private LocalMode localizationMode;
    double slidesI, previousSlidesError, flipperI, previousFlipperError;
    private boolean waitingForDepsoit = false;
    ElapsedTime tagTimer, depositTimer, clawTimer, flipperTimer;
    public SpikePosition propLocation;
    IntakeMode intakeMode = IntakeMode.LOCK;
    PlungerMode plungerMode = PlungerMode.LOAD;
    ClawMode clawMode = ClawMode.OPEN;
    WeaponsState weaponsState = WeaponsState.HOLDING;
    FlipperState flipperState = FlipperState.STORED;
    Servo[] hangReleaseServos;
    OpenCvCamera PropCameraL, PropCameraR, cameraOfInterest;
    PDP_LeftCam pipelineLeft;
    PDP_DualCamera pipelineRight;
    Pose2d aprilTagPosition;
    ThreeZonePropDetectionPipeline propPipeline;
    boolean updateClaw = true;
    boolean openClawForPostDeposit = false;
    boolean isFlipperOut = false;
    private int tagOfInterest = 0;
    double flipperPosAnalog = 0;
    Pose2d depositPos;
    final FtcDashboard dashboard;
    double flipperAccelPower = 0;
    FlipperState previousFlipperState = FlipperState.IDLE;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean invertClaw = false, invertOtherClaw = false;
    private double previousFlipperTarget;
    boolean runningRawFlipper = false;
    double intakePos = 0, previousSlidesPower = 0, distLeftIn, distRightIn, distFrontLeftIn, distFrontRightIn;
    boolean speedyDeposit = false, convertCurPosToIMU = false;
    public RobotDriver(HardwareMap hardwareMap, boolean prepAutoCamera) {
        frp = 0;
        flp = 0;
        brp = 0;
        blp = 0;
        localizationMode = LocalMode.ODOMETRY;
        localizer = new Localizer();
        encoders = new int[localizer.encoders.length];

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");

        verticalLeft = hardwareMap.get(DcMotorEx.class, "fl");
        verticalRight = hardwareMap.get(DcMotorEx.class, "br");
        horizontal = hardwareMap.get(DcMotorEx.class, "fr");

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveMotors = Arrays.asList(fl, bl, fr, br);
        odometryEncoders = Arrays.asList(verticalLeft, verticalRight, horizontal);

        imu = hardwareMap.get(IMU.class, "adaIMU");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        //imu.resetYaw();

        // SLIDES
        slidesL = hardwareMap.get(DcMotorEx.class, "slidesL");
        slidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesR = hardwareMap.get(DcMotorEx.class, "slidesR");
        slidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides = Arrays.asList(slidesL, slidesR);
        leftSlidesEnc = hardwareMap.get(DcMotorEx.class, "bl");



        clawL = hardwareMap.get(Servo.class, "lclaw");
        clawR = hardwareMap.get(Servo.class, "rclaw");
        clawLift = hardwareMap.get(Servo.class, "clawLift");
        clawFlipper = hardwareMap.get(CRServo.class, "armLift");
        flipperEnc = hardwareMap.get(AnalogInput.class, "armAxon");
        launcher = hardwareMap.get(Servo.class, "launcher");
        hangReleaseLeft = hardwareMap.get(Servo.class, "hangReleaseLeft");
        hangReleaseRight = hardwareMap.get(Servo.class, "hangReleaseRight");
        hangReleaseServos = new Servo[] {hangReleaseLeft, hangReleaseRight};
        //clawFlipper.getController().pwmEnable();

        //flipperimpl = hardwareMap.get(ServoImplEx.class, "armLift");
        //flipperimpl.setPwmEnable();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(RevColorSensorV3.class, "colorRight");

        fsr = hardwareMap.analogInput.get("fsr");

        purpleRelease = hardwareMap.get(Servo.class, "purple");
        purpleReleaseNorth = hardwareMap.get(Servo.class, "purpleNorth");

        distLeft = hardwareMap.get(AnalogInput.class, "distLeft");
        distRight = hardwareMap.get(AnalogInput.class, "distRight");
        distFrontLeft = hardwareMap.get(AnalogInput.class, "distFrontLeft");
        distFrontRight = hardwareMap.get(AnalogInput.class, "distFrontRight");


        // INTAKE
        /*intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gantry = hardwareMap.get(CRServoImplEx.class, "gantry");
        gantryEnc = hardwareMap.get(AnalogInput.class, "gantryEnc");
        gantyEncoder = new ContinousAnalogAxon(gantryEnc);
        plunger = hardwareMap.get(Servo.class, "plunger");

         */

        //touch = hardwareMap.get(RevTouchSensor.class, "slidesTouch");

        batterylevel = hardwareMap.get(VoltageSensor.class, "Control Hub");


        //int[] portals = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        if (loadAlliancePreset() == AllianceLocation.BLUE_NORTH | loadAlliancePreset() == AllianceLocation.BLUE_SOUTH) {
            PropCameraR = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "PropCamR"), viewportContainerIds[1]);
            propPipeline = new ThreeZonePropDetectionPipeline(true, loadAlliancePreset());
            PropCameraR.setPipeline(propPipeline);
        } else {
            PropCameraL = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "PropCamL"), viewportContainerIds[0]);
            propPipeline = new ThreeZonePropDetectionPipeline(true, loadAlliancePreset());
            PropCameraL.setPipeline(propPipeline);
        }


        if (prepAutoCamera) {
            if (loadAlliancePreset() == AllianceLocation.BLUE_NORTH | loadAlliancePreset() == AllianceLocation.BLUE_SOUTH) {
                cameraOfInterest = PropCameraR;
            } else {
                cameraOfInterest = PropCameraL;
            }
            cameraOfInterest.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    cameraOfInterest.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                    cameraReady = true;
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            cameraMode = CameraMode.PROP;
        }
        */

        //aprilTag = new AprilTagProcessor.Builder()
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

                //.build();

        //VisionPortal.Builder builder = new VisionPortal.Builder();
        //builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
         //builder.setCameraResolution(new Size(640, 480));

        // Set and enable the processor.
        //builder.addProcessor(aprilTag);
        //builder.setLiveViewContainerId(viewportContainerIds[1]);
        // Build the Vision Portal, using the above settings.
        //visionPortal = builder.build();

        //allHubs = hardwareMap.getAll(LynxModule.class);
        /*for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }*/
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        //expansionHub.setBulkCachingMode();

        dashboard = FtcDashboard.getInstance();
        tagTimer = new ElapsedTime();
        depositTimer = new ElapsedTime();
        clawTimer = new ElapsedTime();
        flipperTimer = new ElapsedTime();
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
        updateCamera();             // Updates current requested camera operation (if any)
        updateDriveMotors();        // Updates drive motor power
        updateAutomation();         // Updates quick commands
        updateIntake();             // Updates intake controls
        updateFlipper();            // Updates flipper controls
        if (updateClaw) {
            updateClaw();           // Updates claw controls
        }
        updateSlides();             // Updates slides controls

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Pulling APRILTAG Localization", localizationMode.equals(LocalMode.APRILTAG));
        packet.put("x", currentPos.getY());
        packet.put("y", currentPos.getX());
        packet.put("heading (deg)", currentPos.getHeading());
        if (useIMU) {
            packet.put("heading (imu)", imuheading);
        }
        packet.put("slides", slidesLength);
        packet.put("loop speed", loopSpeed);
        packet.put("Normalized angle", (currentPos.getHeading()%90));
        //packet.put("camera of interest", cameraOfInterest.);

        overlay = packet.fieldOverlay();

        drawRobot(overlay, new Pose2d(currentPos.getY(), currentPos.getX(), Math.toRadians(currentPos.getHeading())));     //Draw the robot's position on the diagram

        dashboard.sendTelemetryPacket(packet);          // Send field overlay and telemetry data to FTCDashboard
    }

    public void updateEstimate(){           // Updates all encoders and re-estimates localization
        getSensors();
        globalCoords = getAssemblyCoordinate();
        if (localizationMode == LocalMode.ODOMETRY) {
            localizer.updateEncoders(encoders);
            localizer.update(loopSpeed);
            currentPos = localizer.getPosEstimate();
            PreviousVelocities = CurrentVelocities;
            CurrentVelocities = localizer.getvelocity();
            if (convertCurPosToIMU) {
                currentPos = new Pose2d(currentPos.getX(), currentPos.getY(), pullIMUHeading());
            }
        } else if (localizationMode == LocalMode.FUSED) {
            localizer.updateEncoders(encoders);
            localizer.update(loopSpeed);
            currentPos = localizer.getPosEstimate();
            PreviousVelocities = CurrentVelocities;
            CurrentVelocities = localizer.getvelocity();
            setCameraMode(CameraMode.APRILTAG);
            getCameraEstimate();
            if (aprilTagPosition != null) {
                currentPos = new Pose2d(mean(currentPos.getX(), aprilTagPosition.getX()), mean(currentPos.getY(), aprilTagPosition.getY()), currentPos.getHeading());
            }
            /*if (tagTimer.time() >= 15) { // pull an AprilTag reading every 15 seconds
                tagTimer.reset();
                setCameraMode(CameraMode.APRILTAG);
                getCameraEstimate();

            }
             */
        } else if (localizationMode == LocalMode.APRILTAG) {
            cameraMode = CameraMode.APRILTAG; // the updateCamera() method (called later) will handle everything from here
            currentPos = aprilTagPosition;
        }

    }

    public void setUseIMUForLocalization(boolean bool) {
        convertCurPosToIMU = bool;
    }

    public void updateAutomation() {
        switch (weaponsState) {
            /*case PRIMED: // flipped over, no slides
                slidesTarget = 0;//slidesPrimeTarget
                clawMode = ClawMode.PRIMED;
                weaponsState = WeaponsState.IDLE;
                flipperState = FlipperState.READY;
                intakeMode = IntakeMode.LOCK;
                break;

             */
            case EXTEND: // slides out, flipped over, ready to deposit
                slidesTarget = slidesDepositTarget;
                clawMode=ClawMode.BOTH;
                setClawLiftPos(true);
                weaponsState=WeaponsState.IDLE;
                flipperState = FlipperState.READY;
                intakeMode = IntakeMode.LOCK;
                break;
            case DEPOSIT: // release claw, flip thing back, slides down
                depositPos = currentPos;
                clawMode = ClawMode.OPEN;
                depositTimer.reset();
                waitingForDepsoit = true;
                weaponsState=WeaponsState.IDLE;
                flipperState = FlipperState.IDLE;
                intakeMode = IntakeMode.LOCK;
                break;
            case INTAKING: // rollers on, ready to grab
                slidesTarget = 0;
                setClawLiftPos(false);
                if (getRightHasPixel()) { // dont open a claw that has a pixel in it
                    if (getLeftHasPixel()) {
                        clawMode = ClawMode.BOTH;
                    } else {
                        clawMode = ClawMode.RIGHT;
                    }
                } else {
                    if (getLeftHasPixel()) {
                        clawMode = ClawMode.LEFT;
                    } else {
                        clawMode = ClawMode.OPEN;
                    }
                }
                weaponsState = WeaponsState.IDLE;
                flipperState = FlipperState.STORED;
                intakeMode = IntakeMode.INTAKE;
                break;
            case HOLDING: // grab, invert rollers to spit out extras
                if (openClawForPostDeposit) { // dont grab if you just outtaked
                    clawMode = ClawMode.OPEN;
                    openClawForPostDeposit = false;
                } else {
                    clawMode = ClawMode.BOTH;
                }
                setClawLiftPos(false);
                slidesTarget = 0;
                weaponsState = WeaponsState.IDLE;
                flipperState = FlipperState.STORED;
                intakeMode = IntakeMode.LOCK;
                break;
            case MANUAL: // do whatever crazy thing the drivers want to do
                //TODO: set pos to whatever is desired
                weaponsState = WeaponsState.IDLE;
                break;
            case IDLE: // normal resting state, holding positions
                if (waitingForDepsoit && (speedyDeposit || !isInRange(currentPos.getY(), depositPos.getY()-2, depositPos.getY()+2) || !isInRange(currentPos.getHeading(), depositPos.getHeading()-20, depositPos.getHeading()+20))) { //(!isInRange(currentPos.getX(), depositPos.getX()-2, depositPos.getX()+2)
                    if (depositTimer.time() > 0.2 && waitingForDepsoit) { // waits to flip claw until the flipper has moved sufficiently
                        setClawLiftPos(false);
                    }
                    if (depositTimer.time() > 0.4 && waitingForDepsoit) { // waits for pixels to drop before folding up
                        waitingForDepsoit = false;
                        weaponsState = WeaponsState.HOLDING;
                        clawMode = ClawMode.OPEN;
                        openClawForPostDeposit = true;
                    }
                }
                break;
                //do nothing, as all parameters are now set
        }
    }
    public WeaponsState getWeaponsState() {return weaponsState;}

    public void setSpeedyDeposit(boolean speed) {
        speedyDeposit = speed;
    }

    public AllianceLocation loadAlliancePreset() {
        Reader r = new Reader();
        String info = r.readFile("Alliance");
        AllianceLocation location=AllianceLocation.NONE;
        switch (info) {
            case "blue_south":
                location = AllianceLocation.BLUE_SOUTH;
                break;
            case "blue_north":
                location = AllianceLocation.BLUE_NORTH;
                break;
            case "red_south":
                location = AllianceLocation.RED_SOUTH;
                break;
            case "red_north":
                location = AllianceLocation.RED_NORTH;
                break;
        }
        return location;
    }
    public ParkLocation loadParkPreset() {
        Reader r = new Reader();
        String info = r.readFile("park");
        ParkLocation location=ParkLocation.NONE;
        switch (info) {
            case "left":
                location = ParkLocation.LEFT;
                break;
            case "right":
                location = ParkLocation.RIGHT;
                break;
            case "center":
                location = ParkLocation.CENTER;
                break;
        }
        return location;
    }
    public double loadTimerPreset1() {
        Reader r = new Reader();
        return Double.parseDouble(r.readFile("timer"));
    }
    public double loadTimerPreset2() {
        Reader r = new Reader();
        return Double.parseDouble(r.readFile("timer2"));
    }
    public double loadTimerPreset3() {
        Reader r = new Reader();
        return Double.parseDouble(r.readFile("timer3"));
    }
    public boolean loadParkOnWall() {
        Reader r = new Reader();
        return r.readFile("parkspot").equals("wall");
    }
    public boolean loadSlidesUpPreset() {
        Reader r = new Reader();
        return r.readFile("slidespos").equals("up");
    }


    public Pose2d getCurrentPos() {
        return currentPos;
    }
    public Pose2d getCurrentVelocities() {return CurrentVelocities;}

    public void getSensors() {             // Retrieves all encoder data
        encoders[0] = -verticalLeft.getCurrentPosition();
        encoders[2] = horizontal.getCurrentPosition();
        encoders[1] = -verticalRight.getCurrentPosition();
        slidesLength = leftSlidesEnc.getCurrentPosition()/slideTickToInch;
        //flipperAngle = flipper.getCurrentPosition();
        flipperAngle = 0;
        flipperPosAnalog = flipperEnc.getVoltage()/3.3*360-AssemblyConstants.FLIPPER_ENCODER_DEGREES_OFFSET; //TODO: if the servo is replaced, this will need changed
        intakePos = intake.getCurrentPosition();
        //gantryPos = gantyEncoder.getCurrentPosition();
        //touchVal = touch.getValue();
        if (useIMU) {
            pullIMUHeading();
        }
    }


    /**
     * Handles ALL camera functions. Switching between detection states is done with a variable
     */
    public void updateCamera() {
        if (cameraMode == CameraMode.PROP) {
            if (!cameraReady) {
                cameraOfInterest.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        cameraOfInterest.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                        cameraReady = true;
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                    }
                });

                propLocation = propPipeline.getAnalysis(); //get the estimate
                //OpenCvCam.stopStreaming();
                //OpenCvCam.closeCameraDevice();
                //cameraReady = false;
                //cameraMode = CameraMode.IDLE;
            }
            propLocation = propPipeline.getAnalysis();

        } else if (cameraMode == CameraMode.APRILTAG) {
            visionPortal.setProcessorEnabled(aprilTag, true);
            visionPortal.resumeStreaming();
            if (getCameraEstimate || localizationMode == LocalMode.APRILTAG) {
                Pose2d result = getAprilTagEstimate();
                if (result != null) { // We got a result
                    //localizer.resetPosWithEstimate(result); // apply the new estimate to odometry and shut down the camera
                    //currentPos = result;
                    aprilTagPosition = result;
                    getCameraEstimate = false;
                    if (localizationMode != LocalMode.APRILTAG) { // Stop streaming unless if we are constantly getting localization data
                        //visionPortal.setProcessorEnabled(aprilTag, false);
                        //visionPortal.stopStreaming();
                        //cameraMode = CameraMode.IDLE;
                    }
                } else {
                    //continue looping until we have an estimate
                }
            }
        } else if (cameraMode == CameraMode.APLS) {
            visionPortal.setProcessorEnabled(aprilTag, true);
            visionPortal.resumeStreaming();
            if (getCameraEstimate) {
                // do apls stuff
                //TODO: Get the tag location and translate that into a bounding box of the backdrop
            }
        } else if (cameraMode == CameraMode.IDLE) {
            if (cameraReady) {
                cameraOfInterest.stopStreaming();
                cameraOfInterest.closeCameraDevice();
                cameraReady = false;
            }
        }

    }
    public void stopStreamingVP() {
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.stopStreaming();
        cameraMode = CameraMode.IDLE;
    }

    public Pose2d getAprilTagPosition() {
        return aprilTagPosition;
    }


    public void setCameraMode(CameraMode mode) {cameraMode = mode;}
    public void getCameraEstimate() {getCameraEstimate = true;}
    public void setLocalizationMode(LocalMode mode) {localizationMode = mode;}

    public void setSlidesDisable(boolean disable) {slidesDisable = disable;}
    public boolean getSlidesDisable() {return slidesDisable;}
    public double getSlidesLength() {
        return slidesLength;
    }
    public void setSlidesTarget(double target) {
        slidesTarget = target;
    }
    public void setSlidesPower(double power) {
        slidesPower = power;
    }
    public void updateSlides() {
        if (slidesPower == 0) {
            if (!slidesDisable) {

                if (previousSlidesPower != 0) {
                    slidesTarget = slidesLength;
                }

                if (slidesTarget<0) {
                    slidesTarget=0;
                }

                //DO PID STUFF
                double error = (slidesTarget - slidesLength);
                double p = AssemblyConstants.slidesPIDConstants.p * error;
                slidesI += AssemblyConstants.slidesPIDConstants.i * error * loopSpeed;
                double d = AssemblyConstants.slidesPIDConstants.d * (error - previousSlidesError) / loopSpeed;
                double power = (p + slidesI + d + (Math.signum(error) * AssemblyConstants.slidesPIDConstants.f));
                for (DcMotorEx slide : slides) {
                    slide.setPower(-power);
                }

                previousSlidesError = error;
            } else {
                for (DcMotorEx slide : slides) {
                    slide.setPower(0);
                }
            }
        } else {
            if ((slidesLength <= 0 && slidesPower < 0) || (slidesLength > 0)) {
                for (DcMotorEx slide : slides) {
                    slide.setPower(slidesPower);
                }
            } else {
                for (DcMotorEx slide : slides) {
                    slide.setPower(0);
                }
            }
            slidesTarget = slidesLength; //TODO: this will make the PID one loop of error behind. This will need fixed if bounce-back is a problem.
        }
        previousSlidesPower = slidesPower;
    }
    public void resetSlidesEncoder() {
        for (DcMotorEx slide: slides) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftSlidesEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlidesEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setSlidesDepositTarget(double target) {
        slidesDepositTarget=target;
    }
    public double getSlidesTarget() {
        return slidesTarget;
    }

    public void setIntakeMode(IntakeMode mode) {intakeMode = mode;}
    public void setIntakePower(double power) {
        intakeMode = IntakeMode.MANUAL;
        intakePower = power;
    }
    public void updateIntake() {
        switch (intakeMode) {
            case LOCK:
                intake.setPower(0);
                break;
            case INTAKE:
                intake.setPower(1.0);
                break;
            case OUTTAKE:
                intake.setPower(-1.0);
                break;
            case MANUAL:
                intake.setPower(intakePower);
                break;
        }
    }
    public IntakeMode getIntakeMode() {return intakeMode;}
    public double getIntakeCurrent() {return intake.getCurrent(CurrentUnit.AMPS);}
    public double getIntakePos() {return intakePos;}

    public void setFlipperDisable(boolean b) {flipperDisable=b;}
    public boolean getFlipperDisable() {return flipperDisable;}
    public void setFlipperState(FlipperState state) {flipperState = state;}
    public void setFlipperPower(double power) {flipperPower=power;}
    public double getFLipperPos() {
        return flipperAngle;
    }
    public double getFlipperPosAnalog() {return flipperPosAnalog;}
    public FlipperState getFlipperState() {return flipperState;}
    public double getFlipperTarget() {return flipperTarget;}
    public void updateFlipper() {
        switch (flipperState) {
            case STORED:
                //flipperTarget = 0;
                flipperTarget = 60;
                runningRawFlipper = false;
                //clawFlipper.setPosition(0.299);
                break;
            case READY:
                flipperTarget = -60;
                runningRawFlipper = false;
                //flipperTarget = 300;
                //clawFlipper.setPosition(0.765);
                break;
            case IDLE:

                break;
        }
        if (flipperState != previousFlipperState) {
            flipperAccelPower = 0;
           // runningRawFlipper = false;
        }
        if (previousFlipperTarget != flipperTarget) {
            // you have changed states
            flipperTimer.reset();
        }
        if (flipperPower==0) {
            if (!flipperDisable) {
                //3.02, 1.78

                if (!runningRawFlipper) {
                    double error = (flipperTarget - flipperPosAnalog);
                    double p = AssemblyConstants.flipperPIDConstants.p * error;
                    double f = AssemblyConstants.flipperPIDConstants.f * Math.sin(Math.toRadians(flipperPosAnalog));
                    double d = AssemblyConstants.flipperPIDConstants.d * (error - previousFlipperError) / loopSpeed;

                    previousFlipperError = error;


                    if (flipperTarget == 60 && flipperPosAnalog > 55) {
                        clawFlipper.setPower(0);
                    } else if (flipperTarget == -60 && flipperPosAnalog < -55) {
                        clawFlipper.setPower(0);
                    } else {
                        clawFlipper.setPower(p + d + f);
                    }
                } else {
                    clawFlipper.setPower(0);
                }


                //clawFlipper.setPower(p+d+f);


                /*double error = (flipperTarget - flipperAngle);
                double p = AssemblyConstants.flipperPIDConstants.p * error;
                flipperI += AssemblyConstants.flipperPIDConstants.i * error * loopSpeed;
                double d = AssemblyConstants.flipperPIDConstants.d * (error - previousFlipperError) / loopSpeed;
                double power;
                if (flipperAngle > 180) {
                    power = (p + flipperI + d + AssemblyConstants.flipperPIDConstants.f);
                } else {
                    power = (p + flipperI + d);
                }
                previousFlipperError = error;
                flipper.setPower(power);
                 */
//2.5, 1.1
                /*if (flipperTarget == 300) {
                    //clawFlipper.setPosition(0.765);
                    if (flipperPosAnalog > 2.1) {
                        if (flipperPosAnalog > 2.2) {
                            flipperAccelPower += 0.04;
                            if (flipperAccelPower > 0.4) {
                                clawFlipper.setPower(0.4);
                            } else {
                                clawFlipper.setPower(flipperAccelPower);
                            }
                        } else {
                            flipperAccelPower += 0.01;
                            if (flipperAccelPower > 0.6) {
                                clawFlipper.setPower(0.6);
                            } else {
                                clawFlipper.setPower(flipperAccelPower);
                            }
                        }
                    } else {
                        if (flipperPosAnalog < 1.3) {
                            clawFlipper.setPower(0);
                        } else {
                            flipperAccelPower -= 0.17;
                            if (flipperAccelPower < 0) {
                                clawFlipper.setPower(0);
                            } else {
                                clawFlipper.setPower(flipperAccelPower);
                            }
                            //clawFlipper.setPower(0);
                        }
                    }
                } else if (flipperState != FlipperState.IDLE) {
                    //clawFlipper.setPosition(0.299);
                    if (flipperPosAnalog < 1.7) {
                        if (flipperPosAnalog < 1.5) {
                            flipperAccelPower -= 0.04;
                            if (flipperAccelPower < -0.4) {
                                clawFlipper.setPower(-0.4);
                            } else {
                                clawFlipper.setPower(flipperAccelPower);
                            }
                        } else {
                            flipperAccelPower -= 0.01;
                            if (flipperAccelPower < -0.7) {
                                clawFlipper.setPower(-0.7);
                            } else {
                                clawFlipper.setPower(flipperAccelPower);
                            }
                        }
                    } else {
                        if (flipperPosAnalog > 2.1) {
                            clawFlipper.setPower(0);
                        } else {

                            flipperAccelPower += 0.15;
                            if (flipperAccelPower > 0) {
                                clawFlipper.setPower(0);
                            } else {
                                clawFlipper.setPower(flipperAccelPower);
                            }
                        }
                    }
                } else {
                    clawFlipper.setPower(0);
                }

                 */




                /*if (flipperTarget==300) {
                    if (flipperAngle < 160) { //(flipperAngle < 180)(flipperTimer.time() < 0.25)
                        flipper.setPower(-1.0);
                    } else {
                        flipper.setPower(0);
                    }
                } else {
                    if (flipperAngle > 150) { // flipperAngle > 180    (flipperTimer.time() < 0.2)
                        flipper.setPower(0.7);
                    } else {
                        flipper.setPower(0);
                    }
                }

                 */

            } else {
                //flipper.setPower(0);
            }
            previousFlipperTarget = flipperTarget;
        } else {
            //clawFlipper.setPosition(clawFlipper.getPosition()+flipperPower/100);
            clawFlipper.setPower(flipperPower);
            runningRawFlipper = true;
            //flipper.setPower(-flipperPower);
            //flipperTarget=flipperAngle;
            /*if (flipperPower < 0) {
                flipperTarget = 0;
            } else {
                flipperTarget = 300;
            }

             */
            flipperState = FlipperState.IDLE;
            flipperTarget = flipperPosAnalog;
        }
        previousFlipperState = flipperState;
    }
    public void resetFlipperEncoder() {
        //flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setClawMode(ClawMode mode) {
        clawMode = mode;
    }
    public ClawMode getClawMode() {
        return clawMode;
    }
    public void updateClaw() { // updates claw positoins
        switch (clawMode) {
            case LEFT:
                clawL.setPosition(AssemblyConstants.LEFT_CLAW_CLOSED_POS); //CLOSED (left)
                clawR.setPosition(AssemblyConstants.RIGHT_CLAW_OPEN_POS); // OPEN (right)
                break;
            case RIGHT:
                clawR.setPosition(AssemblyConstants.RIGHT_CLAW_CLOSED_POS); //CLOSED (right)
                clawL.setPosition(AssemblyConstants.LEFT_CLAW_OPEN_POS); // OPEN (left)
                break;
            case BOTH:
                clawL.setPosition(AssemblyConstants.LEFT_CLAW_CLOSED_POS); // CLOSED (left)
                clawR.setPosition(AssemblyConstants.RIGHT_CLAW_CLOSED_POS); // CLOSED (right)
                break;
            case OPEN:
                clawL.setPosition(AssemblyConstants.LEFT_CLAW_OPEN_POS); // OPEN (LEFT)
                clawR.setPosition(AssemblyConstants.RIGHT_CLAW_OPEN_POS); // OPEN (right)
                break;
            /*case PRIMED:  
                // ensure both claws are grabbed, and lift the thingy
                clawLift.setPosition(0.65);
                clawL.setPosition(0.5);
                clawR.setPosition(0.5);
                break;
            case INTAKING:
                // lift down, both claws open
                clawLift.setPosition(0.06);
                clawL.setPosition(0.7);
                clawR.setPosition(0.3);
            case IDLE:
                break;

             */
        }
        if (clawTimer.time()>0.28 && invertClaw) {
            invertClaw = false;
            clawLift.setPosition(0.226); //0.226 up
        }
        if (clawTimer.time()>0.01 && invertOtherClaw) {
            invertOtherClaw = false;
            clawLift.setPosition(0.7975); // down
        }

    }
    public void setClawLPos(boolean closed) {
        if (closed) {
            clawL.setPosition(0.5);
        } else {
            clawL.setPosition(0.7);
        }
    }
    public void setClawRPos(boolean closed) {
        if (closed) {
            clawR.setPosition(0.5);
        } else {
            clawR.setPosition(0.3);
        }
    }
    public void setClawLRaw(double pos) {
        clawL.setPosition(pos);
    }
    public void setCLawRRaw(double pos) {
        clawR.setPosition(pos);
    }
    public void setClawLiftPos(boolean up) {
        if (up) {
            //clawLift.setPosition(0.9);
            clawTimer.reset();
            invertClaw = true;
            //clawLift.setPosition(0.226);
        } else {
            clawTimer.reset();
            invertOtherClaw = true;
            //clawLift.setPosition(0.7875);
        }

    }
    public void updateClaw(boolean update) {
        updateClaw = update;
    }
    public double getClawLPos() {
        return clawL.getPosition();
    }
    public double getClawRPos() {
        return clawR.getPosition();
    }
    public double getClawLiftPos() {
        return clawLift.getPosition();
    }

    public void setWeaponsState(WeaponsState mode) {
        weaponsState = mode;
    }

    public void launchHang() {
        hangReleaseRight.setPosition(1.0);
        hangReleaseLeft.setPosition(0.2);
    }
    public void storeHang() {
        hangReleaseRight.setPosition(0.52);
        hangReleaseLeft.setPosition(0.67);
    }

    public void launchPlane() {
        launcher.setPosition(AssemblyConstants.DRONE_LAUNCHER_RELEASE_POS);
    }
    public void storePlane() {
        launcher.setPosition(AssemblyConstants.DRONE_LAUNCHER_STORED_POS);
    }

    public void setPurpleSouthRelease(boolean val) {
        if (val) {
            purpleRelease.setPosition(1);
        } else {
            purpleRelease.setPosition(0.72);
        }
    }

    public void setPurpleNorthRelease(boolean val) {
        if (val) {
            purpleReleaseNorth.setPosition(1.0);
        } else {
            purpleReleaseNorth.setPosition(0.72);
        }
    }


    public void storeAll() {
        storePlane();
        storeHang();
    }


    public int[] getLeftColor() {
        //return new int[] {colorLeft.red(), colorLeft.green(), colorLeft.blue()};
        //return new int[] {0, 0, 0};
        int argb = colorLeft.argb();
        int rgb[] = new int[] {
                (argb >> 16) & 0xff, //red
                (argb >>  8) & 0xff, //green
                (argb      ) & 0xff  //blue
        };
        return rgb;
    }
    public int[] getRightColor() {
        //return new int[] {colorRight.red(), colorRight.green(), colorRight.blue()};
        //return new int[] {0, 0, 0};
        int argb = colorRight.argb();
        int rgb[] = new int[] {
                (argb >> 16) & 0xff, //red
                (argb >>  8) & 0xff, //green
                (argb      ) & 0xff  //blue
        };
        return rgb;
    }
    public boolean getLeftHasPixel() {
        //colorLeft.argb();
        //return (colorLeft.red()+colorLeft.green()+colorLeft.blue() > 600);
        int argb = colorLeft.argb();
        int rgb[] = new int[] {
                (argb >> 16) & 0xff, //red
                (argb >>  8) & 0xff, //green
                (argb      ) & 0xff  //blue
        };
        return (rgb[0]+','+rgb[1]+','+rgb[2] > 93);
    }
    public boolean getRightHasPixel() {
        //return (colorRight.red()+colorRight.green()+colorRight.blue() > 600);
        int argb = colorRight.argb();
        int rgb[] = new int[] {
                (argb >> 16) & 0xff, //red
                (argb >>  8) & 0xff, //green
                (argb      ) & 0xff  //blue
        };
        return (rgb[0]+','+rgb[1]+','+rgb[2] > 100);
    }

    public double getFSRVoltage() {
        return fsr.getVoltage();
    }
    public boolean getFSRPressed() {
        return (fsr.getVoltage() > 0.4);
    }

    public double getUltraL() {
        return distLeft.getVoltage()*1000/3.2;
    }
    public double getUltraFL() {
        return distFrontLeft.getVoltage()*1000/3.2;
    }
    public double getUltraFR() {
        return distFrontRight.getVoltage()*1000/3.2;
    }
    public double getUltraR() {
        return distRight.getVoltage()*1000/3.2;
    }
    /**
     * Returns all of the voltages in an array
     * @return list of raw voltages. Indexes: 0-left, 1-frontLeft, 2-frontRight, 3-right
     */
    public double[] getDistVoltages() {
        return new double[] {distLeft.getVoltage(), distFrontLeft.getVoltage(), distFrontRight.getVoltage(), distRight.getVoltage()};
    }

    /**
     * Returns all of the distances in an array
     * @return list of distances. Indexes: 0-left, 1-frontLeft, 2-frontRight, 3-right
     */
    public double[] getDistances() {
        return new double[] {distLeft.getVoltage()*1000/3.2, distFrontLeft.getVoltage()*1000/3.2, distFrontRight.getVoltage()*1000/3.2, distRight.getVoltage()*1000/3.2};
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


        double x = (gantryPos);
        double z = (slidesLength * Math.sin(Math.toRadians(60))); // TODO: add distance of vertex of slides to the ground
        double y = (slidesLength * Math.cos(Math.toRadians(60)));
        return new double[] {(x*Math.cos(Math.toRadians(currentPos.getHeading()))) + (y*Math.sin(Math.toRadians(currentPos.getHeading())))+ currentPos.getX(), -(x*Math.sin(Math.toRadians(currentPos.getHeading()))) + (y*Math.cos(Math.toRadians(currentPos.getHeading()))) + currentPos.getY(), z};
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
        fl.setPower(flp);
        bl.setPower(blp);
        br.setPower(brp);
        fr.setPower(frp);
    }

    public static void driveXY(double x, double y, double t) {
        double frontLeftPower = (y + x + t);
        double backLeftPower = (y - x + t);
        double frontRightPower = (y - x - t);
        double backRightPower = (y + x - t);
        flp = frontLeftPower;
        blp = backLeftPower;
        frp = frontRightPower;
        brp = backRightPower;
    }

    public void drive(double x, double y, double t, boolean fieldCentric) {
        if (fieldCentric) {
            double head = currentPos.getHeading(); // No matter what localization method we are using, this reading will be what we want (either the IMU or odometry)
            double relativeXToPoint = Math.cos(Math.toRadians(90) - Math.toRadians(head));
            double relativeYToPoint = Math.sin(Math.toRadians(90) - Math.toRadians(head));

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            driveXY((movementXPower * -y) + (movementYPower * x), (movementYPower * y) + (movementXPower * x), t);
        } else {
            driveXY(x, y, t);
        }
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
    public double pullIMUHeading() {
        imuheading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return imuheading;
    }

    public void resetIMUHeading() {
        imu.resetYaw();
    }


    public void getLeftDist() {

    }



    public void waitAndUpdate(long time){
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time){
            update();
        }
    }
    public void waitAndUpdateWithPath(long time, ArrayList<CurvePoint> path){
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time){
            followCurve(path);
            update();
        }
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), -pose.getY(), ROBOT_RADIUS);

        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, -y1, x2, -y2);
        //canvas.strokeCircle(globalCoords[0], globalCoords[1], 2);
    }


    /**
     * Runs calculations for the robots pose based on detected apriltags.
     */
    public Pose2d getAprilTagEstimate() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Pose2d tagEstimate = null;
        // Step through the list of detections
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                for (int i : Constants.VisionConstants.ACCEPTED_IDS) { // ensure that we are not looking at the tags on the wall... those will be inaccurate
                    if (i == detection.id) { // if a detected tag is a permissible one
                        if (tagEstimate == null) {
                            tagEstimate = new Pose2d(72 - detection.metadata.fieldPosition.get(1) - detection.ftcPose.x, detection.metadata.fieldPosition.get(0) + 72 - detection.ftcPose.y, currentPos.getHeading());
                        } else {
                            tagEstimate = new Pose2d(mean(72 - detection.metadata.fieldPosition.get(1) - detection.ftcPose.x, tagEstimate.getX()), mean(detection.metadata.fieldPosition.get(0) + 72 - detection.ftcPose.y, tagEstimate.getY()), currentPos.getHeading());
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
            //return new Pose2d(tagEstimate.getX() - offsetPose.getX(), tagEstimate.getY() - offsetPose.getY(), head);
            return tagEstimate;
        }
    }
    public void setTagOfInterest(int tag) {
        tagOfInterest = tag;
    }


    public boolean runAutoPath(ArrayList<CurvePoint> path) {
        followCurve(path);
        if (Math.abs(currentPos.getX() - path.get(path.size() - 2).x) <= 1 &&
                Math.abs(currentPos.getY() - path.get(path.size() - 2).y) <= 1)
        {
            // We are at our desired position
            // bump index and apply it to the new trajectory
            //waitAndUpdate(1000, currentTrajectory);
            return true;
        } else {
            return false;
        }
    }



    // PURE PURSUIT FUNCTIONS //
    // robot angle is converted to radians

    public void followCurve(ArrayList<CurvePoint> allPoints) {

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

    public void turnInPlace(double targetAngle, boolean useIMU, double turnSpeed) {
        if (useIMU) {
            pullIMUHeading();
            driveXY(0, 0, Range.clip(AngleWrap(Math.toRadians(targetAngle) - Math.toRadians(imuheading)) / Math.toRadians(30), -1, 1) * turnSpeed);
        } else {
            driveXY(0, 0, Range.clip(AngleWrap(Math.toRadians(targetAngle) - Math.toRadians(currentPos.getHeading())) / Math.toRadians(30), -1, 1) * turnSpeed);
        }
    }


    private static boolean isInRange(double number, double lowerBound, double upperBound) {
        return (lowerBound < number && number < upperBound);
    }

    private static double mean(double n1, double n2) {
        return (n1+n2)/2;
    }

}
