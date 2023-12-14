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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.Point;
import org.firstinspires.ftc.teamcode.drive.Constants.AssemblyConstants;
import org.firstinspires.ftc.teamcode.drive.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Sensors.ContinousAnalogAxon;
import org.firstinspires.ftc.teamcode.vision.PDP_DualCamera;
import org.firstinspires.ftc.teamcode.vision.PDP_LeftCam;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline_DualZone;
import org.firstinspires.ftc.teamcode.DataTypes.General.*;
import org.firstinspires.ftc.teamcode.vision.ThreeZonePropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
public class RobotDriver {

    private DcMotorEx fl, bl, fr, br, verticalLeft, verticalRight, horizontal, slidesL, slidesR, intake, leftSlidesEnc;
    private List<DcMotorEx> driveMotors, odometryEncoders, slides;
    private IMU imu;
    private VoltageSensor batterylevel;
    private CRServoImplEx gantry;
    private Servo plunger, clawL, clawR, launcher, hangReleaseLeft, hangReleaseRight, pancake, clawLift;
    private AnalogInput distLeft, distRight;
    private AnalogInput gantryEnc;
    //private ContinousAnalogAxon gantyEncoder;
    public int[] encoders;
    public Localizer localizer;
    public Pose2d CurrentVelocities = new Pose2d(), PreviousVelocities  = new Pose2d();
    public static double[] globalCoords;
    static Pose2d currentPos = new Pose2d(0, 0, 0);
    private RevTouchSensor touch;
    private List<LynxModule> allHubs;
    private double slidesLength, touchVal, imuheading, gantryPos;
    double slidesTarget, slidesPower, gantryTarget, gantryPower, intakePower, plungerPos;
    static double frp=0, flp=0, brp=0, blp=0;
    public boolean useIMU = false, slidesDisable;
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
    double slidesI, previousSlidesError;
    private boolean waitingForDepsoit = false;
    ElapsedTime tagTimer, depositTimer;
    public SpikePosition propLocation;
    IntakeMode intakeMode = IntakeMode.LOCK;
    PlungerMode plungerMode = PlungerMode.LOAD;
    ClawMode clawMode = ClawMode.RELEASE_BOTH;
    WeaponsState weaponsState = WeaponsState.INTAKING;
    Servo[] hangReleaseServos;
    OpenCvCamera PropDetectionCamera;
    PDP_LeftCam pipelineLeft;
    PDP_DualCamera pipelineRight;
    ThreeZonePropDetectionPipeline propPipeline;
    boolean updateClaw = true;

    final FtcDashboard dashboard;
    PIDFCoefficients slidesPIDConstants = AssemblyConstants.slidesPIDConstants;
    PIDFCoefficients gantryPIDConstants = AssemblyConstants.gantryPIDConstants;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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
        horizontal = hardwareMap.get(DcMotorEx.class, "bl");

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveMotors = Arrays.asList(fl, bl, fr, br);
        odometryEncoders = Arrays.asList(verticalLeft, verticalRight, horizontal);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        //imu.resetYaw();

        // SLIDES
        slidesL = hardwareMap.get(DcMotorEx.class, "slidesL");
        slidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesR = hardwareMap.get(DcMotorEx.class, "slidesR");
        slidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides = Arrays.asList(slidesL, slidesR);
        leftSlidesEnc = hardwareMap.get(DcMotorEx.class, "fr");

        clawL = hardwareMap.get(Servo.class, "lclaw");
        clawR = hardwareMap.get(Servo.class, "rclaw");
        clawLift = hardwareMap.get(Servo.class, "clawLift");
        launcher = hardwareMap.get(Servo.class, "launcher");
        hangReleaseLeft = hardwareMap.get(Servo.class, "hangReleaseLeft");
        hangReleaseRight = hardwareMap.get(Servo.class, "hangReleaseRight");
        pancake = hardwareMap.get(Servo.class, "pancake");
        hangReleaseServos = new Servo[] {hangReleaseLeft, hangReleaseRight};

        distLeft = hardwareMap.get(AnalogInput.class, "distleft");
        distRight = hardwareMap.get(AnalogInput.class, "distright");

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        PropDetectionCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        propPipeline = new ThreeZonePropDetectionPipeline(true, loadAlliancePreset());
        PropDetectionCamera.setPipeline(propPipeline);

        if (prepAutoCamera) {
            PropDetectionCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    PropDetectionCamera.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);
                    cameraReady = true;
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

            cameraMode = CameraMode.PROP;
        }

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

        VisionPortal.Builder builder = new VisionPortal.Builder();

        //builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        //builder.setCameraResolution(new Size(640, 360));

        // Set and enable the processor.
        //builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        //visionPortal = builder.build();


        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        dashboard = FtcDashboard.getInstance();
        tagTimer = new ElapsedTime();
        depositTimer = new ElapsedTime();
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
        updateAutomation();
        if (updateClaw) {
            updateClaw();
        }
        updateSlides();

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
        } else if (localizationMode == LocalMode.FUSED) {
            localizer.updateEncoders(encoders);
            localizer.update(loopSpeed);
            currentPos = localizer.getPosEstimate();
            PreviousVelocities = CurrentVelocities;
            CurrentVelocities = localizer.getvelocity();
            if (tagTimer.time() >= 15) { // pull an AprilTag reading every 15 seconds
                tagTimer.reset();
                setCameraMode(CameraMode.APRILTAG);
                getCameraEstimate();
            }
        } else if (localizationMode == LocalMode.APRILTAG) {
            cameraMode = CameraMode.APRILTAG; // the updateCamera() method (called later) will handle everything from here
        }

    }

    public void updateAutomation() {
        switch (weaponsState) {
            case PRIMED:
                slidesTarget = 0;//slidesPrimeTarget
                clawMode = ClawMode.PRIMED;
                weaponsState = WeaponsState.IDLE;
                break;
            case EXTEND:
                slidesTarget = slidesDepositTarget;
                clawMode=ClawMode.PRIMED;
                weaponsState=WeaponsState.IDLE;
                break;
            case DEPOSIT:
                clawMode = ClawMode.RELEASE_BOTH;
                depositTimer.reset();
                waitingForDepsoit = true;
                weaponsState=WeaponsState.IDLE;
                break;
            case INTAKING:
                slidesTarget = 0;
                clawMode = ClawMode.INTAKING;
                weaponsState = WeaponsState.IDLE;
                break;
            case HOLDING:
                clawMode = ClawMode.GRAB_BOTH;
                weaponsState = WeaponsState.IDLE;
                break;
            case IDLE:
                if (depositTimer.time() > 1.2 && waitingForDepsoit) {
                    waitingForDepsoit=false;
                    weaponsState = WeaponsState.INTAKING;
                }
                break;
                //do nothing, as all parameters are now set
        }
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
    public double loadTimerPreset() {
        Reader r = new Reader();
        return Double.parseDouble(r.readFile("timer"));
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
        //gantryPos = gantyEncoder.getCurrentPosition();
        //touchVal = touch.getValue();

        if (useIMU) {
            pullIMUHeading();
        }
    }

    public double getdistLeft() {
        return ((distLeft.getVoltage()*1000)/3.2);
    }
    public double getdistRight() {
        return ((distRight.getVoltage()*1000)/3.2);
    }

    /**
     * Handles ALL camera functions. Switching between detection states is done with a variable
     */
    public void updateCamera() {
        if (cameraMode == CameraMode.PROP) {
            if (!cameraReady) {
                PropDetectionCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        PropDetectionCamera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
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
                    localizer.resetPosWithEstimate(result); // apply the new estimate to odometry and shut down the camera
                    currentPos = result;
                    getCameraEstimate = false;
                    if (localizationMode != LocalMode.APRILTAG) { // Stop streaming unless if we are constantly getting localization data
                        visionPortal.setProcessorEnabled(aprilTag, false);
                        visionPortal.stopStreaming();
                        cameraMode = CameraMode.IDLE;
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
                PropDetectionCamera.stopStreaming();
                PropDetectionCamera.closeCameraDevice();
                cameraReady = false;
            }
        }

    }


    public void setCameraMode(CameraMode mode) {cameraMode = mode;}
    public void getCameraEstimate() {getCameraEstimate = true;}
    public void setLocalizationMode(LocalMode mode) {localizationMode = mode;}

    public void setSlidesDisable(boolean disable) {slidesDisable = disable;}
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
                    slide.setPower(power);
                }

                previousSlidesError = error;
            } else {
                for (DcMotorEx slide : slides) {
                    slide.setPower(0);
                }
            }
        } else {
            if ((slidesLength <= 0 && slidesPower > 0) || (slidesLength > 0)) {
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
    /*
    public double getGantryPos() {
        return gantryPos;
    }
    public void setGantryTarget(double target) {
        gantryTarget = target;
    }
    public void setGantryPower(double power) {
        gantryPower = power;
    }
    public void updateGantry() {
        if (gantryPower == 0) {
            //DO PID STUFF
        } else {
            gantry.setPower(gantryPower);
            gantryTarget = gantryPos;
        }
    }

    public void setIntakeMode(IntakeMode mode) {
        intakeMode = mode;
    }
    public void setIntakePower(double power) {
        intakeMode = IntakeMode.MANUAL;
        intakePower = power;
    }
    public void updateIntake() {
        switch (intakeMode) {
            case LOCK:
                intake.setPower(0);
            case INTAKE:
                intake.setPower(1.0);
            case OUTTAKE:
                intake.setPower(-1.0);
            case MANUAL:
                intake.setPower(intakePower);
        }
    }
    public void setPlungerMode(PlungerMode mode) {
        plungerMode = mode;
    }
    public void setPlungerPosition(double pos) {
        plungerPos = pos;
        plungerMode = PlungerMode.MANUAL;
    }
    public void updatePlunger() {
        switch (plungerMode) {
            case LOAD:
                plunger.setPosition(0);
            case PRIME:
                plunger.setPosition(0.5);
            case DEPOSIT:
                plunger.setPosition(1);
            case MANUAL:
                plunger.setPosition(plungerPos);
        }
    }
*/

    public void setClawMode(ClawMode mode) {
        clawMode = mode;
    }
    public ClawMode getClawMode() {
        return clawMode;
    }
    public void updateClaw() {
        switch (clawMode) {
            case GRAB_L:
                clawL.setPosition(0.5);
                clawR.setPosition(0.3);
                break;
            case GRAB_R:
                clawR.setPosition(0.5);
                clawL.setPosition(0.7);
                break;
            case GRAB_BOTH:
                clawL.setPosition(0.5);
                clawR.setPosition(0.5);
                break;
            case RELEASE_L:
                clawL.setPosition(0.7);
                break;
            case RELEASE_R:
                clawR.setPosition(0.3);
                break;
            case RELEASE_BOTH:
                // release both claws, do not change lift as you may be dropping on the ground
                clawL.setPosition(0.7);
                clawR.setPosition(0.3);
                break;
            case PRIMED:
                // ensure both claws are grabbed, and lift the thingy
                clawLift.setPosition(.96);
                clawL.setPosition(0.5);
                clawR.setPosition(0.5);
                break;
            case INTAKING:
                // lift down, both claws open
                clawLift.setPosition(0.54);
                clawL.setPosition(0.7);
                clawR.setPosition(0.3);
            case IDLE:
                break;
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
            clawLift.setPosition(.96);
        } else {
            clawLift.setPosition(0.54);
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
        hangReleaseRight.setPosition(0.3);
        hangReleaseLeft.setPosition(0.7);
    }
    public void storeHang() {
        hangReleaseRight.setPosition(0);
        hangReleaseLeft.setPosition(1);
    }

    public void launchPlane() {
        launcher.setPosition(1);
    }
    public void storePlane() {
        launcher.setPosition(-1);
    }

    public void dumpPancake() {pancake.setPosition(1);}
    public void storePancake() {pancake.setPosition(-1);}

    public void storeAll() {
        storePancake();
        storePlane();
        storeHang();
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
        double frontLeftPower = (y + x - t);
        double backLeftPower = (y - x + t);
        double frontRightPower = (y - x - t);
        double backRightPower = (y + x + t);
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
        double head = pullIMUHeading(); // We don't have time to wait for the IMU to update in the next loop, so pull a reading right now
        // offset the camera estimation based off of where the camera is on the robot
        Pose2d offsetPose = new Pose2d((CAMERA_X_OFFSET*Math.cos(Math.toRadians(head)) + CAMERA_Y_OFFSET*Math.sin(Math.toRadians(head))),
                CAMERA_X_OFFSET*Math.sin(Math.toRadians(head)) + CAMERA_Y_OFFSET*Math.cos(Math.toRadians(head)),
                head
                );
        Pose2d tagEstimate = null;

        // Step through the list of detections
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                for (int i : Constants.VisionConstants.ACCEPTED_IDS) { // ensure that we are not looking at the tags on the wall... those will be inaccurate
                    if (i==detection.id) {
                        Pose2d offset = Constants.FieldConstants.TAG_FIELD_POSITIONS[i-1];
                        if (tagEstimate == null) {
                            //TODO: Update these field positions
                            tagEstimate = new Pose2d(detection.ftcPose.x + offset.getX(), offset.getY() - detection.ftcPose.y, head);
                        } else { // if multiple tags are detected, take the average of the estimates
                            tagEstimate = new Pose2d((tagEstimate.getX()+detection.ftcPose.x+offset.getX())/2, (offset.getY()-tagEstimate.getY()+detection.ftcPose.y)/2, head);
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

}
