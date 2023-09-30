package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.MathFunctions.AngleWrap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.Point;
import org.firstinspires.ftc.teamcode.DataTypes.booleanInt;
import org.firstinspires.ftc.teamcode.drive.Constants.AssemblyConstants;
import org.firstinspires.ftc.teamcode.drive.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
public class RobotDriver {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront, verticalLeft, verticalRight, horizontal, turret, slides, v4bar, slides2;
    private List<DcMotorEx> driveMotors, odometryEncoders;
    private Servo verticalServo, horizontalServo, coneServo, vbarServo;
    private RevColorSensorV3 color;
    private ColorSensor color2;
    private List<Servo> odometryServos;
    private IMU imu;
    private RevTouchSensor touch;
    private AnalogInput distleft, distright;
    private VoltageSensor batterylevel;
    String verticalLeftEncoderName = "motorFrontLeft", horizontalEncoderName = "motorBackRight", verticalRightEncoderName = "motorBackLeft";
    public int[] encoders;
    public Localizer localizer;
    public ATLocalizer ATlocalizer;
    public Pose2d CurrentVelocities = new Pose2d();
    private Pose2d PreviousVelocities  = new Pose2d();
    static Pose2d currentPos = new Pose2d(0, 0, 0);

    private List<LynxModule> allHubs;
    private double slidesLength, turretHeading, vbarHeading, distLeft, distRight, touchVal, imuheading;
    double turretTarget, slidesTarget, v4barTarget, turretPower, slidesPower, v4barPower;
    static double frp=0, flp=0, brp=0, blp=0;
    public int[] colorLeft, colorRight;
    public boolean turretPID = true, slidesPID = true, v4barPID = true;
    final double slideTickToInch = AssemblyConstants.slideTickToInch;
    final double vbarTickToInch = AssemblyConstants.vbarTickToInch;
    final double zeroV4barAngle = AssemblyConstants.zeroV4barAngle;
    public int loops = 0;
    long lastLoopTime = System.nanoTime();
    public double loopSpeed = 0;
    final double v4barLength = AssemblyConstants.v4barLength;
    double turretI = 0, slidesI = 0, v4barI = 0, previousTurretError=0, previousSlidesError = 0, previousv4barError = 0;
    public static double ROBOT_RADIUS = Constants.DriveConstants.ROBOT_RADIUS;
    public Canvas overlay;
    public boolean useIMU = false;
    public boolean RSlipped, LSlipped, HSlipped;
    private int Riterations, Literations, Hiterations;
    boolean localizeWithCamera = false;

    private double tagsize = DriveConstants.tagsize;
    private double fx = DriveConstants.fx;
    private double fy = DriveConstants.fy;
    private double cx = DriveConstants.cx;
    private double cy = DriveConstants.cy;
    private boolean cameraReady = false;
    private boolean useOdometry = true;
    private boolean cameraIsStreaming = false;
    private boolean useSlipCheck = true;


    final FtcDashboard dashboard;
    PIDFCoefficients turretPIDConstants = AssemblyConstants.turretPIDConstants;
    PIDFCoefficients slidesPIDConstants = AssemblyConstants.slidesPIDConstants;
    PIDFCoefficients v4barPIDConstants = AssemblyConstants.v4barPIDConstants;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public RobotDriver(HardwareMap hardwareMap, boolean useIMU) {


        frp = 0;
        flp = 0;
        brp = 0;
        blp = 0;

        localizer = new Localizer();
        ATlocalizer = new ATLocalizer();
        encoders = new int[localizer.encoders.length];

        leftFront = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorBackRight");

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

        verticalServo = hardwareMap.get(Servo.class, "verticalServo");
        horizontalServo = hardwareMap.get(Servo.class, "horizontalServo");
        odometryServos = Arrays.asList(verticalServo, horizontalServo);
        vbarServo = hardwareMap.servo.get("vBarServo");

        if (useIMU) {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
            imu.resetYaw();
        }

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        v4bar = hardwareMap.get(DcMotorEx.class, "v4bar");
        v4bar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        v4bar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        coneServo = hardwareMap.get(Servo.class, "coneGrabber");

        color = hardwareMap.get(RevColorSensorV3.class, "color");
        color2 = hardwareMap.get(ColorSensor.class, "color2");

        touch = hardwareMap.get(RevTouchSensor.class, "touch");

        distleft = hardwareMap.get(AnalogInput.class, "distleft");
        distright = hardwareMap.get(AnalogInput.class, "distright");

        batterylevel = hardwareMap.get(VoltageSensor.class, "Control Hub");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                cameraReady = true;}
            @Override
            public void onError(int errorCode) {}
        });

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
     * @param useOdometry
     * @param useSlipCheck
     */
    public void setKeyParameters(boolean useIMU, boolean useOdometry, boolean useSlipCheck) {
        this.useIMU = useIMU;
        this.useOdometry = useOdometry;
        this.useSlipCheck = useSlipCheck;
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
        updateTurret();             // Updates turret power/PID
        updateSlides();             // Updates slides power/PID
        updatev4bar();              // Updates v4bar power/PID
        if (useSlipCheck) {
            checkForSlip();         // Analyses odometry velocities for potential slippage
        }
        // If one of the encoders has slipped and we don't already know it
        if ((HSlipped || RSlipped || LSlipped) && !localizeWithCamera) {
            // activate camera localization
            localizeWithCamera = true;
        }

        // if camera localization is activated, run the camera localizer
        if (localizeWithCamera || !useOdometry) {
            getCameraLocalization();
        }

        TelemetryPacket packet = new TelemetryPacket();
        if (!useOdometry) {
            packet.put("RUNNING CAMERA LOCALIZATION PER USER SETTINGS", "");
        }
        packet.put("localizing with camera", (localizeWithCamera || !useOdometry));
        packet.put("analyzing wheel slippage", useSlipCheck);
        packet.put("x", currentPos.getY());
        packet.put("y", currentPos.getX());
        packet.put("heading (deg)", currentPos.getHeading());
        if (useIMU) {
            packet.put("heading (imu)", imuheading);
        }
        packet.put("slides", slidesLength);
        packet.put("turret", turretHeading);
        packet.put("v4bar", vbarHeading);
        packet.put("dist right", distRight);
        packet.put("dist left", distLeft);
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

    public void useOdometry(boolean val) {
        useOdometry = val;
    }

    public void runSlipCheck(boolean val) {useSlipCheck = val;}

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
        turretHeading = turret.getCurrentPosition()*25;
        vbarHeading = (v4bar.getCurrentPosition()/vbarTickToInch)+zeroV4barAngle;
        distRight = (distright.getVoltage()*1000)/3.2;
        distLeft = (distleft.getVoltage()*1000)/3.2;
        colorLeft[0] = color.red();
        colorLeft[1] = color.green();
        colorLeft[2] = color.blue();
        colorRight[0] = color2.red();
        colorRight[1] = color2.green();
        colorRight[2] = color2.blue();
        touchVal = touch.getValue();
        if (useIMU) {
            imuheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
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
        double x = ((slidesLength*Math.cos(Math.toRadians(40.9)) + v4barLength*Math.cos(Math.toRadians(vbarHeading)))*Math.sin(Math.toRadians(turretHeading+currentPos.getHeading())))+ currentPos.getX();
        double y = ((slidesLength*Math.cos(Math.toRadians(40.9)) + v4barLength*Math.cos(Math.toRadians(vbarHeading)))*Math.cos(Math.toRadians(turretHeading+currentPos.getHeading())))+ currentPos.getY();
        double z = slidesLength*Math.sin(Math.toRadians(40.9)) + v4barLength*Math.sin(vbarHeading);
        return new double[] {x, y, z};
    }

    public void setAssemblyCoordinates(double x, double y, double z, double v4barAngle) {       //Sets subsystems to best possible 3D location
        double deltaX = x - currentPos.getX();
        double deltaY = y - currentPos.getY();
        double deltaTheta = Math.toDegrees(Math.atan(deltaY/deltaX));
        turretTarget = deltaTheta - currentPos.getHeading();
        slidesTarget = (deltaX + (v4barLength*Math.cos(Math.toRadians(v4barAngle))))/Math.sin(Math.toRadians(40.9));
        v4barTarget=v4barAngle;
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

    public double verticalServoPos() {return verticalServo.getPosition();}
    public double horizontalServoPos() {return horizontalServo.getPosition();}
    public void liftOdometry() {
        setHorizontalServo(0.4);
        setVerticalServo(0.7);
    }
    public void lowerOdometry() {
        setHorizontalServo(0.15);
        setVerticalServo(1);
    }

    public void setVServoPos(double pos) {
        vbarServo.setPosition(pos);
    }


    public void setVerticalServo(double pos) {verticalServo.setPosition(pos);}
    public void setHorizontalServo(double pos) {horizontalServo.setPosition(pos);}

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

    // Analyses the acceleration of each odo pod for slippage
    public void checkForSlip() {
        booleanInt result = slipCheck(CurrentVelocities.getY(), PreviousVelocities.getY(), Literations);
        LSlipped = result.getBool();
        Literations = result.getNum();
        result = slipCheck(CurrentVelocities.getX(), PreviousVelocities.getX(), Riterations);
        RSlipped = result.getBool();
        Riterations = result.getNum();
        result = slipCheck(CurrentVelocities.getHeading(), PreviousVelocities.getHeading(), Hiterations);
        HSlipped = result.getBool();
        Hiterations = result.getNum();
    }

    public boolean getSlippedL() {
        return LSlipped;
    }
    public boolean getSlippedR() {
        return RSlipped;
    }
    public boolean getSlippedH() {
        return HSlipped;
    }


    private booleanInt slipCheck(double currentVelocity, double previousVelocity, int iterations) {
        boolean slipped = false;
        // if the velocity suddenly decreases
        if (Math.abs(previousVelocity) < Math.abs(currentVelocity) - 10) {
            //If it hasn't bounced back in a while, you just decelerating and no slipping occurred
            if (iterations >= 15) {
                iterations = 0;
            }
            // accumulate the amount of loops you've been waiting
            iterations++;
        } else {
            // if the velocity bounces back, that means you slipped
            if (iterations >= 15) {
                slipped = true;
                // reset iterations so that the loop can run again and again and...
                iterations = 0;
            }
        }

        return new booleanInt(slipped, iterations);
    }


    public void getCameraLocalization() {
        // if the camera initialized with no errors
        if (cameraReady) {
            // start streaming the camera
            if (!cameraIsStreaming) {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                cameraIsStreaming = true;
            }
            // run the pipeline and detemine positions
            ATlocalizer.update(aprilTagDetectionPipeline);
            // get the localizer's estimate
            Pose2d ATEstimate = ATlocalizer.getEstimatePosition();
            // if the localizer found a tag and relative position,

            if (ATEstimate != null) {
                // feed the new estimate to the odometry localizer
                localizer.resetPosWithEstimate(ATEstimate);
                currentPos = ATEstimate;
                // the issue has been taken care of, reset trigger values
                localizeWithCamera = false;
                HSlipped=false;
                RSlipped=false;
                LSlipped=false;
                // if we plan on using the odometry to determine position from now on
                if (useOdometry) {
                    // stop streaming the camera to conserve loop times
                    camera.stopStreaming();
                    cameraIsStreaming = false;
                }
            } else {
                // if the localizer did not find a location, continue to run it until it does- TODO: Add a timeout for this
                localizeWithCamera = true;
            }
        }

    }




}
