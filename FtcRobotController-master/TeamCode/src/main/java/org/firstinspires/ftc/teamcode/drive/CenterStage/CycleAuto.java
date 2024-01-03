package org.firstinspires.ftc.teamcode.drive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.CurvePoint;
import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.AutoStorage;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.ArrayList;

@Autonomous(group = "a")
public class CycleAuto extends LinearOpMode {
    General.AutoState autoState = General.AutoState.VISION;
    General.SpikePosition position = General.SpikePosition.LEFT;
    General.AllianceLocation allianceLocation;
    General.ParkLocation parkLocation;
    General.AutoMode autoMode = General.AutoMode.STANDARD;
    ElapsedTime timer;
    double timerOffset;
    ArrayList<Trajectory> paths = new ArrayList<>();





    @Override
    public void runOpMode() throws InterruptedException {
        paths.add(new Trajectory(135, 34, 0.25, 12, 0.2).addPoint(107, 34, 90).build());
        paths.add(new Trajectory(107, 34, 0.25, 16).addPoint(107, 14, 180).build());
        paths.add(new Trajectory(107, 13, 0.5, 10).addPoint(107, 18, 0).addPoint(77, 18, 90).addPoint(77, 96, 0).addPoint(101, 96, -90).addPoint(101, 108, 0).build());
        paths.add(new Trajectory(101, 108, 0.4, 10).addPoint(96, 108, 90).build());
        paths.add(new Trajectory(96, 115, 0.5, 10).addPoint(96, 100, 180).addPoint(77, 100, 90).addPoint(77, 50, 180).addPoint(85, 14, 180).build()); // cycle intake/approach
        /*paths.add(new Trajectory(85, 20, 0.6, 10).addPoint(73, 40, 0).addPoint(73, 95, 0).addPoint(101, 95, -90).addPoint(101, 108, 0).build()); // cycle backdrop apprach
        paths.add(new Trajectory(101, 108, 0.4, 10).addPoint(96, 108, 90).build()); //cycle backdrop go to perfect position
        paths.add(new Trajectory(95, 115, 0.4, 8, 0).addPoint(95, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
        paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 115, 0).build());

         */

        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.storeAll();
        driver.resetFlipperEncoder();
        driver.resetSlidesEncoder();
        driver.resetIMUHeading();
        driver.setWeaponsState(General.WeaponsState.INTAKING);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.resetSlidesEncoder();
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
                driver.localizer.setEstimatePos(135, 34, -90);
                break;
            case RED_NORTH:
                driver.localizer.setEstimatePos(135, 84, -90);
                break;
            case BLUE_SOUTH:
                driver.localizer.setEstimatePos(9, 34, 90);
                break;
            case BLUE_NORTH:
                driver.localizer.setEstimatePos(9, 84, 90);
                break;
        }

         */


        driver.localizer.setEstimatePos(135, 34, 0);
        driver.setWeaponsState(General.WeaponsState.HOLDING);
        telemetry.update();
        driver.setClawLiftPos(false);
        /*while (opModeInInit()) {
            driver.setCameraMode(General.CameraMode.PROP);
            driver.getCameraEstimate();
            driver.update();
            position = driver.propLocation;
        }

         */
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            driver.setSlidesDepositTarget(12);
            driver.update();
            telemetry.addData("CurrentPos", driver.getCurrentPos().toString());
            telemetry.addData("Spike Position", position);
            telemetry.update();


            switch (autoState) {

                case VISION: // Read camera, no movement
                    timer.reset();
                    driver.setClawMode(General.ClawMode.BOTH);
                    while (timer.time() < 0.5+timerOffset && opModeIsActive()) {
                        driver.setCameraMode(General.CameraMode.PROP);
                        driver.getCameraEstimate();
                        driver.update();
                        position = driver.propLocation;
                        telemetry.addData("Estimate", position.toString());
                        telemetry.update();
                    }
                    driver.setCameraMode(General.CameraMode.IDLE);
                    autoState = General.AutoState.PURPLE_APPROACH;
                    //trajectories = Constants.AutoPaths.generateAutoPaths(parkLocation, position, allianceLocation);
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
                            //driver.followCurve(trajectories.get(0).path);
                            driver.setClawMode(General.ClawMode.LEFT);


                            // release pixel
                            driver.update();
                        }
                        timer.reset();
                        autoState = General.AutoState.BACKUP;
                    }
                    break;
                case SPIKE:
                    break;
                case BACKUP: // drive to the stack, pick one pixel up
                    driver.setWeaponsState(General.WeaponsState.INTAKING);
                    result = driver.runAutoPath(trajectories.get(1).path);
                    telemetry.addData("current pos", driver.getCurrentPos().toString());
                    telemetry.update();
                    if (result) {
                        timer.reset();
                        driver.setWeaponsState(General.WeaponsState.INTAKING);
                        while ((timer.time() < 2 && !driver.getRightHasPixel()) && opModeIsActive()) {
                            driver.followCurve(trajectories.get(1).path); //TODO: Drive forwards slowly?
                            //driver.drive(0, -0.25, 0, false);
                            driver.update();
                        }
                        driver.setWeaponsState(General.WeaponsState.HOLDING);
                        driver.update();
                        //while (opModeIsActive()) {driver.drive(0, 0, 0, false);}
                        autoState = General.AutoState.APPROACH_2;
                    }
                    break;
                case APPROACH_2:
                    result = driver.runAutoPath(trajectories.get(2).path);
                    if (driver.getCurrentPos().getY() > 85 && driver.getClawLiftPos()<0.64) {
                        //driver.setSlidesTarget(4);
                        driver.setWeaponsState(General.WeaponsState.EXTEND);
                    } else if (driver.getCurrentPos().getY() < 30 && driver.getCurrentPos().getY() > 15) {
                        driver.setIntakeMode(General.IntakeMode.OUTTAKE);
                    } else {
                        driver.setIntakeMode(General.IntakeMode.LOCK);
                    }
                    if (result) {
                        //driver.drive(0, 0, 0, false);
                        timer.reset();
                        while (opModeIsActive() && (!driver.getFSRPressed() || timer.time()<2)) {
                            driver.drive(0, 0, 0, false);
                            driver.update();
                        }
                        autoState = General.AutoState.APPROACH_3;
                    }
                    break;
                case APPROACH_3:
                    timer.reset();
                    while (timer.time() < 0.8 && opModeIsActive()) {
                        driver.followCurve(trajectories.get(3).path);
                        //driver.drive(0, 0.2, 0, false);
                        driver.update();
                    }
                    while (timer.time() < 2 && opModeIsActive()) {
                        //driver.drive(0, 0.2, 0, false);
                        driver.update();
                    }
                    while (timer.time() < 3.5 && opModeIsActive()) {
                        driver.drive(0, 0, 0, false);
                        driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                        // release pixel
                        driver.update();
                    }
                    timer.reset();

                    autoState = General.AutoState.CYCLE_INTAKE;
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
                        while (opModeIsActive()) {driver.drive(0, 0, 0, false);}
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
                        driver.setClawMode(General.ClawMode.BOTH);
                        result = driver.runAutoPath(trajectories.get(4).path);
                        //driver.storePancake();
                        if (result) {
                            autoState = General.AutoState.PARK2;
                        }
                    } else {
                        if (timer.time() > 1) {
                            driver.setSlidesTarget(0);
                            //driver.setClawMode(General.ClawMode.PRIMED);
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
                    driver.setSlidesTarget(0);
                    driver.followCurve(trajectories.get(5).path);
                    break;
            }



        }
    }
}
