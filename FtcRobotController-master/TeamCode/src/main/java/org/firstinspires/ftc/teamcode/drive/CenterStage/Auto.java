package org.firstinspires.ftc.teamcode.drive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.AutoStorage;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.ArrayList;

@Autonomous(group = "a")
public class Auto extends LinearOpMode {
    General.AUTO_RED_NORTH_1 autoMode = General.AUTO_RED_NORTH_1.VISION;
    General.SpikePosition position = General.SpikePosition.LEFT;
    General.AllianceLocation allianceLocation;
    General.ParkLocation parkLocation;
    ElapsedTime timer;
    double timerOffset;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.storeAll();
        driver.resetIMUHeading();
        driver.setWeaponsState(General.WeaponsState.INTAKING);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.resetSlidesEncoder();
        timerOffset = driver.loadTimerPreset();

        timer = new ElapsedTime();
        timer.reset();

        ArrayList<Trajectory> trajectories = AutoStorage.generateAutoPaths(General.ParkLocation.RIGHT, General.SpikePosition.CENTER, General.AllianceLocation.RED_NORTH);
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



        telemetry.update();
        driver.setClawLiftPos(false);
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            driver.setSlidesDepositTarget(32);
            driver.update();
            telemetry.addData("CurrentPos", driver.getCurrentPos().toString());
            telemetry.addData("Spike Position", position);
            telemetry.update();


            switch (autoMode) {

                case VISION:
                    timer.reset();
                    driver.setClawMode(General.ClawMode.GRAB_BOTH);
                    while (timer.time() < 3+timerOffset && opModeIsActive()) {
                        if (timer.time()>0.5) {
                            driver.setSlidesTarget(12);
                        }
                        driver.setCameraMode(General.CameraMode.PROP);
                        driver.getCameraEstimate();
                        driver.update();
                        position = driver.propLocation;
                        telemetry.addData("Estimate", position.toString());
                        telemetry.update();
                    }
                    driver.setCameraMode(General.CameraMode.IDLE);
                    autoMode = General.AUTO_RED_NORTH_1.APPROACH_1;
                    //trajectories = Constants.AutoPaths.generateAutoPaths(parkLocation, position, allianceLocation);
                    trajectories = AutoStorage.generateAutoPaths(parkLocation, position, allianceLocation);
                    break;
                case APPROACH_1:
                    driver.setSlidesTarget(0);
                    boolean result = driver.runAutoPath(trajectories.get(0).path);
                    telemetry.addLine("Running Approach 1");
                    //telemetry.update();
                    if (result) {
                        // the path is ready to move on
                        timer.reset();

                        while (timer.time() < 1 && opModeIsActive()) {
                            driver.drive(0, 0, 0, false);
                            //driver.followCurve(trajectories.get(0).path);
                            switch (allianceLocation) {
                                case RED_SOUTH:
                                    driver.setClawMode(General.ClawMode.RELEASE_R);
                                    break;
                                case RED_NORTH:
                                    driver.setClawMode(General.ClawMode.RELEASE_L);
                                    break;
                                case BLUE_SOUTH:
                                    driver.setClawMode(General.ClawMode.RELEASE_L);
                                    break;
                                case BLUE_NORTH:
                                    driver.setClawMode(General.ClawMode.RELEASE_R);
                                    break;
                                case NONE:
                                    break;
                            }

                            // release pixel
                            driver.update();
                        }

                        autoMode = General.AUTO_RED_NORTH_1.BACKUP;
                    }
                    break;
                case BACKUP:
                    driver.setWeaponsState(General.WeaponsState.PRIMED);
                    result = driver.runAutoPath(trajectories.get(1).path);
                    if (result) {
                        timer.reset();
                        autoMode = General.AUTO_RED_NORTH_1.APPROACH_2;
                    }
                    break;
                case APPROACH_2:
                    result = driver.runAutoPath(trajectories.get(2).path);
                    if (allianceLocation == General.AllianceLocation.RED_SOUTH || allianceLocation == General.AllianceLocation.BLUE_SOUTH) {
                        if (driver.getCurrentPos().getY() > 85) {
                            //driver.setSlidesTarget(4);
                            driver.setWeaponsState(General.WeaponsState.EXTEND);
                        }
                    } else {
                        driver.setWeaponsState(General.WeaponsState.EXTEND);
                        //driver.setSlidesTarget(4);
                    }
                    if (result) {
                        //driver.waitAndUpdateWithPath(1000, Constants.AutoPaths.approach_2.path); //depositing pixel
                        autoMode = General.AUTO_RED_NORTH_1.APPROACH_3;
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
                    autoMode = General.AUTO_RED_NORTH_1.PARK_1;
                case PARK_1:
                    if (parkLocation != General.ParkLocation.CENTER) {


                        telemetry.addData("x", driver.getCurrentPos().getX());
                        telemetry.addData("y", driver.getCurrentPos().getY());
                        telemetry.addData("head", driver.getCurrentPos().getHeading());
                        telemetry.update();
                        driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        result = driver.runAutoPath(trajectories.get(4).path);
                        //driver.storePancake();
                        if (result) {
                            autoMode = General.AUTO_RED_NORTH_1.PARK2;
                        }
                    } else {
                        if (timer.time() > 1) {
                            driver.setSlidesTarget(0);
                            driver.setClawMode(General.ClawMode.PRIMED);
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
