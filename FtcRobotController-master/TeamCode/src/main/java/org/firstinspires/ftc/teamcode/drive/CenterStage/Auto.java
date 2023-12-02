package org.firstinspires.ftc.teamcode.drive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
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
    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.storeAll();
        driver.resetIMUHeading();
        driver.setWeaponsState(General.WeaponsState.HOLDING);


        timer = new ElapsedTime();
        timer.reset();

        ArrayList<Trajectory> trajectories = Constants.AutoPaths.generateAutoPaths(General.ParkLocation.RIGHT, General.SpikePosition.CENTER, General.AllianceLocation.RED_NORTH);
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
        telemetry.update();
        driver.setClawLiftPos(0.55);
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            driver.setSlidesDepositTarget(29);
            driver.update();
            telemetry.addData("CurrentPos", driver.getCurrentPos().toString());
            telemetry.addData("Spike Position", position);
            telemetry.update();


            switch (autoMode) {

                case VISION:
                    timer.reset();

                    while (timer.time() < 3 && opModeIsActive()) {
                        driver.setCameraMode(General.CameraMode.PROP);
                        driver.getCameraEstimate();
                        driver.update();
                        position = driver.propLocation;
                        telemetry.addData("Estimate", position.toString());
                        telemetry.update();
                    }
                    driver.setCameraMode(General.CameraMode.IDLE);
                    autoMode = General.AUTO_RED_NORTH_1.APPROACH_1;
                    trajectories = Constants.AutoPaths.generateAutoPaths(parkLocation, position, allianceLocation);
                    break;
                case APPROACH_1:
                    boolean result = driver.runAutoPath(trajectories.get(0).path);
                    telemetry.addLine("Running Approach 1");
                    //telemetry.update();
                    if (result) {
                        // the path is ready to move on
                        timer.reset();

                        while (timer.time() < 1 && opModeIsActive()) {
                            driver.followCurve(trajectories.get(0).path);
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
                        if (timer.time() > 3) {
                            //driver.setSlidesTarget(4);
                            driver.setWeaponsState(General.WeaponsState.EXTEND);
                        }
                    } else {
                        driver.setWeaponsState(General.WeaponsState.EXTEND);
                        //driver.setSlidesTarget(4);
                    }
                    if (result) {
                        //driver.waitAndUpdateWithPath(1000, Constants.AutoPaths.approach_2.path); //depositing pixel
                        timer.reset();
                        while (timer.time() < 0.5 && opModeIsActive()) {
                            driver.drive(0, 0.2, 0, false);
                            driver.update();
                        }
                        while (timer.time() < 2 && opModeIsActive()) {
                            driver.drive(0, 0, 0, false);
                            driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                            //driver.dumpPancake();
                            // release pixel
                            driver.update();
                        }
                        autoMode = General.AUTO_RED_NORTH_1.PARK_1;
                    }
                    break;
                case PARK_1:
                    telemetry.addData("cur pos x", driver.getCurrentPos().getX());
                    telemetry.update();
                    result = driver.runAutoPath(trajectories.get(3).path);
                    //driver.storePancake();
                    if (result) {
                        autoMode = General.AUTO_RED_NORTH_1.PARK2;
                    }
                    break;
                case PARK2:
                    driver.setSlidesTarget(0);
                    driver.followCurve(trajectories.get(4).path);
                    break;
            }



        }
    }
}
