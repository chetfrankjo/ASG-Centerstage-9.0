package org.firstinspires.ftc.teamcode.drive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.AutoStorage;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.ArrayList;

@Autonomous(group = "a")
public class Auto extends LinearOpMode {
    General.AutoState autoState = General.AutoState.VISION;
    General.SpikePosition position = General.SpikePosition.LEFT;
    General.AllianceLocation allianceLocation;
    General.ParkLocation parkLocation;
    General.AutoMode autoMode = General.AutoMode.STANDARD;
    ElapsedTime timer;
    double timerOffset;
    boolean didTheThing = false;
    @Override
    public void runOpMode() throws InterruptedException {

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
                driver.localizer.setEstimatePos(135, 34, 0);
                break;
            case RED_NORTH:
                driver.localizer.setEstimatePos(135, 84, 0);
                break;
            case BLUE_SOUTH:
                driver.localizer.setEstimatePos(9, 34, 0);
                break;
            case BLUE_NORTH:
                driver.localizer.setEstimatePos(9, 84, 0);
                break;
        }



        driver.setWeaponsState(General.WeaponsState.HOLDING);
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
        telemetry.update();
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            driver.setSlidesDepositTarget(10);
            driver.update();
            telemetry.addData("CurrentPos", driver.getCurrentPos().toString());
            telemetry.addData("Spike Position", position);
            telemetry.update();


            switch (autoState) {

                case VISION:
                    timer.reset();
                    driver.setClawMode(General.ClawMode.BOTH);
                    while (timer.time() < 2.5+timerOffset && opModeIsActive()) {
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
                    trajectories = AutoStorage.generateAutoPaths(parkLocation, position, allianceLocation);
                    break;
                case PURPLE_APPROACH:
                    //driver.setSlidesTarget(0);
                    boolean result = driver.runAutoPath(trajectories.get(0).path);
                    telemetry.addLine("Running Approach 1");
                    //telemetry.update();
                    if (result) {
                        // the path is ready to move on
                        timer.reset();

                        while (timer.time() < 0.08 && opModeIsActive()) {
                            driver.drive(0, 0, 0, false);
                            //driver.followCurve(trajectories.get(0).path);
                            /*switch (allianceLocation) {
                                case RED_SOUTH:
                                    driver.setClawMode(General.ClawMode.LEFT);
                                    break;
                                case RED_NORTH:
                                    driver.setClawMode(General.ClawMode.RIGHT);
                                    break;
                                case BLUE_SOUTH:
                                    driver.setClawMode(General.ClawMode.RIGHT);
                                    break;
                                case BLUE_NORTH:
                                    driver.setClawMode(General.ClawMode.LEFT);
                                    break;
                                case NONE:
                                    break;
                            }

                             */
                            driver.setIntakePower(-0.4);

                            // release pixel
                            driver.update();
                        }
                        driver.setIntakeMode(General.IntakeMode.LOCK);
                        autoState = General.AutoState.BACKUP;
                    }
                    break;
                case BACKUP:
                    driver.setWeaponsState(General.WeaponsState.HOLDING);
                    result = driver.runAutoPath(trajectories.get(1).path);
                    if (result) {
                        timer.reset();
                        autoState = General.AutoState.APPROACH_2;
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
                        if (!didTheThing) {
                            driver.setWeaponsState(General.WeaponsState.EXTEND);
                            didTheThing = true;
                        }
                        //driver.setSlidesTarget(4);
                    }
                    if (result) {
                        //driver.waitAndUpdateWithPath(1000, Constants.AutoPaths.approach_2.path); //depositing pixel
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
                    while (timer.time() < 2 && opModeIsActive() && !driver.getFSRPressed()) {
                        //TODO: Drive until FSR
                        driver.drive(0, 0.2, 0, false);
                        driver.update();
                    }
                    driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                    while (timer.time() < 2.5 && opModeIsActive()) {
                        driver.drive(0, 0, 0, false);
                        // release pixel
                        driver.update();
                    }
                    timer.reset();
                    autoState = General.AutoState.PARK_1;
                case PARK_1:
                    if (parkLocation != General.ParkLocation.CENTER) {


                        telemetry.addData("x", driver.getCurrentPos().getX());
                        telemetry.addData("y", driver.getCurrentPos().getY());
                        telemetry.addData("head", driver.getCurrentPos().getHeading());
                        telemetry.update();
                        //driver.setClawMode(General.ClawMode.BOTH);
                        result = driver.runAutoPath(trajectories.get(4).path);
                        //driver.storePancake();
                        if (result) {
                            autoState = General.AutoState.PARK2;
                        }
                    } else {
                        if (timer.time() > 1) {
                            //driver.setSlidesTarget(0);
                            //driver.setClawMode(General.ClawMode.OPEN);
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
