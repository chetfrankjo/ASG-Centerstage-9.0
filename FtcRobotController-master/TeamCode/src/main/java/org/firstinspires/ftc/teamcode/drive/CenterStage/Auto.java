package org.firstinspires.ftc.teamcode.drive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.ArrayList;

@Autonomous
public class Auto extends LinearOpMode {
    General.AUTO_RED_NORTH_1 autoMode = General.AUTO_RED_NORTH_1.VISION;
    General.SpikePosition position = General.SpikePosition.LEFT;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.storeAll();

        timer = new ElapsedTime();
        timer.reset();

        ArrayList<Trajectory> trajectories = Constants.AutoPaths.generateAutoPaths(General.ParkLocation.LEFT, General.SpikePosition.RIGHT, General.AllianceLocation.RED_NORTH);


        waitForStart();
        timer.reset();
        while (opModeIsActive()) {


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
                    trajectories = Constants.AutoPaths.generateAutoPaths(General.ParkLocation.RIGHT, position, General.AllianceLocation.RED_NORTH);
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
                            // release pixel
                            driver.update();
                        }

                        autoMode = General.AUTO_RED_NORTH_1.BACKUP;
                    }
                    break;
                case BACKUP:
                    result = driver.runAutoPath(trajectories.get(1).path);
                    if (result) {
                        autoMode = General.AUTO_RED_NORTH_1.APPROACH_2;
                    }
                    break;
                case APPROACH_2:
                    result = driver.runAutoPath(trajectories.get(2).path);
                    driver.setSlidesTarget(4);
                    if (result) {
                        //driver.waitAndUpdateWithPath(1000, Constants.AutoPaths.approach_2.path); //depositing pixel
                        timer.reset();
                        while (timer.time() < 2 && opModeIsActive()) {
                            driver.drive(0, 0.2, 0, false);
                            driver.update();
                        }
                        while (timer.time() < 6 && opModeIsActive()) {
                            driver.drive(0, 0, 0, false);
                            driver.dumpPancake();
                            // release pixel
                            driver.update();
                        }
                        autoMode = General.AUTO_RED_NORTH_1.PARK_1;
                    }
                    break;
                case PARK_1:
                    result = driver.runAutoPath(trajectories.get(3).path);
                    driver.storePancake();
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
