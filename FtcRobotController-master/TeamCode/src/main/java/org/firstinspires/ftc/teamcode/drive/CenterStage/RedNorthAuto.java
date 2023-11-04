package org.firstinspires.ftc.teamcode.drive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@Autonomous
public class RedNorthAuto extends LinearOpMode {
    General.AUTO_RED_NORTH_1 autoMode = General.AUTO_RED_NORTH_1.APPROACH_1;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.localizer.setEstimatePos(132, 84, -90);
        timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {

            driver.update();

            switch (autoMode) {
                case APPROACH_1:
                     boolean result = driver.runAutoPath(Constants.AutoPaths.approach_1.path);

                     if (result) {
                         // the path is ready to move on
                         timer.reset();
                         while (timer.time() < 1) {
                             driver.followCurve(Constants.AutoPaths.approach_1.path);
                             // release pixel
                             driver.update();
                         }
                         autoMode = General.AUTO_RED_NORTH_1.BACKUP;
                     }
                case BACKUP:
                    result = driver.runAutoPath(Constants.AutoPaths.backup.path);
                    if (result) {
                        autoMode = General.AUTO_RED_NORTH_1.APPROACH_2;
                    }
                case APPROACH_2:
                    result = driver.runAutoPath(Constants.AutoPaths.approach_2.path);
                    if (result) {
                        //driver.waitAndUpdateWithPath(1000, Constants.AutoPaths.approach_2.path); //depositing pixel
                        timer.reset();
                        while (timer.time() < 1) {
                            driver.drive(0, 0.2, 0, false);
                            // release pixel
                            driver.update();
                        }
                        autoMode = General.AUTO_RED_NORTH_1.PARK;
                    }
                case PARK:
                    driver.followCurve(Constants.AutoPaths.park.path);
            }


        }
    }
}
