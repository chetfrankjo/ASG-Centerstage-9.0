package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class AprilTagRDTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setLocalizationMode(General.LocalMode.ODOMETRY);
        boolean update = true;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                update = true;
            }
            if (gamepad1.b) {
                update = false;
            }
            if (update) {
                //driver.getCameraEstimate();
            }
            //driver.setCameraMode(General.CameraMode.APRILTAG);

            //driver.update();
            driver.setCameraMode(General.CameraMode.APRILTAG);
            driver.setTagOfInterest(5);
            driver.getCameraEstimate();
            Pose2d pos = driver.getAprilTagPosition();
            driver.update();
            if (pos != null) {
                if (pos.getX() < -2) {
                    driver.drive(0.3, 0, 0, false);
                } else if (pos.getX() > 2) {
                    driver.drive(-0.3, 0, 0, false);
                } else {
                    driver.drive(0, 0, 0, false);
                }
            }
            //telemetry.addData("tiemr", timer.time());
            assert pos != null;
            telemetry.addData("pos", pos.toString());
            //telemetry.update();

            telemetry.addData("result", driver.getAprilTagPosition());
            telemetry.addData("update?", update);
            telemetry.update();
        }
    }
}
