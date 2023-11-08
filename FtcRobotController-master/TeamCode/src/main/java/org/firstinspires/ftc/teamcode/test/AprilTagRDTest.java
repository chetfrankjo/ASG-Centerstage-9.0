package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@Disabled
@TeleOp
public class AprilTagRDTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setLocalizationMode(General.LocalMode.APRILTAG);

        waitForStart();

        while (opModeIsActive()) {
            driver.update();

            telemetry.addData("current pos", driver.getCurrentPos());
            telemetry.update();
        }
    }
}
