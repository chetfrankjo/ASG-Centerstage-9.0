package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class StoreRobot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        waitForStart();
        while (opModeIsActive()) {
            driver.update();

            if (gamepad1.a) {
                driver.launchPlane();
            }
            if (gamepad1.b) {
                driver.storePlane();
            }
            if (gamepad1.y) {
                driver.storeHang();
            }
            if (gamepad1.x) {
                driver.launchHang();
            }

        }
    }
}
