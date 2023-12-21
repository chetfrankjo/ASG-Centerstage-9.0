package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@Autonomous(group = "b")
public class ParkAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.update();
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            if (timer.time() < 2) {
                driver.drive(0, -0.3, 0, false);
            } else {
                driver.drive(0, 0, 0, false);
            }
            driver.update();

        }
    }
}
