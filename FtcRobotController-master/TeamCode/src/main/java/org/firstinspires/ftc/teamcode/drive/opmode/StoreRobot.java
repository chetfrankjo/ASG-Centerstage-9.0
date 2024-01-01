package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp(group = "b")
public class StoreRobot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setWeaponsState(General.WeaponsState.IDLE);
        waitForStart();
        while (opModeIsActive()) {
            driver.update();
            telemetry.addLine("DPAD_UP - RESET ENCODERS");
            telemetry.addLine("Left Bumper/Trigger - Load/Launch Plane");
            telemetry.addLine("Right Bumper/Trigger - Load/Launch Hand");
            telemetry.addLine("A/B - Load/Launch Pancake");
            telemetry.update();

            if (gamepad1.left_trigger > 0.7) {
                driver.launchPlane();
            }
            if (gamepad1.left_bumper) {
                driver.storePlane();
            }
            if (gamepad1.right_bumper) {
                driver.storeHang();
            }
            if (gamepad1.right_trigger > 0.7) {
                driver.launchHang();
            }
            if (gamepad1.dpad_up) {
                driver.resetFlipperEncoder();
                driver.resetSlidesEncoder();
            }


        }
    }
}
