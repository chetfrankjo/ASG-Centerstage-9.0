package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.Reader;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp(group = "a")
public class Teleop extends LinearOpMode{

    boolean superMegaDrive = false;
    double planeTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setWeaponsState(General.WeaponsState.IDLE);

        Reader r = new Reader();
        String info = r.readFile("Alliance");

        switch (info){
            case "blue_south":
                planeTarget = -90-34.2;
                break;
            case "blue_north":
                planeTarget = -90-34.2;
                break;
            case "red_south":
                planeTarget = 90+34.2;
                break;
            case "red_north":
                planeTarget = 90+34.2;
                break;
        }

        waitForStart();
        while (opModeIsActive()) {
            driver.update();

            telemetry.addData("imu heading", driver.getIMUHeading());
            telemetry.update();


            if (!gamepad2.a) {
                driver.setSlidesPower(gamepad2.left_stick_y); //manual slides
            }


            if (gamepad2.right_trigger > 0.5) {
                driver.setWeaponsState(General.WeaponsState.PRIMED);
            }//priming
            if (gamepad2.left_trigger > 0.5) {
                driver.setWeaponsState(General.WeaponsState.HOLDING);
            }//intake
            if (gamepad2.left_bumper) {
                driver.setWeaponsState(General.WeaponsState.DEPOSIT);
            }//Deposit
            if (gamepad2.x) {driver.setSlidesTarget(0); driver.setClawMode(General.ClawMode.RELEASE);}
            if (gamepad1.right_bumper) {
                switch (driver.getClawMode()) {
                    case RELEASE:
                        driver.setClawMode(General.ClawMode.GRAB);
                    case GRAB:
                        driver.setClawMode(General.ClawMode.RELEASE);
                }
            }

            if (gamepad1.left_stick_button) {
                driver.setSlidesTarget(12);
            }

            if (gamepad1.x) {
                if (superMegaDrive) {
                    superMegaDrive = false;
                } else {
                    superMegaDrive = true;
                }
            }
            if (gamepad1.left_trigger > 0.7) {
                driver.launchPlane();
            }//GO PLANE GO
            if (gamepad1.right_trigger > 0.7) {
                driver.launchHang();
            }//HANG

            if (gamepad2.share) {
                driver.resetSlidesEncoder();
            }
            if (gamepad1.y) {
                driver.turnInPlace(planeTarget, true, 1.0);
            } else if (!gamepad2.a) {
                driver.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
            } else {
                driver.drive(gamepad2.left_stick_x/4, -gamepad2.left_stick_y/4, gamepad1.right_stick_x, superMegaDrive);
            }



        }
    }
}
