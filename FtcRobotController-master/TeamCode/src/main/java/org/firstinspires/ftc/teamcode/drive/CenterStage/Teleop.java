package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class Teleop extends LinearOpMode{

    boolean superMegaDrive = true;


    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, true);

        waitForStart();
        while (opModeIsActive()) {



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
            if (gamepad2.b) {driver.setSlidesTarget(0); driver.setClawMode(General.ClawMode.RELEASE);}
            if (gamepad1.right_bumper) {
                switch (driver.getClawMode()) {
                    case RELEASE:
                        driver.setClawMode(General.ClawMode.GRAB);
                    case GRAB:
                        driver.setClawMode(General.ClawMode.RELEASE);
                }
            }

            if (gamepad1.b) {
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

            if (gamepad2.y) {
                driver.resetSlidesEncoder();
            }

            if (!gamepad2.a) {
                driver.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
            } else {
                driver.drive(gamepad2.left_stick_x/4, -gamepad2.left_stick_y/4, gamepad1.right_stick_x, superMegaDrive);
            }


        }
    }
}