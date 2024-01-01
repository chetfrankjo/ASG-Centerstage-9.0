package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.Reader;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.Arrays;

@TeleOp(group = "a")
public class Teleop extends LinearOpMode{

    boolean superMegaDrive = false;
    double planeTarget = 0, wallTarget = 0;
    boolean g1Launch = false, g2Launch = false, g1Hang = false, g2Hang = false, hanging =false, tl=false, tr=false, bl=false, br=false;
    int X, Y, B;
    double slidesDepositTarget = 12;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setWeaponsState(General.WeaponsState.INTAKING);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.resetSlidesEncoder();
        //driver.resetFlipperEncoder();
        driver.updateClaw(false);
        Reader r = new Reader();
        String info = r.readFile("Alliance");

        switch (info){
            case "blue_south":
                planeTarget = -90-34.2;
                wallTarget = 90;
                break;
            case "blue_north":
                planeTarget = -90-34.2;
                wallTarget = 90;
                break;
            case "red_south":
                planeTarget = 90+34.2;
                wallTarget = -90;
                break;
            case "red_north":
                planeTarget = 90+34.2;
                wallTarget = -90;
                break;
        }
        driver.setSlidesDisable(false);
        driver.setClawMode(General.ClawMode.IDLE);

        ElapsedTime hangtime = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {
            driver.update();




            driver.setSlidesPower(-gamepad2.right_stick_y); //manual slides

            /*if (gamepad2.left_stick_y != 0) {
                driver.setSlidesDisable(false);
            }
            if (gamepad2.b) {
                driver.setSlidesDisable(true);
            }



            if (gamepad1.x && X == 0) {
                if (driver.getClawLPos() >= 0.6) {
                    driver.setCLawLPos(0.5);
                } else {
                    driver.setCLawLPos(0.7);
                }
                X = 1;
            }
            if (!gamepad1.x) {
                X = 0;
            }
            if (gamepad1.b && B == 0) {
                if (driver.getClawRPos() <= 0.4) {
                    driver.setCLawRPos(0.5);
                } else {
                    driver.setCLawRPos(0.3);
                }
                B = 1;
            }
            if (!gamepad1.b) {
                B = 0;
            }
            if (gamepad1.y && Y == 0) {
                if (driver.getClawLiftPos() >= 0.8) {
                    driver.setClawLiftPos(0.5);
                } else {
                    driver.setClawLiftPos(1);
                }
                Y = 1;
            }
            if (!gamepad1.y) {
                Y = 0;
            }

             */


            if (gamepad2.left_trigger > 0.7 && !tl) {
                // generic toggle for left claw

                tl=true;
                if (driver.getSlidesLength() > 6) {
                    driver.setClawLRaw(0.6);
                } else {
                    driver.setClawLPos(false);
                }

                /*switch (driver.getClawMode()) {

                    case GRAB_L:
                        driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        //driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case GRAB_R:
                        //driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case GRAB_BOTH:
                        driver.setClawMode(General.ClawMode.RELEASE_L);
                        break;
                    case RELEASE_L:
                        //driver.setWeaponsState(General.WeaponsState.HOLDING);
                        //driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case RELEASE_R:
                        driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case RELEASE_BOTH:
                        //driver.setWeaponsState(General.WeaponsState.HOLDING);
                        //driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case PRIMED:
                        driver.setClawMode(General.ClawMode.RELEASE_L);
                        break;
                    case INTAKING:
                        //driver.setWeaponsState(General.WeaponsState.HOLDING);
                        //driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case IDLE:
                        break;
                }

                 */



            }
            if (gamepad2.left_trigger <= 0.7) {
                tl = false;
            }

            if (gamepad2.right_trigger > 0.7 && !tr) {
                tr = true;
                if (driver.getSlidesLength() > 6) {
                    driver.setCLawRRaw(0.4);
                } else {
                    driver.setClawRPos(false);
                }

                /*switch (driver.getClawMode()) {

                    case GRAB_L:
                        //driver.setWeaponsState(General.WeaponsState.INTAKING);
                        break;
                    case GRAB_R:
                        driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case GRAB_BOTH:
                        driver.setClawMode(General.ClawMode.RELEASE_R);
                        break;
                    case RELEASE_L:
                        driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case RELEASE_R:
                        break;
                    case RELEASE_BOTH:
                        //lower systems and prepare for intaking
                        //driver.setWeaponsState(General.WeaponsState.INTAKING);
                        //TODO: We do nothing here because the automation will automaticall set it to intaking
                        //      when the timer is up for the deposit. This all happens within RobotDriver (~line 318)
                        //driver.setSlidesTarget(0);
                        //driver.setClawMode(General.ClawMode.INTAKING);
                        break;
                    case PRIMED:
                        //deposit
                        driver.setClawMode(General.ClawMode.RELEASE_R);
                        //slidesDepositTarget=driver.getSlidesLength();
                        //driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case INTAKING:
                        break;
                    case IDLE:
                        break;
                }

                 */



            }
            if (gamepad2.right_trigger <= 0.7) {
                tr = false;
            }



            if (gamepad2.left_bumper && !bl) { //grabbing
                bl = true;
                driver.setClawLPos(true);
                /*switch (driver.getClawMode()) {
                    case GRAB_L:
                        //driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case GRAB_R:
                        // means that l is open, close it
                        driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case GRAB_BOTH:
                        //driver.setClawMode(General.ClawMode.RELEASE_L);
                        break;
                    case RELEASE_L:
                        driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case RELEASE_R:
                        // means that L is closed, open it
                        //driver.setClawMode(General.ClawMode.GRAB_R);
                        break;
                    case RELEASE_BOTH:
                        driver.setClawMode(General.ClawMode.GRAB_L);
                        break;
                    case PRIMED:
                        //driver.setClawMode(General.ClawMode.RELEASE_L);
                        break;
                    case INTAKING:
                        driver.setClawMode(General.ClawMode.GRAB_L);
                        break;
                    case IDLE:
                        break;
                }

                 */
            }
            if (!gamepad2.left_bumper) {
                bl = false;
            }

            if (gamepad2.right_bumper && !br) { // deposit, priming, intaking
                br = true;
                driver.setClawRPos(true);
                /*
                switch (driver.getClawMode()) {
                    case GRAB_L:
                        // means that r is open, close it
                        driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case GRAB_R:
                        //driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case GRAB_BOTH:
                        //driver.setClawMode(General.ClawMode.RELEASE_R);
                        break;
                    case RELEASE_L:
                        // means that r is closed, open it
                        //driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case RELEASE_R:
                        driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case RELEASE_BOTH:
                        driver.setClawMode(General.ClawMode.GRAB_R);
                        break;
                    case PRIMED:
                        //driver.setClawMode(General.ClawMode.RELEASE_R);
                        break;
                    case INTAKING:
                        driver.setClawMode(General.ClawMode.GRAB_R);
                        break;
                    case IDLE:
                        break;
                }

                 */
            }
            if (!gamepad2.right_bumper) {
                br = false;
            }


/*

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

 */

            /*if (gamepad2.left_trigger > 0.7) {
                driver.setClawMode(General.ClawMode.GRAB_BOTH);
            }
            if (gamepad2.right_trigger > 0.7) {
                driver.setClawMode(General.ClawMode.RELEASE_BOTH);
            }

             */

            if (gamepad2.y) {
                driver.setClawLiftPos(true);
                /*
                switch (driver.getClawMode()) {

                    case GRAB_L:
                        driver.setClawMode(General.ClawMode.PRIMED);
                        driver.update();
                        driver.setClawMode(General.ClawMode.GRAB_L);
                        break;
                    case GRAB_R:
                        driver.setClawMode(General.ClawMode.PRIMED);
                        driver.update();
                        driver.setClawMode(General.ClawMode.GRAB_R);
                        break;
                    case GRAB_BOTH:
                        driver.setClawMode(General.ClawMode.PRIMED);
                        break;
                    case RELEASE_L:
                        driver.setClawMode(General.ClawMode.PRIMED);
                        driver.update();
                        driver.setClawMode(General.ClawMode.RELEASE_L);
                        break;
                    case RELEASE_R:
                        driver.setClawMode(General.ClawMode.PRIMED);
                        driver.update();
                        driver.setClawMode(General.ClawMode.RELEASE_R);
                        break;
                    case RELEASE_BOTH:
                        driver.setClawMode(General.ClawMode.PRIMED);
                        driver.update();
                        driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case PRIMED:
                        break;
                    case INTAKING:
                        break;
                    case IDLE:
                        break;
                }

                 */
            }
            if (gamepad2.x) {
                driver.setWeaponsState(General.WeaponsState.EXTEND);
            }
            if (gamepad2.a) {
                driver.setClawLiftPos(false);

                /*switch (driver.getClawMode()) {

                    case GRAB_L:

                        break;
                    case GRAB_R:

                        break;
                    case GRAB_BOTH:
                        driver.setClawMode(General.ClawMode.INTAKING);
                        driver.update();
                        driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case RELEASE_L:

                        break;
                    case RELEASE_R:

                        break;
                    case RELEASE_BOTH:
                        driver.setClawMode(General.ClawMode.INTAKING);
                        driver.update();
                        driver.setClawMode(General.ClawMode.RELEASE_BOTH);
                        break;
                    case PRIMED:
                        driver.setClawMode(General.ClawMode.INTAKING);
                        driver.update();
                        driver.setClawMode(General.ClawMode.GRAB_BOTH);
                        break;
                    case INTAKING:

                        break;
                    case IDLE:
                        break;
                }

                 */
                //driver.setClawMode(General.ClawMode.INTAKING);
            }







            if (gamepad1.back) {
                driver.storePlane();
            }
            if (gamepad1.start) {
                driver.storeHang();
            }

            if (gamepad2.start) {
                driver.resetIMUHeading();
            }



            if (gamepad1.x) {
                if (superMegaDrive) {
                    superMegaDrive = false;
                } else {
                    superMegaDrive = true;
                }
                while (gamepad1.x) {}
            }
            if (gamepad1.left_bumper) {
                if (!g2Launch && !g1Launch) {
                    gamepad2.rumble(1, 0, 500);
                }
                g1Launch = true;
            } else {
                g1Launch = false;
            }
            if (gamepad2.dpad_left) {
                if (!g1Launch && !g2Launch) {
                    gamepad1.rumble(1, 0, 500);
                }
                g2Launch = true;
            } else {
                g2Launch = false;
            }

            if (g1Launch && g2Launch) { // launch plane if both driver confirm
                driver.launchPlane();
                gamepad1.rumble(1, 1, 500);
                gamepad2.rumble(1, 1, 500);
                g1Launch = false;
                g2Launch = false;
            }

            if (gamepad1.right_bumper) {
                if (!g2Hang && !g1Hang) {
                    gamepad2.rumble(0,1, 500);
                }
                g1Hang = true;
            } else {
                g1Hang = false;
            }

            if (gamepad2.dpad_right) {
                if (!g1Hang && !g2Hang) {
                    gamepad1.rumble(0, 1, 500);
                }
                g2Hang = true;
            } else {
                g2Hang = false;
            }

            if (g1Hang && g2Hang) {
                driver.launchHang();
                gamepad1.rumble(1, 1, 500);
                gamepad2.rumble(1, 1, 500);
                g2Hang=false;
                g1Hang=false;
            }

            if (gamepad2.back) {
                driver.resetSlidesEncoder();
            }

            if (hanging && gamepad1.left_trigger<0.7) {
                driver.drive(0, 0.2, 0, false);
                driver.setClawLiftPos(true);
                if (hangtime.time() > 0.6) {
                    hanging = false;
                }
            } else if (gamepad1.left_trigger>=0.7) {
                driver.drive(gamepad1.left_stick_x/2, -0.5, gamepad1.right_stick_x, false);
                hangtime.reset();
                hanging = true;
            } else if (gamepad1.y) {
                driver.turnInPlace(planeTarget, true, 1.0);
            } else if (gamepad1.right_trigger > 0.7){
                driver.drive((gamepad1.left_stick_x/3)+(gamepad2.left_stick_x/3), -gamepad1.left_stick_y/3, gamepad1.right_stick_x/3, superMegaDrive);
            } else {
                driver.drive(gamepad1.left_stick_x+(gamepad2.left_stick_x/3), -gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
            }

            telemetry.addData("IMU Heading", driver.getIMUHeading());
            telemetry.addData("Claw State", driver.getClawMode().toString());
            telemetry.addData("slides target", driver.getSlidesTarget());
            telemetry.addData("slides deposit target", driver.slidesDepositTarget);
            telemetry.addData("slides pos", driver.getSlidesLength());
            telemetry.addData("loop speed", driver.loopSpeed);
            telemetry.update();
        }
    }
    public static double[] removeMinMax(double[] arr) {
        Arrays.sort(arr);
        return Arrays.copyOfRange(arr, 3, arr.length - 3);
    }


    /*
    } else if (gamepad1.right_trigger > 0.7) {
                double head = driver.pullIMUHeading();
                double distvalue=0;
                double[] distarray=new double[8];
                for (int i=0; i<8; i++) {
                    //distvalue += driver.getdistRight();
                    if (info.equals("red_south") || info.equals("red_north")) {
                        distarray[i] = driver.getdistLeft();
                    } else {
                        distarray[i] = driver.getdistRight();
                    }
                }
                double[] result = removeMinMax(distarray);
                for (int i=0; i<2; i++) {
                    distvalue += result[i];
                }
                distvalue=distvalue/2;

                if (gamepad1.left_stick_x == 0) {
                    driver.drive((8.1 - distvalue) / -3.5, -gamepad1.left_stick_y, gamepad1.right_stick_x/2, false);
                } else {
                    driver.drive(gamepad1.left_stick_x/3, -gamepad1.left_stick_y, gamepad1.right_stick_x/2, false);
                }
     */
}
