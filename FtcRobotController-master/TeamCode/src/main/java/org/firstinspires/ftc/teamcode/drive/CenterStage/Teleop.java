package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.Reader;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.Arrays;

@TeleOp(group = "a")
public class Teleop extends LinearOpMode{

    boolean superMegaDrive = false;
    double planeTarget = 0, wallTarget = 0;
    boolean g1Launch = false, g2Launch = false, g1Hang = false, g2Hang = false;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setWeaponsState(General.WeaponsState.IDLE);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

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

        waitForStart();
        while (opModeIsActive()) {
            driver.update();



            if (!gamepad2.a) {
                driver.setSlidesPower(gamepad2.left_stick_y); //manual slides
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

            if (gamepad1.a) {
                driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (gamepad1.b) {
                driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
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

            if (gamepad2.x) {
                driver.storePancake();
            }
            if (gamepad2.y) {
                driver.dumpPancake();
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
            if (gamepad2.left_bumper) {
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

            if (gamepad2.right_bumper) {
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
                driver.resetSlidesEncoder();
            }


            if (gamepad1.right_trigger > 0.7) {
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
            } else if (gamepad1.y) {
                driver.turnInPlace(planeTarget, true, 1.0);
            } else if (!gamepad2.a) {
                driver.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
            } else {
                driver.drive(gamepad2.left_stick_x/4, -gamepad2.left_stick_y/4, gamepad1.right_stick_x, superMegaDrive);
            }


            telemetry.addData("dist left", driver.getdistLeft());
            telemetry.addData("dist right", driver.getdistRight());
            telemetry.addData("IMU Heading", driver.getIMUHeading());
            telemetry.addData("dist", driver.getdistRight());
            telemetry.addData("loop speed", driver.loopSpeed);
            telemetry.update();
        }
    }
    public static double[] removeMinMax(double[] arr) {
        Arrays.sort(arr);
        return Arrays.copyOfRange(arr, 3, arr.length - 3);
    }
}
