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
    boolean g1Launch = false, g2Launch = false, g1Hang = false, g2Hang = false, hanging =false, tl=false, tr=false, bl=false, br=false;
    boolean redAlliance = false;
    boolean allowAutoIntake = true, allowAutoHolding=false, allowAutoDeposit=false, intakeFront=false, g1lt=false, outtake = false, autoIntaking = false;
    ElapsedTime outtakeTimer;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setWeaponsState(General.WeaponsState.HOLDING);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.resetSlidesEncoder();
        driver.resetFlipperEncoder();
        //driver.updateClaw(false);

        switch (driver.loadAlliancePreset()) {
            case RED_SOUTH:
                redAlliance = true;
                switch (driver.loadParkPreset()) {
                    case LEFT:
                        driver.localizer.setEstimatePos(76, 115, 0);
                        break;
                    case RIGHT:
                        driver.localizer.setEstimatePos(125, 115, 0);
                        break;
                    case CENTER:
                        driver.localizer.setEstimatePos(100, 115, 0);
                        break;
                    case NONE:
                        break;
                }
                break;
            case RED_NORTH:
                redAlliance = true;
                switch (driver.loadParkPreset()) {
                    case LEFT:
                        driver.localizer.setEstimatePos(76, 115, 0);
                        break;
                    case RIGHT:
                        driver.localizer.setEstimatePos(125, 115, 0);
                        break;
                    case CENTER:
                        driver.localizer.setEstimatePos(100, 115, 0);
                        break;
                    case NONE:
                        break;
                }
                break;
            case BLUE_SOUTH:
                redAlliance = false;
                switch (driver.loadParkPreset()) {
                    case LEFT:
                        driver.localizer.setEstimatePos(22, 115, 0);
                        break;
                    case RIGHT:
                        driver.localizer.setEstimatePos(45, 115, 0);
                        break;
                    case CENTER:
                        driver.localizer.setEstimatePos(68, 115, 0);
                        break;
                    case NONE:
                        break;
                }
                break;
            case BLUE_NORTH:
                redAlliance = false;
                switch (driver.loadParkPreset()) {
                    case LEFT:
                        driver.localizer.setEstimatePos(22, 115, 0);
                        break;
                    case RIGHT:
                        driver.localizer.setEstimatePos(45, 115, 0);
                        break;
                    case CENTER:
                        driver.localizer.setEstimatePos(68, 115, 0);
                        break;
                    case NONE:
                        break;
                }
                break;
            case NONE:
                break;
        }

        driver.setSlidesDisable(false);
        //driver.setClawMode(General.ClawMode.IDLE);

        ElapsedTime hangtime = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            driver.update();

            driver.setSlidesPower(gamepad2.right_stick_y); // manual slides
            driver.setFlipperPower(gamepad2.left_stick_y/4); // manual flipper

            if (gamepad2.left_trigger > 0.7 && !tl) { // open left claw
                tl=true;
                switch (driver.getClawMode()) {
                    case OPEN:
                        break;
                    case LEFT:
                        driver.setClawMode(General.ClawMode.OPEN);
                        if (driver.getClawLiftPos()>=0.9) { // if you are in a deposit position, do a full deposit
                            driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                            driver.setSlidesDepositTarget(driver.getSlidesLength());
                        }
                        break;
                    case RIGHT:
                        break;
                    case BOTH:
                        driver.setClawMode(General.ClawMode.RIGHT);
                        if (!driver.getRightHasPixel() && driver.getClawLiftPos()>=0.9&&allowAutoIntake) { // if right doesnt have a pixel, then you are doing a full deposit
                            driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                            driver.setSlidesDepositTarget(driver.getSlidesLength());
                        }
                        break;
                }
            }
            if (gamepad2.left_trigger <= 0.7) {
                tl = false;
            }

            if (gamepad2.right_trigger > 0.7 && !tr) { // open right claw
                tr = true;
                switch (driver.getClawMode()) {
                    case OPEN:
                        break;
                    case LEFT:
                        break;
                    case RIGHT:
                        driver.setClawMode(General.ClawMode.OPEN);
                        if (driver.getClawLiftPos()>=0.9) { // if you are in a deposit position, do a full deposit
                            driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                            driver.setSlidesDepositTarget(driver.getSlidesLength());
                        }
                        break;
                    case BOTH:
                        driver.setClawMode(General.ClawMode.LEFT);
                        if (!driver.getLeftHasPixel() && driver.getClawLiftPos()>=0.9&&allowAutoIntake) { // if left doesnt have a pixel, then you are doing a full deposit
                            driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                            driver.setSlidesDepositTarget(driver.getSlidesLength());
                        }
                        break;
                }
            }
            if (gamepad2.right_trigger <= 0.7) {
                tr = false;
            }

            if (gamepad2.left_bumper && !bl) { //grab left claw
                bl = true;

                switch (driver.getClawMode()) {
                    case OPEN:
                        driver.setClawMode(General.ClawMode.LEFT);
                        break;
                    case LEFT:
                        break;
                    case RIGHT:
                        driver.setClawMode(General.ClawMode.BOTH);
                        if (driver.getIntakeMode() == General.IntakeMode.INTAKE) { // if you are intaking and both claws are grabbed, stop intaking
                            driver.setWeaponsState(General.WeaponsState.HOLDING);
                            outtakeTimer.reset();
                            outtake = true;
                        }
                        break;
                    case BOTH:
                        break;

                }
            }
            if (!gamepad2.left_bumper) {
                bl = false;
            }

            if (gamepad2.right_bumper && !br) { // grab right claw
                br = true;
                switch (driver.getClawMode()) {
                    case OPEN:
                        driver.setClawMode(General.ClawMode.RIGHT);
                        break;
                    case LEFT:
                        driver.setClawMode(General.ClawMode.BOTH);
                        if (driver.getIntakeMode() == General.IntakeMode.INTAKE) { // if you are intaking and both claws are grabbed, stop intaking
                            driver.setWeaponsState(General.WeaponsState.HOLDING);
                            outtakeTimer.reset();
                            outtake = true;
                        }
                        break;
                    case RIGHT:
                        break;
                    case BOTH:
                        break;
                }
            }
            if (!gamepad2.right_bumper) {
                br = false;
            }

            if (gamepad2.y) { // extend+flip for depositing
                driver.setWeaponsState(General.WeaponsState.EXTEND);
            }

            if (gamepad2.dpad_down) { // grab both claws, stop intake, start temporary outtake
                driver.setWeaponsState(General.WeaponsState.HOLDING);
                outtakeTimer.reset();
                outtake = true;
            }

            if (gamepad2.x) { // manual outtake
                driver.setIntakeMode(General.IntakeMode.OUTTAKE);
            } else if (!gamepad2.x && driver.getIntakeMode()== General.IntakeMode.OUTTAKE && !outtake) {
                driver.setIntakeMode(General.IntakeMode.LOCK);
            }

            if (gamepad2.a) { // start intaking
                driver.setWeaponsState(General.WeaponsState.INTAKING);
            }
            if (driver.getCurrentPos().getY() < 50 && allowAutoIntake && !autoIntaking) { // when near the pickup zone, start the intake automatically
                if (redAlliance) {
                    if (driver.getCurrentPos().getX() < 72) {
                        autoIntaking = true;
                        driver.setWeaponsState(General.WeaponsState.INTAKING);
                    }
                } else {
                    if (driver.getCurrentPos().getX() > 72) {
                        autoIntaking = true;
                        driver.setWeaponsState(General.WeaponsState.INTAKING);
                    }
                }
            } else if (driver.getCurrentPos().getY()>=50) {
                autoIntaking = false;
            }
            if (driver.getIntakeMode() == General.IntakeMode.INTAKE && allowAutoHolding) { // when color sensor sees pixel, grab it
                if (driver.getLeftHasPixel()) {
                    switch (driver.getClawMode()) {
                        case OPEN:
                            driver.setClawMode(General.ClawMode.LEFT);
                            break;
                        case LEFT:
                            break;
                        case RIGHT:
                            driver.setClawMode(General.ClawMode.BOTH);
                            break;
                        case BOTH:
                            break;
                    }
                }
                if (driver.getRightHasPixel()) {
                    switch (driver.getClawMode()) {
                        case OPEN:
                            driver.setClawMode(General.ClawMode.RIGHT);
                            break;
                        case LEFT:
                            driver.setClawMode(General.ClawMode.BOTH);
                            break;
                        case RIGHT:
                            break;
                        case BOTH:
                            break;
                    }
                }
                if (driver.getRightHasPixel() && driver.getLeftHasPixel()) { // if both claws see pixels, stop intaking
                    driver.setWeaponsState(General.WeaponsState.HOLDING);
                    outtakeTimer.reset();
                    outtake = true;
                }
            }
            if (allowAutoDeposit && driver.getClawLiftPos()>=0.9) { // if you are depositing, drop the pixels if the FSR touches the backdrop
                if (driver.getFSRPressed()) { // nest this in here to decrease loop times
                    driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                    driver.setSlidesDepositTarget(driver.getSlidesLength());
                }
            }
            if (outtake) { // outtake for a set amount of time
                if (outtakeTimer.time()<0.5) {
                    //driver.setIntakeMode(General.IntakeMode.OUTTAKE);
                } else {
                    driver.setIntakeMode(General.IntakeMode.LOCK);
                    outtake = false;
                }
            }






            if (gamepad2.touchpad_finger_1_x < 0 && gamepad2.touchpad_finger_1) { // enable/disable auto intake
                if (allowAutoIntake) {
                    allowAutoIntake = false;
                } else {
                    allowAutoIntake = true;
                }
                gamepad2.rumble(1, 0, 200);
                while (gamepad2.touchpad_finger_1) {}
            } else if (gamepad2.touchpad_finger_1_x > 0 && gamepad2.touchpad_finger_1) { // enable/disable auto grab (color sensors)
                if (allowAutoHolding) {
                    allowAutoHolding = false;
                } else {
                    allowAutoHolding = true;
                }
                gamepad2.rumble(0, 1, 200);
                while (gamepad2.touchpad_finger_1) {}
            }
            if (gamepad2.b) { // enable/disable auto deposit (FSR)
                if (allowAutoDeposit) {
                    allowAutoDeposit = false;
                    gamepad2.setLedColor(255, 0, 0, -1);
                } else {
                    allowAutoDeposit = true;
                    gamepad2.setLedColor(0, 255, 0, -1);
                }
                while (gamepad2.b) {}
            }
            /*if (gamepad1.touchpad && !g1lt) { // toggle robot front
                g1lt = true;
                if (intakeFront) {
                    intakeFront = false;
                } else {
                    intakeFront = true;
                }
            } else {
                g1lt = false;
            }

             */



            if (gamepad1.back) {
                driver.resetSlidesEncoder();
            }
            if (gamepad2.back) {
                if (driver.getFlipperDisable()) {
                    driver.setFlipperDisable(false);
                } else {
                    driver.setFlipperDisable(true);
                }
                while (gamepad2.back) {}
            }
            if (gamepad2.start) {
                if (driver.getSlidesDisable()) {
                    driver.setSlidesDisable(false);
                } else {
                    driver.setSlidesDisable(true);
                }
                while (gamepad2.start) {}
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


            if (hanging && gamepad1.left_trigger<0.7) {
                //driver.drive(0, 0.2, 0, false);
                driver.setClawLiftPos(true);
                if (hangtime.time() > 0.6) {
                    hanging = false;
                }
            } else if (gamepad1.left_trigger>=0.7) {
                driver.drive(gamepad1.left_stick_x/2, -0.5, gamepad1.right_stick_x, false);
                hangtime.reset();
                hanging = true;
            } else if (gamepad1.right_trigger > 0.7){
                if (intakeFront) {
                    driver.drive(-(gamepad1.left_stick_x / 3), gamepad1.left_stick_y / 3, gamepad1.right_stick_x / 3, superMegaDrive);
                } else {
                    driver.drive((gamepad1.left_stick_x / 3), -gamepad1.left_stick_y / 3, gamepad1.right_stick_x / 3, superMegaDrive);
                }
            } else {
                if (intakeFront) {
                    driver.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
                } else {
                    driver.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
                }
            }
            telemetry.addData("current pos", driver.getCurrentPos().toString());
            telemetry.addData("IMU Heading", driver.getIMUHeading());
            telemetry.addData("claw lift pos", driver.getClawLiftPos());
            telemetry.addData("weapons state", driver.getWeaponsState());
            telemetry.addData("left color", driver.getLeftHasPixel());
            telemetry.addData("Intake Front?", intakeFront);
            telemetry.addData("touchpad", gamepad2.touchpad_finger_1_x);
            telemetry.addData("Claw State", driver.getClawMode().toString());
            telemetry.addData("slides target", driver.getSlidesTarget());
            telemetry.addData("slides deposit target", driver.slidesDepositTarget);
            telemetry.addData("slides pos", driver.getSlidesLength());
            telemetry.addData("loop speed", driver.loopSpeed);
            telemetry.update();
        }
    }

}
