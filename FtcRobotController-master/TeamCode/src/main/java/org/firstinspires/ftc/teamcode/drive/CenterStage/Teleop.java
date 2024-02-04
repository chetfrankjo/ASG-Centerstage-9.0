package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp(group = "a")
public class Teleop extends LinearOpMode{

    boolean superMegaDrive = false;
    boolean g1Launch = false, g2Launch = false, g1Hang = false, g2Hang = false, hanging =false, tl=false, tr=false, bl=false, br=false;
    boolean redAlliance = false;
    boolean allowSmartDeposit = false, allowAutoHolding=false, allowAutoDeposit=false, intakeFront=false, g1lt=false, outtake = false, autoIntaking = false, waitForDepositToOpenClaw = false, waitingForTimer = false, waitToStopIntake = false, intakingWhileExtending, hasTouchedBoard = false;
    ElapsedTime outtakeTimer, waitForDepositClawTimer, waitToStopIntakeTimer, intakeWhileExtendingTimer;
    int leftColorLoops = 0, rightColorLoops = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        //TODO: do not deposit until driven away from backdrop
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setWeaponsState(General.WeaponsState.HOLDING);
        driver.setClawMode(General.ClawMode.OPEN);
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.setPurpleSouthRelease(true);
        driver.setPurpleNorthRelease(true);
        driver.resetSlidesEncoder();
        //driver.resetFlipperEncoder();

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
        driver.storeAll();
        driver.update();
        ElapsedTime hangtime = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        waitForDepositClawTimer = new ElapsedTime();
        waitToStopIntakeTimer = new ElapsedTime();
        intakeWhileExtendingTimer = new ElapsedTime();
        waitForStart();
        driver.setClawMode(General.ClawMode.OPEN);
        while (opModeIsActive()) {
            driver.update();

            driver.setSlidesPower(gamepad2.right_stick_y); // manual slides
            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                driver.setFlipperPower(-gamepad2.left_stick_y); // manual flipper
            } else {
                driver.setFlipperPower(0);
            }

            if (gamepad2.left_trigger > 0.2 && !tl) { // open left claw
                tl=true;
                switch (driver.getClawMode()) {
                    case LEFT: // right is already open
                        driver.setClawMode(General.ClawMode.OPEN);
                        if (driver.getClawLiftPos()<=0.3) { // if you are in a deposit position, do a full deposit
                            driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                            waitForDepositToOpenClaw = true;
                            driver.setClawMode(General.ClawMode.OPEN);
                            driver.setSlidesDepositTarget(driver.getSlidesLength()); // store deposit height for next time
                        }
                        break;
                    case BOTH:
                        driver.setClawMode(General.ClawMode.RIGHT); // only right grabs now
                        if (driver.getClawLiftPos()<0.3 && allowSmartDeposit) { // if right doesn't have a pixel, then you are doing a full deposit (no point in manually opening the right claw)
                            if (!driver.getRightHasPixel()) { // nest this in here to conserve loop times
                                driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                                driver.setSlidesDepositTarget(driver.getSlidesLength()); // store deposit height for next time
                            }
                        }
                        break;
                }
            }
            if (gamepad2.left_trigger <= 0.2) {
                tl = false;
            }

            if (gamepad2.right_trigger > 0.2 && !tr) { // open right claw
                tr = true;
                switch (driver.getClawMode()) {
                    case RIGHT:
                        driver.setClawMode(General.ClawMode.OPEN);
                        if (driver.getClawLiftPos()<0.3) { // if you are in a deposit position, do a full deposit
                            driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                            driver.setClawMode(General.ClawMode.OPEN);
                            waitForDepositToOpenClaw = true;
                            driver.setSlidesDepositTarget(driver.getSlidesLength()); // store deposit height for next time
                        }
                        break;
                    case BOTH:
                        driver.setClawMode(General.ClawMode.LEFT);
                        if (driver.getClawLiftPos()<0.3&& allowSmartDeposit) { // if left doesn't have a pixel, then you are doing a full deposit
                            if (!driver.getLeftHasPixel()) {
                                driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                                driver.setSlidesDepositTarget(driver.getSlidesLength()); // store deposit height for next time
                            }

                        }
                        break;
                }
            }
            if (gamepad2.right_trigger <= 0.2) {
                tr = false;
            }

            if (gamepad2.left_bumper && !bl) { //grab left claw
                bl = true;
                switch (driver.getClawMode()) {
                    case OPEN:
                        driver.setClawMode(General.ClawMode.LEFT);
                        break;
                    case RIGHT:
                        driver.setClawMode(General.ClawMode.BOTH);
                        if (driver.getIntakeMode() == General.IntakeMode.INTAKE) { // if you are intaking and both claws are grabbed, stop intaking
                            if (driver.getRightHasPixel() && driver.getLeftHasPixel()) {
                                driver.setWeaponsState(General.WeaponsState.HOLDING);
                                //driver.setIntakeMode(General.IntakeMode.INTAKE);
                                waitToStopIntake = true;
                                waitToStopIntakeTimer.reset();
                                outtakeTimer.reset();
                                outtake = true;
                            } else {
                                driver.setWeaponsState(General.WeaponsState.HOLDING);
                                driver.setIntakeMode(General.IntakeMode.INTAKE);
                                waitToStopIntake = true;
                                waitToStopIntakeTimer.reset();
                                outtakeTimer.reset();
                                outtake = true;
                            }
                        }
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
                            if (driver.getLeftHasPixel() && driver.getRightHasPixel()) {
                                driver.setWeaponsState(General.WeaponsState.HOLDING);
                                //driver.setIntakeMode(General.IntakeMode.INTAKE);
                                waitToStopIntake = true;
                                waitToStopIntakeTimer.reset();
                                outtakeTimer.reset();
                                outtake = true;
                            } else {
                                driver.setWeaponsState(General.WeaponsState.HOLDING);
                                driver.setIntakeMode(General.IntakeMode.INTAKE);
                                waitToStopIntake = true;
                                waitToStopIntakeTimer.reset();
                                outtakeTimer.reset();
                                outtake = true;
                            }
                        }
                        break;
                }
            }
            if (!gamepad2.right_bumper) {
                br = false;
            }

            if (waitToStopIntake && waitToStopIntakeTimer.time() > 0.25) { // extra intake movement
                waitToStopIntake = false;
                driver.setIntakeMode(General.IntakeMode.LOCK);
            }

            if (gamepad2.y) { // extend+flip for depositing
                intakingWhileExtending = true;
                intakeWhileExtendingTimer.reset();
            }
            if (intakingWhileExtending && intakeWhileExtendingTimer.time()< 0.5) { // Intake slightly while extending so pixels dont get knocked out
                if (intakeWhileExtendingTimer.time()>0.05&&intakeWhileExtendingTimer.time()<0.12) {
                    driver.setWeaponsState(General.WeaponsState.EXTEND);
                }
                driver.setIntakePower(0.5);
            } else {
                if (intakingWhileExtending) {
                    driver.setIntakeMode(General.IntakeMode.LOCK);
                }
                intakingWhileExtending = false;
            }

            if (waitForDepositToOpenClaw) { // waits to open the claw after depositing
                waitForDepositClawTimer.reset();
                waitForDepositToOpenClaw = false;
                waitingForTimer = true;
            }
            if (waitingForTimer && waitForDepositClawTimer.time()> 0.9) {
                waitingForTimer = false;
                driver.setClawMode(General.ClawMode.OPEN);
            }

            if (gamepad2.x) { // manual outtake
                driver.setIntakePower(-0.5);
            } else if (!gamepad2.x && driver.getIntakeMode()== General.IntakeMode.MANUAL && !outtake) {
                driver.setIntakeMode(General.IntakeMode.LOCK);
            }

            if (gamepad2.a) { // start intaking
                driver.setIntakeMode(General.IntakeMode.INTAKE); // this does nothing to the claw. If you set the weapons state to intaking, it will only open the claw that dosent have a pixel
            }
            if (gamepad2.b) { // stop intake
                driver.setIntakeMode(General.IntakeMode.LOCK);
            }
            if (gamepad2.dpad_up) { // lift wrist
                driver.setClawLiftPos(true);
            }
            if (gamepad2.dpad_down) { // lower wrist
                driver.setClawLiftPos(false);
            }
            if (driver.getCurrentPos().getY() < 50 && allowSmartDeposit && !autoIntaking) { // when near the pickup zone, start the intake automatically
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
                boolean leftHasPixel = driver.getLeftHasPixel();
                boolean rightHasPixel = driver.getRightHasPixel();
                if (leftHasPixel) { // conserving loop time
                    switch (driver.getClawMode()) {
                        case OPEN:
                            if (leftColorLoops > 22) {
                                driver.setClawMode(General.ClawMode.LEFT);
                                leftColorLoops = 0;
                            }
                            break;
                        case LEFT:
                            break;
                        case RIGHT:
                            if (leftColorLoops > 22) {
                                driver.setClawMode(General.ClawMode.BOTH);
                                leftColorLoops = 0;
                                driver.setWeaponsState(General.WeaponsState.HOLDING);
                                leftColorLoops = 0;
                                rightColorLoops = 0;
                                outtakeTimer.reset();
                                outtake = true;
                            }
                            break;
                        case BOTH:
                            break;
                    }
                    leftColorLoops++;
                }
                if (rightHasPixel) {
                    switch (driver.getClawMode()) {
                        case OPEN:
                            if (rightColorLoops > 22) {
                                driver.setClawMode(General.ClawMode.RIGHT);
                                rightColorLoops = 0;
                            }
                            break;
                        case LEFT:
                            if (rightColorLoops > 22) {
                                driver.setClawMode(General.ClawMode.BOTH);
                                rightColorLoops = 0;
                                driver.setWeaponsState(General.WeaponsState.HOLDING);
                                leftColorLoops = 0;
                                rightColorLoops = 0;
                                outtakeTimer.reset();
                                outtake = true;
                            }
                            break;
                        case RIGHT:
                            break;
                        case BOTH:
                            break;
                    }
                    rightColorLoops++;
                }
                /*if (leftHasPixel && rightHasPixel) { // if both claws see pixels, stop intaking
                    driver.setWeaponsState(General.WeaponsState.HOLDING);
                    leftColorLoops = 0;
                    rightColorLoops = 0;
                    outtakeTimer.reset();
                    outtake = true;
                }

                 */
            }
            if (allowAutoDeposit && driver.getClawLiftPos()< 0.3) { // if you are depositing, drop the pixels if the FSR touches the backdrop
                if (driver.getFSRPressed()) { // nest this in here to decrease loop times
                    driver.setWeaponsState(General.WeaponsState.DEPOSIT);
                    driver.setSlidesDepositTarget(driver.getSlidesLength());
                }
            }
            if (outtake) { // outtake for a set amount of time
                if (outtakeTimer.time()<1) {
                    if (outtakeTimer.time()>0.2) {
                        //driver.setIntakeMode(General.IntakeMode.OUTTAKE);
                        driver.setIntakePower(-0.5);
                    }
                } else {
                    driver.setIntakeMode(General.IntakeMode.LOCK);
                    outtake = false;
                }
            }


            if (gamepad2.touchpad_finger_1_x < 0 && gamepad2.touchpad_finger_1) { // enable/disable auto intake
                if (allowSmartDeposit) {
                    allowSmartDeposit = false;
                } else {
                    allowSmartDeposit = true;
                }
                gamepad2.rumble(1, 0, 200);
                while (gamepad2.touchpad_finger_1) {}
            } else if (gamepad2.touchpad_finger_1_x > 0 && gamepad2.touchpad_finger_1) { // enable/disable auto grab (color sensors)
                if (allowAutoHolding) {
                    allowAutoHolding = false;
                    gamepad2.setLedColor(255, 0, 0, -1);
                } else {
                    allowAutoHolding = true;
                    gamepad2.setLedColor(0, 255, 0, -1);
                }
                gamepad2.rumble(0, 1, 200);
                while (gamepad2.touchpad_finger_1) {}
            }
            if (gamepad2.ps) { // enable/disable auto deposit (FSR)

                allowAutoDeposit = true;

                //while (gamepad2.ps) {}
            } else {
                allowAutoDeposit = false;
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
                driver.setSlidesTarget(0);
            }
            /*if (gamepad2.back) {
                if (driver.getFlipperDisable()) {
                    driver.setFlipperDisable(false);
                } else {
                    driver.setFlipperDisable(true);
                }
                while (gamepad2.back) {}
            }

             */
            if (gamepad2.back) {
                driver.resetFlipperEncoder();
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

            if (hanging && gamepad1.dpad_down) {
                if (hangtime.time() > 0.6) {
                    hanging = false;
                }
            } else if (gamepad1.dpad_down) {
                driver.drive(gamepad1.left_stick_x/2+gamepad2.left_stick_x, -0.5, gamepad1.right_stick_x, false);
                hangtime.reset();
                hanging = true;
            } else if (gamepad1.right_trigger > 0.7){
                if (intakeFront) {
                    driver.drive(-(gamepad1.left_stick_x / 3)+gamepad2.left_stick_x, gamepad1.left_stick_y / 3, gamepad1.right_stick_x / 3, superMegaDrive);
                } else {
                    driver.drive((gamepad1.left_stick_x / 3)+gamepad2.left_stick_x, -gamepad1.left_stick_y / 3, gamepad1.right_stick_x / 3, superMegaDrive);
                }
            } else if (gamepad1.left_trigger > 0.7 && allowAutoDeposit) {
                if (!driver.getFSRPressed() && !hasTouchedBoard) {
                    driver.drive(gamepad1.left_stick_x / 2 +gamepad2.left_stick_x, 0.3, gamepad1.right_stick_x, false);
                } else {
                    hasTouchedBoard = true;
                    driver.drive(gamepad1.left_stick_x / 2 +gamepad2.left_stick_x, -0.3, gamepad1.right_stick_x, false);
                }
            } else {
                if (intakeFront) {
                    driver.drive(-gamepad1.left_stick_x+gamepad2.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
                } else {
                    driver.drive(gamepad1.left_stick_x+gamepad2.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, superMegaDrive);
                }
            }
            if (gamepad1.left_trigger <= 0.7) {
                hasTouchedBoard = false;
            }
            telemetry.addData("fsr pressed", driver.getFSRPressed());
            telemetry.addData("flipper pos", driver.getFlipperPosAnalog());
            telemetry.addData("flipper state", driver.getFlipperState());
            telemetry.addData("flipper target", driver.getFlipperTarget());
            telemetry.addData("current pos", driver.getCurrentPos().toString());
            //telemetry.addData("flipper pos", driver.getFLipperPos());
            telemetry.addData("claw lift pos", driver.getClawLiftPos());
            telemetry.addData("weapons state", driver.getWeaponsState());
            telemetry.addData("Claw State", driver.getClawMode().toString());
            telemetry.addData("slides target", driver.getSlidesTarget());
            telemetry.addData("slides deposit target", driver.slidesDepositTarget);
            telemetry.addData("slides pos", driver.getSlidesLength());
            telemetry.addData("loop speed", driver.loopSpeed);
            telemetry.update();
        }
    }

}
