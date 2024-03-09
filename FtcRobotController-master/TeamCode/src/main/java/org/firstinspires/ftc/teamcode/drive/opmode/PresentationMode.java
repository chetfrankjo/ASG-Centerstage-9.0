package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class PresentationMode extends LinearOpMode {
    boolean l = true;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.resetSlidesEncoder();
        driver.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        driver.setWeaponsState(General.WeaponsState.HOLDING);
        driver.setSlidesDisable(false);



        driver.setClawMode(General.ClawMode.BOTH);

        driver.storeAll();
        driver.setSpeedyDeposit(true);
        driver.update();

        driver.setSlidesDepositTarget(12);

        waitForStart();

        while (opModeIsActive()) {

            if (l) {
                driver.resetSlidesEncoder();
                driver.setWeaponsState(General.WeaponsState.EXTEND);
            }
            l = false;
            driver.update();
            driver.drive(0, 0, 0, false);

            if (driver.getFSRPressed()) {
                driver.setWeaponsState(General.WeaponsState.DEPOSIT);
            }
            telemetry.addData("slides pos", driver.getSlidesLength());
            telemetry.update();

            sleep(20);

        }
    }
}
