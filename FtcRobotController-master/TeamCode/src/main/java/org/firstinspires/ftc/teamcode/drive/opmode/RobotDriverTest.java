package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

import java.util.logging.Logger;

@TeleOp(group = "c")
public class RobotDriverTest extends LinearOpMode {
    boolean fieldCentric = false;
    double headingAccum = 0;
    double lastHeading = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        driver.setDriveZeroPower(FLOAT);
        //driver.setSlidesZeroPower(FLOAT);
        //driver.setTurretZeroPower(FLOAT);
        //driver.setv4barZeroPower(FLOAT);
        driver.localizer.setEstimatePos(0, 0, 0);
        Pose2d currentPos;
        driver.setLocalizationMode(General.LocalMode.ODOMETRY);
        driver.resetIMUHeading();
        driver.setIntakeMode(General.IntakeMode.LOCK);
        driver.update();

        waitForStart();
        while (opModeIsActive()) {
            driver.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, fieldCentric);


            if (gamepad1.a) {
                if (!fieldCentric) {
                    fieldCentric = true;
                } else {
                    fieldCentric = false;
                }
                while (gamepad1.a) {}
            }
            currentPos = driver.getCurrentPos();

            if (gamepad2.a) {
                driver.setWeaponsState(General.WeaponsState.INTAKING);
            }
            if (gamepad2.b) {
                driver.setWeaponsState(General.WeaponsState.HOLDING);
            }
            if (gamepad2.x) {
                driver.setWeaponsState(General.WeaponsState.PRIMED);
            }
            if (gamepad2.y) {
                driver.setWeaponsState(General.WeaponsState.DEPOSIT);
            }

            double heading = currentPos.getHeading();
            double deltaHeading = heading - lastHeading;
            headingAccum += Angle.normDelta(deltaHeading);
            lastHeading = heading;
            //double[] systemCoordinates = driver.getAssemblyCoordinate();

            //telemetry.addData("x", systemCoordinates[0]);
            //telemetry.addData("y", systemCoordinates[1]);
            //telemetry.addData("z", systemCoordinates[2]);
            telemetry.addData("-----", "----");
            telemetry.addData("Global X", currentPos.getX());
            telemetry.addData("Global Y", currentPos.getY());
            telemetry.addData("Global Heading", currentPos.getHeading());
            driver.pullIMUHeading();
            telemetry.addData("IMU Heading", driver.getIMUHeading());
            telemetry.addData("accum", headingAccum);
            telemetry.addData("loop speed", driver.loopSpeed);
            telemetry.update();
            driver.update();
        }
    }
}
