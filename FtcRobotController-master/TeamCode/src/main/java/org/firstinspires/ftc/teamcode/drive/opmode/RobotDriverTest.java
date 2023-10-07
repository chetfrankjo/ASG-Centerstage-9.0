package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class RobotDriverTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDriver driver = new RobotDriver(hardwareMap, true);
        driver.setDriveZeroPower(FLOAT);
        //driver.setSlidesZeroPower(FLOAT);
        //driver.setTurretZeroPower(FLOAT);
        //driver.setv4barZeroPower(FLOAT);
        Pose2d currentPos;
        driver.localizer.setEstimatePos(0, 0, 0);
        waitForStart();
        while (opModeIsActive()) {
            driver.driveXY(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            driver.slidesPID = false;
            //driver.setSlidesPower(-gamepad2.left_stick_y);
            //driver.turretPID = false;
            //driver.setTurretPower(gamepad2.right_stick_x);
            //driver.v4barPID = false;
            //driver.setV4barPower(-gamepad2.right_stick_y);
            double[] systemCoordinates = driver.getAssemblyCoordinate();
            currentPos = driver.getCurrentPos();
            telemetry.addData("x", systemCoordinates[0]);
            telemetry.addData("y", systemCoordinates[1]);
            telemetry.addData("z", systemCoordinates[2]);
            telemetry.addData("-----", "----");
            telemetry.addData("Global X", currentPos.getX());
            telemetry.addData("Global Y", currentPos.getY());
            telemetry.addData("Global Heading", currentPos.getHeading());

            telemetry.addData("loop speed", driver.loopSpeed);
            telemetry.update();
            driver.update();
        }
    }
}
