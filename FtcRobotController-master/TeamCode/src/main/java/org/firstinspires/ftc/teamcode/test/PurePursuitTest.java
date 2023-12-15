package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;
@Disabled
@Config
@TeleOp
public class PurePursuitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj = new Trajectory(0, 0, 0.4, 8).addPoint(0, -11, 180).addPointSpeed(-18, -11, 90, 0.3).build();
        //traj.addPoint(48, 0, -180);

        RobotDriver driver = new RobotDriver(hardwareMap, false);

        waitForStart();
        while (opModeIsActive()) {
            driver.update();
            driver.followCurve(traj.path);

            telemetry.addData("x", driver.getCurrentPos().getX());
            telemetry.addData("y", driver.getCurrentPos().getY());
            telemetry.addData("head", driver.getCurrentPos().getHeading());
            telemetry.update();
        }
    }
}
