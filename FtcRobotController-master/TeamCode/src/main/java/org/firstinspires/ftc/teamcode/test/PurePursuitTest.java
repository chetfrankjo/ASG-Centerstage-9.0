package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class PurePursuitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj = new Trajectory(0, 0, 0.6, 10);
        traj.addPoint(0, 48, 0);
        traj.addPoint(48, 48, -90);
        traj.addPoint(48, 0, -180);
        traj.build();

        RobotDriver driver = new RobotDriver(hardwareMap, true);
        waitForStart();
        while (opModeIsActive()) {
            driver.update();
            driver.followCurve(traj.path, 0);

            telemetry.addData("x", driver.getCurrentPos().getX());
            telemetry.addData("y", driver.getCurrentPos().getY());
            telemetry.addData("head", driver.getCurrentPos().getHeading());
            telemetry.update();
        }
    }
}
