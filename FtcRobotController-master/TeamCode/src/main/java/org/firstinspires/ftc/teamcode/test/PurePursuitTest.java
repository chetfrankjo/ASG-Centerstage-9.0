package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@Config
@TeleOp
public class PurePursuitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj = new Trajectory(0, 0, 0.4, 15);
        traj.addPoint(0, 32, 0);
        //traj.addPoint(48, 0, -180);
        traj.build();

        RobotDriver driver = new RobotDriver(hardwareMap, true);

        waitForStart();
        while (opModeIsActive()) {
            driver.update();
            driver.followCurve(new Trajectory(0, 0,0.2, 5, 0.3).addPoint(0, 10, 0).addPoint(0, 20, 0).addPoint(-5, 25, 0).build().path);

            telemetry.addData("x", driver.getCurrentPos().getX());
            telemetry.addData("y", driver.getCurrentPos().getY());
            telemetry.addData("head", driver.getCurrentPos().getHeading());
            telemetry.update();
        }
    }
}
