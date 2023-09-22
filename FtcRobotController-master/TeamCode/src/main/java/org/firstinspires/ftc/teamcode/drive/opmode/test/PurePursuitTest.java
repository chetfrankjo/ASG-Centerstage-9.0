package org.firstinspires.ftc.teamcode.drive.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;

@TeleOp
public class PurePuresuitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj = new Trajectory(0, 0, 0.7, 25);
        traj.addPoint(0, 50, 0);
        traj.addPoint(40, 50, -90);
        traj.addPoint(40, 10, -180);
        traj.build();

        RobotDriver driver = new RobotDriver(hardwareMap, true);
        waitForStart();
        while (opModeIsActive()) {
            driver.update();
            driver.followCurve(traj.path, 0);
        }
    }
}
