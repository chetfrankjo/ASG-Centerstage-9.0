package org.firstinspires.ftc.teamcode.drive.CenterStage;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(group = "a")
public class CalesFirstProgram_whoLetBroCode extends LinearOpMode{
    double current_time;
    double current_error;
    double p, i, d, k_p, k_i, k_d;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    public void runOpMode() throws InterruptedException {
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("X Position", );
            driver.drive(gamepad1.left_stick_x + gamepad2.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, false);
            driver.update();
        }
    }

}
