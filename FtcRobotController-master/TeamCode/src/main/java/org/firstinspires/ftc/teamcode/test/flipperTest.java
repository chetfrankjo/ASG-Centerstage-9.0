package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@Config
@TeleOp
public class flipperTest extends LinearOpMode {
    public static double constant = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flipper = hardwareMap.get(DcMotorEx.class, "flipper");
        flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {

            flipper.setPower(-gamepad1.left_stick_y);
            // angle / ticks
            telemetry.addData("pos", flipper.getCurrentPosition()*constant);
            telemetry.update();

        }
    }
}
