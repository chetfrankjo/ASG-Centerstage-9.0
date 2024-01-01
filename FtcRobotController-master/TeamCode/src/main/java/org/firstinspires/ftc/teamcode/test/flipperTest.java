package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class flipperTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flipper = hardwareMap.get(DcMotorEx.class, "flipper");
        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            flipper.setPower(-gamepad1.left_stick_y);
            telemetry.addData("pos", flipper.getCurrentPosition());
            telemetry.update();

        }
    }
}
