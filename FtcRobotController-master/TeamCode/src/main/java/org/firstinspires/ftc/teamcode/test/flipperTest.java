package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class flipperTest extends LinearOpMode {
    public static double constant = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo flipper = hardwareMap.get(Servo.class, "armLift");
        DcMotorEx slidesL = hardwareMap.get(DcMotorEx.class, "slidesL");
        slidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotorEx slidesR = hardwareMap.get(DcMotorEx.class, "slidesR");
        slidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                flipper.setPosition(0.299);
            }
            if (gamepad1.b) {
                flipper.setPosition(0.765);
            }
            slidesL.setPower(gamepad1.right_stick_y);
            slidesR.setPower(gamepad1.right_stick_y);
            // angle / ticks
            telemetry.addData("pos", flipper.getPosition());
            telemetry.update();

        }
    }
}
