package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Config
@TeleOp
public class flipperTest extends LinearOpMode {
    public static double constant = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo flipper = hardwareMap.get(CRServo.class, "armLift");
        DcMotorEx slidesL = hardwareMap.get(DcMotorEx.class, "slidesL");
        slidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotorEx slidesR = hardwareMap.get(DcMotorEx.class, "slidesR");
        slidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AnalogInput enc = hardwareMap.get(AnalogInput.class, "armAxon");
        waitForStart();

        while (opModeIsActive()) {
            flipper.setPower(-gamepad1.left_stick_y);
            slidesL.setPower(gamepad1.right_stick_y);
            slidesR.setPower(gamepad1.right_stick_y);
            // angle / ticks
            telemetry.addData("pos", enc.getVoltage()/3.3*360-202);
            telemetry.update();

        }
    }
}
