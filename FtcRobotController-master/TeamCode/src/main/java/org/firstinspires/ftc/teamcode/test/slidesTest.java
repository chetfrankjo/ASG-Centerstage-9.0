package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class slidesTest extends LinearOpMode {
    public static double cons = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx sl = hardwareMap.get(DcMotorEx.class, "slidesL");
        DcMotorEx sr = hardwareMap.get(DcMotorEx.class, "slidesR");
        sl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {


            sl.setPower(-gamepad1.left_stick_y);
            sr.setPower(gamepad1.left_stick_y);

            telemetry.addData("Slides Raw", sl.getCurrentPosition());
            telemetry.addData("Slides Length (in)", sl.getCurrentPosition()/cons);
            telemetry.update();

        }
    }
}
