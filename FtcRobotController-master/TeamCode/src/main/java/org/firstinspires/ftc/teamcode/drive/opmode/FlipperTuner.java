package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(group = "z")
public class FlipperTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo flipper = hardwareMap.get(CRServo.class, "armLift");
        AnalogInput enc = hardwareMap.get(AnalogInput.class, "armAxon");

        waitForStart();

        while (opModeIsActive()) {
            flipper.setPower(-gamepad1.left_stick_y);
            telemetry.addData("pos corrected", enc.getVoltage()/3.3*360-202);
            telemetry.addData("pos raw (deg)", enc.getVoltage()/3.3*360);
            telemetry.addData("raw voltage", enc.getVoltage());
            telemetry.update();
        }
    }
}
