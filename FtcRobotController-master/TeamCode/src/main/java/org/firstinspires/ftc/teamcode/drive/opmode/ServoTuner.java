package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="z")
public class ServoTuner extends LinearOpMode {
    int currentClaw = 0;
    double servoPos = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawL = hardwareMap.get(Servo.class, "lclaw");
        Servo clawR = hardwareMap.get(Servo.class, "rclaw");
        Servo clawLift = hardwareMap.get(Servo.class, "clawLift");
        waitForStart();

        while (opModeIsActive()) {



            if (gamepad1.dpad_up) {
                if (currentClaw < 2) {
                    currentClaw++;
                    servoPos = 0.5;
                } else {
                    currentClaw = 0;
                    servoPos = 0.5;
                }
                while (gamepad1.dpad_up) {}
            }
            if (gamepad1.dpad_down) {
                if (currentClaw > 0) {
                    currentClaw--;
                    servoPos = 0.5;
                } else {
                    currentClaw = 2;
                    servoPos = 0.5;
                }
                while (gamepad1.dpad_down) {}
            }


            if (servoPos < 1 && -gamepad1.right_stick_y > 0) {
                servoPos+= -gamepad1.right_stick_y/100;
            } else if (servoPos > 0 && -gamepad1.right_stick_y < 0) {
                servoPos+= -gamepad1.right_stick_y/100;
            }
            if (servoPos < 1 && gamepad1.y) {
                servoPos += 0.0001;
            }
            if (servoPos > 0 && gamepad1.a) {
                servoPos -= 0.0001;
            }


            telemetry.addData("Current Position", servoPos);
            if (currentClaw == 0) {
                telemetry.addData("Current servo", "Left Claw");
                clawL.setPosition(servoPos);
            } else if (currentClaw == 1) {
                telemetry.addData("Current servo", "Right Claw");
                clawR.setPosition(servoPos);
            } else {
                telemetry.addData("Current servo", "Claw Lift");
                clawLift.setPosition(servoPos);
            }
            telemetry.addLine("Use DPAD to toggle servos");
            telemetry.addLine("Use Right Joystick to move position, use a/y to move slowly");

            telemetry.update();

        }
    }
}
