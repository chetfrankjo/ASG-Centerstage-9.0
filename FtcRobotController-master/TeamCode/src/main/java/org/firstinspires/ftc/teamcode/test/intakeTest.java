package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
@TeleOp
public class intakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        double intakePower = 0;
        FtcDashboard dash = FtcDashboard.getInstance();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                intakePower = -1;
            }
            if (gamepad1.b) {
                intakePower = 0;
            }
            if (gamepad1.x) {
                intakePower = 1;
            }


            //intake.setPower(intakePower);

            if (gamepad1.dpad_down) {
                intakePower -= 0.05;
                while (gamepad1.dpad_down) {}
            }
            if (gamepad1.dpad_up) {
                intakePower += 0.05;
                while (gamepad1.dpad_up) {}
            }
            intake.setPower(intakePower);
            telemetry.addData("intake power", intakePower);
            telemetry.addData("intake pos", intake.getCurrentPosition());
            telemetry.addData("current", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("current", intake.getCurrent(CurrentUnit.AMPS));
            dash.sendTelemetryPacket(packet);

        }
    }
}
