package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
@TeleOp
public class intakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
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
            intake.setPower(intakePower);
            telemetry.addData("current", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("current", intake.getCurrent(CurrentUnit.AMPS));
            dash.sendTelemetryPacket(packet);

        }
    }
}
