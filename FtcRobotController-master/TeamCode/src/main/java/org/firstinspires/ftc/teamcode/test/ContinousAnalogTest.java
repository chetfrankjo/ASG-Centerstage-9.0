package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.drive.Sensors.ContinousAnalogAxon;

@TeleOp
public class ContinousAnalogTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput input = hardwareMap.get(AnalogInput.class, "gantryEnc");
        ContinousAnalogAxon enc = new ContinousAnalogAxon(input);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("current pos", enc.getCurrentPosition());
            telemetry.addData("current voltage", enc.getVoltage());
            telemetry.update();
        }
    }
}
