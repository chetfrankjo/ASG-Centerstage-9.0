package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Constants;
@Config
@TeleOp
public class flipperTuner extends LinearOpMode {
    public static double target = 0;
    double flipperAngle, flipperI, previousFlipperError=0;
    public static double pc=0, ic=0, dc=0, fc=0.13, constant;
    long lastLoopTime = System.nanoTime();
    double loopSpeed;
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo flipper = hardwareMap.get(CRServo.class, "armLift");
        AnalogInput enc = hardwareMap.get(AnalogInput.class, "armAxon");
        FtcDashboard dash;
        dash = FtcDashboard.getInstance();
        waitForStart();
        int loops = 0;

        while (opModeIsActive()) {
            loops++;

            flipperAngle = enc.getVoltage()/3.3*360-202;


            long currentTime = System.nanoTime();
            if (loops == 1){
                lastLoopTime = currentTime;
            }
            loopSpeed = (currentTime - lastLoopTime)/1000000000.0;
            lastLoopTime = currentTime;

            double error = (target- flipperAngle);

            double p = pc*error;
            double f = fc*Math.sin(Math.toRadians(flipperAngle));
            double d = dc * (error- previousFlipperError) / loopSpeed;

            previousFlipperError = error;

            flipper.setPower(f+p+d);

            telemetry.addData("f", f);
            telemetry.addData("pos", flipperAngle);
            telemetry.addData("target", target);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("pos", flipperAngle);
            packet.put("target", target);
            dash.sendTelemetryPacket(packet);
        }
    }
}
