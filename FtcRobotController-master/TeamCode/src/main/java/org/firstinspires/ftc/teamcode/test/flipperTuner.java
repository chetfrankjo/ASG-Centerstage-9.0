package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Constants;
@Config
@TeleOp
public class flipperTuner extends LinearOpMode {
    public static double target = -0;
    double flipperAngle, flipperI, previousFlipperError;
    public static double pc, ic, dc, fc, constant;
    long lastLoopTime = System.nanoTime();
    double loopSpeed;
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx flipper = hardwareMap.get(DcMotorEx.class, "flipper");
        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FtcDashboard dash;
        dash = FtcDashboard.getInstance();
        waitForStart();
        int loops = 0;

        while (opModeIsActive()) {
            loops++;

            flipperAngle = -flipper.getCurrentPosition();


            long currentTime = System.nanoTime();
            if (loops == 1){
                lastLoopTime = currentTime;
            }
            loopSpeed = (currentTime - lastLoopTime)/1000000000.0;
            lastLoopTime = currentTime;

            /*double error = (target- flipperAngle);
            double p = pc * error;
            flipperI +=ic * error * loopSpeed;
            double d = dc * (error- previousFlipperError) / loopSpeed;
            double power;
            if (flipperAngle > 180) {
                power = (p+ flipperI +d + fc);
            } else {
                power = (p + flipperI + d);
            }
             */
            double error = (target- flipperAngle);

            double p = pc*error;
            double f = fc*Math.sin(Math.toRadians(flipperAngle*constant));
            double d = dc * (error- previousFlipperError) / loopSpeed;

            previousFlipperError = error;

            flipper.setPower(p + d + f);

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
