package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class flipperTuner extends LinearOpMode {
    public static double target = -0;
    double flipperAngle, flipperI, previousFlipperError;
    public static double pc, ic, dc, fc;
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

            flipperAngle = flipper.getCurrentPosition() * 0;


            long currentTime = System.nanoTime();
            if (loops == 1){
                lastLoopTime = currentTime;
            }
            loopSpeed = (currentTime - lastLoopTime)/1000000000.0;
            lastLoopTime = currentTime;

            double error = (target- flipperAngle);
            double p = pc * error;
            flipperI +=ic * error * loopSpeed;
            double d = dc * (error- previousFlipperError) / loopSpeed;
            double power = (p+ flipperI +d + (Math.signum(error)*fc));


            previousFlipperError = error;




            flipper.setPower(power);

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
