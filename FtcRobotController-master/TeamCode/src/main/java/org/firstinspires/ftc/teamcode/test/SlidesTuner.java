package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Constants;

@Config
@TeleOp
public class SlidesTuner extends LinearOpMode {
    public static double slidesTarget = 0;
    double slidesLength, slidesI, previousSlidesError;
    public static double pc, ic, dc, fc;
    long lastLoopTime = System.nanoTime();
    double loopSpeed;


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx sl = hardwareMap.get(DcMotorEx.class, "slidesL");
        DcMotorEx sr = hardwareMap.get(DcMotorEx.class, "slidesR");
        sl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        int loops = 0;
        while (opModeIsActive()) {

            slidesLength = sl.getCurrentPosition()/Constants.AssemblyConstants.slideTickToInch;


            loops++;
            long currentTime = System.nanoTime();
            if (loops == 1){
                lastLoopTime = currentTime;
            }
            loopSpeed = (currentTime - lastLoopTime)/1000000000.0;
            lastLoopTime = currentTime;

            double error = (slidesTarget-slidesLength);
            double p = pc * error;
            slidesI+=ic * error * loopSpeed;
            double d = dc * (error-previousSlidesError) / loopSpeed;
            double power = (p+slidesI+d + (Math.signum(error)*fc));


            previousSlidesError = error;

            sl.setPower(power);
            sr.setPower(-power);


        }
    }
}
