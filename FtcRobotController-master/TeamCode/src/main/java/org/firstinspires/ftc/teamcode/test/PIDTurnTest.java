package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp(group = "a")
@Config
public class PIDTurnTest extends LinearOpMode{
    double Tcurrent_time, Tprevious_time;
    double Tcurrent_error, Tprevious_error;
    double Tp, Ti, Td, Tmax_i, Ttotal;
    long Tlast_time;

    public static double Tk_p = 0.0132;
    public static double Tk_i = 0;
    public static double Tk_d = 0;
    double Tangle = 0;
    boolean press = false;
    @Override
    public void runOpMode(){
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        Tmax_i = 1;
        waitForStart();
        if (opModeIsActive()){
            driver.resetIMUHeading();
            while (opModeIsActive()) {


                Tcurrent_time = System.nanoTime();
                Tcurrent_error = Tangle - driver.pullIMUHeading();

                telemetry.update();

                if(gamepad1.a){
                    Tangle = 10;
                    Tprevious_error = 0;
                    press = true;
                } else if (gamepad1.b){
                    Tangle = 0;
                    Tprevious_error = 0;
                    press = true;
                }
                if (press && !gamepad1.a && !gamepad1.b) {
                    press = false;
                }
                Tp = Tk_p * Tcurrent_error;
                Ti += Tk_i * (Tcurrent_error * (Tcurrent_time / 1000000000));
                if (Ti > Tmax_i) {
                    Ti = Tmax_i;
                } else if (Ti < -Tmax_i) {
                    Ti = -Tmax_i;
                }
                Td = Tk_d * (Tcurrent_error - Tprevious_error) / (Tcurrent_time);
                Ttotal = Tp + Ti + Td;
                driver.drive(0, 0, Ttotal, false);
                Tprevious_error = Tcurrent_error;
                Tprevious_time = Tcurrent_time;

                driver.update();
            }
        }
    }
}
