package org.firstinspires.ftc.teamcode.drive.CenterStage;

import android.util.Size;

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
public class PIDTurnTest extends LinearOpMode{
    double Tcurrent_time, Tprevious_time;
    double Tcurrent_error, Tprevious_error;
    double Tp, Ti, Td, Tmax_i, Ttotal;
    ttimer = new ElapsedTime();

    double Tk_p = 0.1;
    double Tk_i = 0;
    double Tk_d = 0;
    @Override
    public void runOpMode(){
        RobotDriver driver = new RobotDriver(hardwareMap, false);
        waitForStart();
        if (opModeIsActive()){
            Tcurrent_time = ttimer.time();
            Tcurrent_error = Math.toRadians(90)-driver.getIMUHeading();
            Tp = Tk_p*Tcurrent_error;
            Ti += Tk_i*(Tcurrent_error*(Tcurrent_time));
            if (Ti>Tmax_i){
                Ti = Tmax_i;
            } else if (Ti < -Tmax_i) {
                Ti = -Tmax_i;
            }
            Td = Tk_d*(Tcurrent_error-Tprevious_error)/(Tcurrent_time);
            Ttotal = Tp+Ti+Td;
            driver.drive(0,0,Ttotal, false);
            Tprevious_error = Tcurrent_error;
            Ttimer.reset();
            driver.update();
        }
    }
}
