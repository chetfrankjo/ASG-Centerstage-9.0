package org.firstinspires.ftc.teamcode.test;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.RobotDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

@TeleOp(group = "a")
@Config
public class PidTurn extends LinearOpMode {
    double Tcurrent_time, Tprevious_time;
    double Tcurrent_error, Tprevious_error;
    double Tp, Ti, Td, Tmax_i, Ttotal;
    long Tlast_time;
    double Tangle = 0;
    boolean press = false;

    public static double Tk_p = 0.0132;
    public static double Tk_i = 0.00000001;
    public static double Tk_d = 0.0008;

    public void runOpMode(){
        RobotDriver driver = new RobotDriver(hardwareMap, false);

    }
}
