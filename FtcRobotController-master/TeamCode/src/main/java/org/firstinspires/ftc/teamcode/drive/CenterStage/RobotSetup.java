package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.Logger;

@TeleOp
public class RobotSetup extends LinearOpMode {

    String msg = "  \n" +
            "               BLUE_N --V   \n" +
            "      ------------.-'   _  '-..+   \n" +
            "               |   _  ( Y )  _  |  \n" +
            "   BLUE_S ->  |  ( X )  _  ( B ) |   <-- RED_N \n" +
            "         ___  '.      ( A )     /|   \n" +
            "       .'    '.    '-._____.-'  .' \n" +
            "      |       |         ^       | \n" +
            "       '.___.' '.       |       | \n" +
            "                '.    RED_S     /  \n" +
            "                  |.          .\n" +
            "                   |________|";

    boolean advance = false;
    General.AllianceLocation location;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("ENTER AUTO START ZONE");
        telemetry.addLine("            BLUE_N --V   ");
        telemetry.addLine("   ------------.-'   _  '-..+   ");
        telemetry.addLine("            |   _  ( Y )  _  |  ");
        telemetry.addLine("BLUE_S ->  |  ( X )  _  ( B ) | <-- RED_N ");
        telemetry.addLine("      ___  '.      ( A )     /|   ");
        telemetry.addLine("    .'    '.    '-._____.-'  .' ");
        telemetry.addLine("   |       |         ^       | ");
        telemetry.addLine("    '.___.' '.       |       | ");
        telemetry.addLine("             '.    RED_S     /  ");
        telemetry.addLine("               |.          .");
        telemetry.addLine("                |________|");

        telemetry.update();

        while (!advance && opModeInInit()) {
            if (gamepad1.a) {
                location = General.AllianceLocation.RED_SOUTH;
                Logger a = new Logger("Alliance",false);
                String l = "red_south";
                a.addData(l);
                a.update();
                a.close();
                advance=true;
            }
            if (gamepad1.b) {
                location = General.AllianceLocation.RED_NORTH;
                Logger a = new Logger("Alliance",false);
                String l = "red_north";
                a.addData(l);
                a.update();
                a.close();
                advance=true;
            }
            if (gamepad1.x) {
                location = General.AllianceLocation.BLUE_SOUTH;
                Logger a = new Logger("Alliance",false);
                String l = "blue_south";
                a.addData(l);
                a.update();
                a.close();
                advance=true;
            }
            if (gamepad1.y) {
                location = General.AllianceLocation.BLUE_NORTH;
                Logger a = new Logger("Alliance",false);
                String l = "blue_north";
                a.addData(l);
                a.update();
                a.close();
                advance=true;
            }
        }
        advance = true; //TODO: Make this false once you add auto path choosing
        telemetry.clearAll();
        while (!advance && opModeInInit()) {
            telemetry.addLine("SELECT DESIRED AUTO PATH");
            telemetry.update();

            //TODO: choose auto path
        }
        telemetry.clearAll();
        while (opModeInInit()) {
            telemetry.addLine("PRESS START TO CONFIRM SETTINGS:\n");
            telemetry.addData("Alliance Location", location.toString());
            telemetry.addData("Auto Path", "default");
        }

    }
}
