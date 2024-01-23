package org.firstinspires.ftc.teamcode.drive.CenterStage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.drive.Logger;

@TeleOp(group = "a")
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
    double timerOffset = 0, timerOffset2 = 0, timerOffset3 = 0;
    General.AllianceLocation location;
    General.ParkLocation parkLocation;
    General.AutoMode autoMode;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("ENTER AUTO START ZONE");
        telemetry.addLine("                   BLUE_N --V   ");
        telemetry.addLine("--------------------.-'         _         '-..+   ");
        telemetry.addLine("                        |      _    ( Y )    _    |    ");
        telemetry.addLine("BLUE_S ->   |    ( X )    _    ( B )  |  <-- RED_N ");
        telemetry.addLine("            ___      '.            ( A )          /|   ");
        telemetry.addLine("        .'        '.           '-._____.-'    .' ");
        telemetry.addLine("      |              |                  ^              | ");
        telemetry.addLine("        '.___.' '.                     |              | ");
        telemetry.addLine("                          '.        RED_S       /  ");
        telemetry.addLine("                              |.                    .");
        telemetry.addLine("                                |________|");

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
        advance = false;
        telemetry.clearAll();
        while (!advance && opModeInInit()) {
            telemetry.addLine("SELECT DESIRED PARK LOCATION (Left or Right - use DPAD)\nUP FOR CENTER PARK (no movement)");
            telemetry.update();
            if (gamepad1.dpad_left) {
                parkLocation = General.ParkLocation.LEFT;
                Logger a = new Logger("park",false);
                String l = "left";
                a.addData(l);
                a.update();
                a.close();
                while (gamepad1.dpad_left) {}
                advance=true;
            }
            if (gamepad1.dpad_right) {
                parkLocation = General.ParkLocation.RIGHT;
                Logger a = new Logger("park",false);
                String l = "right";
                a.addData(l);
                a.update();
                a.close();
                while (gamepad1.dpad_right) {}
                advance=true;
            }
            if (gamepad1.dpad_up) {
                parkLocation = General.ParkLocation.CENTER;
                Logger a = new Logger("park",false);
                String l = "center";
                a.addData(l);
                a.update();
                a.close();
                while (gamepad1.dpad_up) {}
                advance=true;
            }
        }

        advance=false;
        telemetry.clearAll();
        while (!advance && opModeInInit()) {
            telemetry.addData("Added Timer - STAGE 1 TIMER", timerOffset);
            telemetry.addLine("Press START to advance");
            if (location== General.AllianceLocation.BLUE_NORTH | location== General.AllianceLocation.RED_NORTH) {
                telemetry.addLine("Auto Timings:\nStandard: 7 Seconds\nCycle: None");
            } else {
                telemetry.addLine("Auto Timings:\nStandard: 11 Seconds\nCycle: None");
            }

            telemetry.update();
            if (gamepad1.dpad_up) {
                timerOffset+=1;
                while (gamepad1.dpad_up) {}
            }
            if (gamepad1.dpad_down) {
                if (timerOffset>0) {
                    timerOffset-=1;
                }
                while (gamepad1.dpad_down) {}
            }
            if (gamepad1.start) {
                while (gamepad1.start) {}
                advance=true;
            }
        }
        advance=false;
        telemetry.clearAll();
        Logger a = new Logger("timer", false);
        String l = String.valueOf(timerOffset);
        a.addData(l);
        a.update();
        a.close();


        advance=false;
        telemetry.clearAll();
        while (!advance && opModeInInit()) {
            telemetry.addData("Added Timer - STAGE 2 TIMER (After dropping pixel on spike mark)", timerOffset);
            telemetry.addLine("Press START to advance");
            telemetry.update();
            if (gamepad1.dpad_up) {
                timerOffset2+=1;
                while (gamepad1.dpad_up) {}
            }
            if (gamepad1.dpad_down) {
                if (timerOffset2>0) {
                    timerOffset2-=1;
                }
                while (gamepad1.dpad_down) {}
            }
            if (gamepad1.start) {
                while (gamepad1.start) {}
                advance=true;
            }
        }
        advance=false;
        telemetry.clearAll();
        Logger b = new Logger("timer2", false);
        String c = String.valueOf(timerOffset2);
        b.addData(c);
        b.update();
        b.close();


        advance=false;
        telemetry.clearAll();
        while (!advance && opModeInInit()) {
            telemetry.addData("Added Timer - STAGE 3 TIMER (After dropping pixel on backdrop)", timerOffset);
            telemetry.addLine("Press START to advance");
            telemetry.update();
            if (gamepad1.dpad_up) {
                timerOffset3+=1;
                while (gamepad1.dpad_up) {}
            }
            if (gamepad1.dpad_down) {
                if (timerOffset3>0) {
                    timerOffset3-=1;
                }
                while (gamepad1.dpad_down) {}
            }
            if (gamepad1.start) {
                while (gamepad1.start) {}
                advance=true;
            }
        }
        advance=false;
        telemetry.clearAll();
        Logger d = new Logger("timer3", false);
        String e = String.valueOf(timerOffset3);
        d.addData(e);
        d.update();
        d.close();

        telemetry.clearAll();
        while (opModeInInit()) {
            telemetry.addLine("PRESS START TO CONFIRM SETTINGS:\n");
            telemetry.addData("Alliance Location", location.toString());
            telemetry.addData("Park Location", parkLocation.toString());
            telemetry.addData("Stage 1 Timer", timerOffset);
            telemetry.addData("Stage 2 Timer", timerOffset2);
            telemetry.addData("Stage 3 Timer", timerOffset3);
            telemetry.update();
        }

    }
}
