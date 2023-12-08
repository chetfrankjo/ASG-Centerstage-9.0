package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.DataTypes.General;
import org.firstinspires.ftc.teamcode.DataTypes.Trajectory;

import java.util.ArrayList;

public class AutoStorage {


    /*public static Trajectory approach_1 = new Trajectory(132, 84,0.4, 25).addPoint(120, 84, 0).addPoint(108, 72, 0).build();
    public static Trajectory backup = new Trajectory(108, 72, 0.4, 25).addPoint(120, 84, 180).build();
    public static Trajectory approach_2 = new Trajectory(120, 84, 0.4, 25).addPoint(108, 84, 0).addPoint(108, 120, 0).build();
    public static Trajectory park = new Trajectory(108, 120, 0.4, 25).addPoint(84, 120, 90).addPoint(84, 132, 0).build();

     */
    public static ArrayList<Trajectory> generateAutoPaths(General.ParkLocation parkLocation, General.SpikePosition spikePosition, General.AllianceLocation allianceLocation) {
        ArrayList<Trajectory> paths = new ArrayList<>();


        switch (allianceLocation) {
            case RED_SOUTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(135, 34, 0.25, 12, 0.2).addPoint(113, 34, 0).addPoint(107, 29, 0).build());
                        paths.add(new Trajectory(107, 29, 0.6, 10).addPoint(115, 32, 180).build());
                        paths.add(new Trajectory(115, 32, 0.4, 13).addPoint(80, 32, 0).addPoint(80, 85, 0).addPoint(101, 96, 0).addPoint(101, 108, 0).build());
                        paths.add(new Trajectory(101, 108, 0.4, 10).addPoint(96, 108, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(90, 115, 0.4, 8, 0).addPoint(90, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
                                paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 123, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(93, 115, 0.4, 8, 0).addPoint(93, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 126, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(135, 34, 0.25, 12, 0.2).addPoint(107, 35, 0).build());
                        paths.add(new Trajectory(107, 34, 0.6, 10).addPoint(120, 25, 180).build());
                        paths.add(new Trajectory(120, 25, 0.4, 13).addPoint(77, 25, 0).addPoint(77, 85, 0).addPoint(103, 96, 0).addPoint(103, 108, 0).build());
                        paths.add(new Trajectory(103, 108, 0.4, 10).addPoint(103, 108, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(103, 115, 0.4, 8, 0).addPoint(103, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
                                paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 123, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(103, 115, 0.4, 8, 0).addPoint(103, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 126, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(135, 34, 0.25, 12, 0.2).addPoint(113, 34, 0).addPoint(107, 38, 0).build());
                        paths.add(new Trajectory(107, 38, 0.6, 10).addPoint(115, 32, 180).build());
                        paths.add(new Trajectory(115, 32, 0.4, 13).addPoint(78, 32, 0).addPoint(78, 85, 0).addPoint(101, 96, 0).addPoint(101, 108, 0).build());
                        paths.add(new Trajectory(101, 108, 0.4, 10).addPoint(111, 108, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(111, 115, 0.4, 8, 0).addPoint(111, 104, 180).addPointSpeed(78, 104, 90, 0.3).build());
                                paths.add(new Trajectory(78, 104, 0.3, 20).addPoint(78, 123, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 126, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case RED_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(135, 84, 0.25, 12, 0.2).addPoint(113, 84, 0).addPoint(107, 80, 0).build());
                        paths.add(new Trajectory(107, 80, 0.6, 5).addPoint(120, 88, 180).build());
                        paths.add(new Trajectory(120, 88, 0.4, 13).addPoint(105, 88, 0).addPoint(101, 96, 0).addPoint(101, 112, 0).build());
                        paths.add(new Trajectory(101, 112, 0.4, 10).addPoint(93, 112, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(93, 115, 0.4, 8, 0).addPoint(93, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
                                paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 126, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(93, 115, 0.4, 8, 0).addPoint(93, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 126, 0).build());
                                break;
                        }
                        break;
                    case CENTER:

                        paths.add(new Trajectory(135, 84, 0.5, 15).addPoint(108, 84, 0).build());
                        paths.add(new Trajectory(108, 84, 0.6, 5).addPoint(115, 84, 180).build());
                        paths.add(new Trajectory(115, 84, 0.4, 15).addPoint(101, 84, 0).addPoint(101, 95, 0).build());
                        paths.add(new Trajectory(101, 95, 0.4, 8).addPoint(101, 109, 0).build()); // dud point
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(101, 109, 0.4, 8, 0).addPoint(101, 104, 180).addPointSpeed(77, 104, 90, 0.3).build());
                                paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 126, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(101, 109, 0.4, 8, 0).addPoint(101, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 126, 0).build());
                                break;
                        }


                        paths.add(new Trajectory(0, 0, 0.5, 15).addPoint(0, 27, 0).build());
                        paths.add(new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build());
                        paths.add(new Trajectory(0, 10, 0.4, 15).addPoint(0, 34, 0).addPoint(25, 34, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(25, 34, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 59, 90).build());
                                paths.add(new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(25, 34, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 8, -90).build());
                                paths.add(new Trajectory(20, 8, 0.3, 20).addPoint(42, 8, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(135, 84, 0.25, 12, 0.2).addPoint(113, 84, 0).addPoint(107, 89, 0).build());
                        paths.add(new Trajectory(107, 88, 0.6, 5).addPoint(125, 88, 180).build());
                        paths.add(new Trajectory(125, 88, 0.4, 13).addPoint(110, 88, 0).addPoint(105, 96, 0).addPoint(105, 112, 0).build());
                        paths.add(new Trajectory(105, 112, 0.4, 10).addPoint(110, 112, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPointSpeed(78, 104, 90, 0.3).build());
                                paths.add(new Trajectory(78, 104, 0.3, 20).addPoint(78, 126, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 126, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case BLUE_SOUTH:
                switch (spikePosition) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(9, 34, 0.25, 12, 0.2).addPoint(30, 34, 0).addPoint(40, 29, 0).build());
                        paths.add(new Trajectory(40, 29, 0.6, 10).addPoint(31, 32, 180).build());
                        paths.add(new Trajectory(31, 32, 0.4, 13).addPoint(65, 32, 0).addPoint(65, 85, 0).addPoint(44, 96, 0).addPoint(44, 108, 0).build());
                        paths.add(new Trajectory(44, 108, 0.4, 10).addPoint(52, 108, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(52, 115, 0.4, 8, 0).addPoint(52, 104, 180).addPointSpeed(10, 104, 90, 0.3).build());
                                paths.add(new Trajectory(10, 104, 0.3, 20).addPoint(10, 123, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(52, 115, 0.4, 8, 0).addPoint(52, 104, 180).addPoint(60, 104, -90).build());
                                paths.add(new Trajectory(60, 104, 0.3, 20).addPoint(60, 126, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case BLUE_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(9, 84, 0.25, 12, 0.2).addPoint(32, 84, 0).addPoint(38, 89, 0).build());
                        paths.add(new Trajectory(38, 89, 0.6, 5).addPoint(25, 89, 180).build());
                        paths.add(new Trajectory(25, 89, 0.4, 13).addPoint(35, 89, 0).addPoint(38, 102, 0).addPoint(38, 112, 0).build());
                        paths.add(new Trajectory(38, 112, 0.4, 10).addPoint(35, 112, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(35, 115, 0.4, 8, 0).addPoint(35, 104, 180).addPointSpeed(16, 104, 90, 0.3).build());
                                paths.add(new Trajectory(16, 104, 0.3, 20).addPoint(16, 126, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(35, 115, 0.4, 8, 0).addPoint(35, 104, 180).addPoint(66, 104, -90).build());
                                paths.add(new Trajectory(66, 104, 0.3, 20).addPoint(66, 126, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(9, 84, 0.25, 12, 0.2).addPoint(37, 84, 0).build());
                        paths.add(new Trajectory(37, 84, 0.6, 5).addPoint(25, 84, 180).build());
                        paths.add(new Trajectory(25, 84, 0.4, 13).addPoint(40, 88, 0).addPoint(43, 96, 0).addPoint(43, 106, 0).build());
                        paths.add(new Trajectory(43, 106, 0.4, 10).addPoint(43, 112, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(43, 115, 0.4, 8, 0).addPoint(43, 104, 180).addPointSpeed(16, 104, 90, 0.3).build());
                                paths.add(new Trajectory(16, 104, 0.3, 20).addPoint(16, 126, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(43, 115, 0.4, 8, 0).addPoint(43, 104, 180).addPoint(66, 104, -90).build());
                                paths.add(new Trajectory(66, 104, 0.3, 20).addPoint(66, 126, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(9, 84, 0.25, 12, 0.2).addPoint(32, 84, 0).addPoint(38, 80, 0).build());
                        paths.add(new Trajectory(38, 80, 0.6, 5).addPoint(25, 88, 180).build());
                        paths.add(new Trajectory(25, 88, 0.4, 13).addPoint(40, 88, 0).addPoint(43, 96, 0).addPoint(43, 112, 0).build());
                        paths.add(new Trajectory(43, 112, 0.4, 10).addPoint(50, 112, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(50, 115, 0.4, 8, 0).addPoint(50, 104, 180).addPointSpeed(16, 104, 90, 0.3).build());
                                paths.add(new Trajectory(16, 104, 0.3, 20).addPoint(16, 126, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(50, 115, 0.4, 8, 0).addPoint(50, 104, 180).addPoint(66, 104, -90).build());
                                paths.add(new Trajectory(66, 104, 0.3, 20).addPoint(66, 126, 0).build());
                                break;
                        }
                        break;
                }
                break;

        }






        /*switch (spikePosition) {

            case LEFT:
                switch (allianceLocation) {
                    case RED_SOUTH:
                        paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 20, 0).addPoint(-6, 29, 0).build());
                        paths.add(new Trajectory(-5, 29, 0.4, 5).addPoint(0, 4, 180).build());
                        paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(53, 4, -90).addPoint(60, 43, 0).addPoint(72, 43, 0).build());
                        switch (parkLocation) {

                            case LEFT:
                                paths.add(new Trajectory(72, 43, 0.4, 10).addPoint(65, 43, 180).addPoint(65, 59, 90).build());
                                paths.add(new Trajectory(65, 59, 0.3, 20).addPoint(87, 59, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(70, 44, 0.4, 10).addPoint(65, 44, 180).addPoint(65, 8, -90).build());
                                paths.add(new Trajectory(65, 8, 0.3, 20).addPoint(87, 8, 0).build());
                                break;
                        }
                        break;
                    case RED_NORTH:
                        paths.add(new Trajectory(0, 0,0.25, 12, 0.2).addPoint(0, 23, 0).addPoint(-3, 28, 0).build());
                        paths.add(new Trajectory(-3, 27, 0.6, 5).addPoint(0, 18, 180).build());
                        paths.add(new Trajectory(0, 18, 0.4, 13).addPoint(0, 30, 0).addPoint(12, 34, 0).addPoint(28, 34, 0).build());
                        paths.add(new Trajectory(28, 34, 0.4, 10).addPointSpeed(28, 43, 90, 0.4).build());
                        switch (parkLocation) {

                            case LEFT:
                                paths.add(new Trajectory(31, 43, 0.4, 8, 0).addPoint(20, 43, 180).addPointSpeed(20, 59, 90, 0.3).build());
                                paths.add(new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(31, 41, 0.4, 10).addPoint(20, 44, 180).addPoint(20, 8, -90).build());
                                paths.add(new Trajectory(20, 8, 0.3, 20).addPoint(42, 8, 0).build());
                                break;
                        }
                        break;
                    case BLUE_SOUTH:
                        paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 23, 0).addPoint(-4, 28, 0).build());
                        paths.add(new Trajectory(0, 28, 0.3, 5).addPoint(0, 4, 180).build());
                        paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(-55, 4, 90).addPoint(-62, 21, 0).addPoint(-72, 21, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(-72, 21, 0.4, 10).addPoint(-65, 21, 180).addPoint(-65, 5, 90).build());
                                paths.add(new Trajectory(-65, 5, 0.3, 20).addPoint(-87, 5, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(-72, 27, 0.4, 10).addPoint(-65, 27, 180).addPoint(-65, 56, -90).build());
                                paths.add(new Trajectory(-65, 56, 0.3, 20).addPoint(-87, 56, 0).build());
                                break;
                        }
                        break;
                    case BLUE_NORTH:
                        paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 20, 0).addPoint(-6, 29, 0).build());
                        paths.add(new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build());
                        paths.add(new Trajectory(0, 22, 0.4, 15).addPoint(-12, 22, 90).addPoint(-12, 27, 0).addPoint(-25, 27, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(-25, 27, 0.4, 10).addPoint(-20, 27, 180).addPoint(-20, 8, 90).build());
                                paths.add(new Trajectory(-20, 8, 0.3, 20).addPoint(-42, 8, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(-25, 27, 0.4, 10).addPoint(-20, 27, 180).addPoint(-20, 56, -90).build());
                                paths.add(new Trajectory(-20, 56, 0.3, 20).addPoint(-42, 56, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case CENTER:
                switch (allianceLocation) {
                    case RED_SOUTH:
                        paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                        paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 4, 180).build());
                        paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(55, 4, -90).addPoint(60, 32, 0).addPoint(72, 32, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(72, 34, 0.4, 10).addPoint(65, 34, 180).addPoint(65, 59, 90).build());
                                paths.add(new Trajectory(65, 59, 0.3, 10).addPoint(87, 59, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(72, 34, 0.4, 10).addPoint(65, 34, 180).addPoint(65, 8, -90).build());
                                paths.add(new Trajectory(65, 8, 0.3, 10).addPoint(87, 8, 0).build());
                                break;
                        }
                        break;
                    case RED_NORTH:
                        paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                        paths.add(new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build());
                        paths.add(new Trajectory(0, 10, 0.4, 15).addPoint(0, 34, 0).addPoint(25, 34, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(25, 34, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 59, 90).build());
                                paths.add(new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(25, 34, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 8, -90).build());
                                paths.add(new Trajectory(20, 8, 0.3, 20).addPoint(42, 8, 0).build());
                                break;
                        }
                        break;
                    case BLUE_SOUTH:
                        paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                        paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 4, 180).build());
                        paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(-55, 4, 90).addPoint(-63, 30, 0).addPoint(-72, 30, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(-72, 30, 0.4, 10).addPoint(-65, 30, 180).addPoint(-65, 5, 90).build());
                                paths.add(new Trajectory(-65, 5, 0.3, 10).addPoint(-87, 5, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(-72, 34, 0.4, 10).addPoint(-65, 34, 180).addPoint(-65, 56, -90).build());
                                paths.add(new Trajectory(-65, 56, 0.3, 10).addPoint(-87, 56, 0).build());
                                break;
                        }
                        break;
                    case BLUE_NORTH:
                        paths.add(new Trajectory(0, 0,0.5, 15).addPoint(0, 27, 0).build());
                        paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 22, 180).build());
                        paths.add(new Trajectory(0, 10, 0.4, 15).addPoint(0, 34, 0).addPoint(-25, 34, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(-25, 34, 0.4, 10).addPoint(-20, 34, -180).addPoint(-20, 5, 90).build());
                                paths.add(new Trajectory(-20, 5, 0.3, 20).addPoint(-42, 5, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(-25, 34, 0.4, 10).addPoint(-20, 34, 180).addPoint(-20, 56, -90).build());
                                paths.add(new Trajectory(-20, 56, 0.3, 20).addPoint(-42, 56, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case RIGHT:
                switch (allianceLocation) {
                    case RED_SOUTH:
                        paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 23, 0).addPoint(4, 28, 0).build());
                        paths.add(new Trajectory(0, 27, 0.3, 5).addPoint(0, 4, 180).build());
                        paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(53, 4, -90).addPoint(60, 27, 0).addPoint(72, 27, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(72, 30, 0.4, 20).addPoint(65, 34, -180).addPoint(65, 59, 90).build());
                                paths.add(new Trajectory(65, 59, 0.3, 20).addPoint(87, 59, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(72, 30, 0.4, 10).addPoint(65, 34, 180).addPoint(65, 8, -90).build());
                                paths.add(new Trajectory(65, 8, 0.3, 20).addPoint(87, 8, 0).build());
                                break;
                        }
                        break;
                    case RED_NORTH:
                        paths.add(new Trajectory(0, 0,0.3, 10).addPoint(0, 10, 0).addPoint(6, 28, 0).build());
                        paths.add(new Trajectory(0, 27, 0.6, 5).addPoint(0, 22, 180).build());
                        paths.add(new Trajectory(0, 22, 0.4, 15).addPoint(12, 22, -90).addPoint(12, 30, 0).addPoint(25, 30, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(25, 30, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 59, 90).build());
                                paths.add(new Trajectory(20, 59, 0.3, 20).addPoint(42, 59, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(25, 30, 0.4, 10).addPoint(20, 34, 180).addPoint(20, 8, -90).build());
                                paths.add(new Trajectory(20, 8, 0.3, 20).addPoint(42, 8, 0).build());
                                break;
                        }
                        break;
                    case BLUE_SOUTH:
                        paths.add(new Trajectory(0, 0,0.3, 10, 0.3).addPoint(0, 20, 0).addPoint(6, 29, 0).build());
                        paths.add(new Trajectory(5, 29, 0.4, 5).addPoint(0, 4, 180).build());
                        paths.add(new Trajectory(0, 4, 0.4, 15).addPoint(-55, 4, 90).addPoint(-63, 38, 0).addPoint(-72, 38, 0).build());
                        switch (parkLocation) {

                            case LEFT:
                                paths.add(new Trajectory(-70, 38, 0.4, 10).addPoint(-65, 38, -180).addPoint(-65, 5, 90).build());
                                paths.add(new Trajectory(-65, 5, 0.3, 20).addPoint(-87, 5, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(-72, 38, 0.4, 10).addPoint(-65, 38, 180).addPoint(-65, 56, -90).build());
                                paths.add(new Trajectory(-65, 56, 0.3, 20).addPoint(-87, 56, 0).build());
                                break;
                        }
                        break;
                    case BLUE_NORTH:
                        paths.add(new Trajectory(0, 0,0.3, 10).addPoint(0, 20, 0).addPoint(5, 28, 0).build());
                        paths.add(new Trajectory(5, 28, 0.3, 5).addPoint(0, 22, 180).build());
                        paths.add(new Trajectory(0, 12, 0.4, 15).addPoint(0, 30, 0).addPoint(-12, 41, 0).addPoint(-25, 41, 0).build());
                        switch (parkLocation) {

                            case LEFT:
                                paths.add(new Trajectory(-25, 44, 0.4, 10).addPoint(-20, 44, 180).addPoint(-20, 8, 90).build());
                                paths.add(new Trajectory(-20, 8, 0.3, 20).addPoint(-42, 8, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(-25, 44, 0.4, 10).addPoint(-20, 44, 180).addPoint(-20, 56, -90).build());
                                paths.add(new Trajectory(-20, 56, 0.3, 20).addPoint(-42, 56, 0).build());
                                break;
                        }
                        break;
                }
                break;

        }

         */
        return paths;
    }

}
