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
                /*switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(135, 34, 0.25, 12, 0.2).addPoint(117, 34, 0).addPoint(110, 29, 0).build());
                        paths.add(new Trajectory(110, 29, 0.6, 10).addPoint(115, 33, 180).build());
                        paths.add(new Trajectory(115, 33, 0.4, 13).addPoint(77, 33, 0).addPoint(77, 85, 0).addPoint(101, 96, 0).addPoint(101, 108, 0).build());
                        paths.add(new Trajectory(101, 108, 0.4, 10).addPoint(96, 108, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(95, 115, 0.4, 8, 0).addPoint(95, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
                                paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(95, 115, 0.4, 8, 0).addPoint(95, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(135, 34, 0.25, 12, 0.2).addPoint(107, 35, 0).build());
                        paths.add(new Trajectory(107, 34, 0.6, 10).addPoint(116, 22, 180).addPoint(125, 22, 180).build());
                        paths.add(new Trajectory(120, 22, 0.4, 13).addPoint(77, 22, 0).addPoint(77, 90, 0).addPoint(103, 102, 0).addPoint(103, 110, 0).build());
                        paths.add(new Trajectory(103, 110, 0.4, 10).addPoint(103, 110, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(103, 115, 0.4, 8, 0).addPoint(103, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
                                paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(103, 115, 0.4, 8, 0).addPoint(103, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(135, 34, 0.25, 12, 0.2).addPoint(113, 34, 0).addPoint(107, 38, 0).build());
                        paths.add(new Trajectory(107, 38, 0.6, 10).addPoint(115, 32, 180).build());
                        paths.add(new Trajectory(115, 32, 0.4, 13).addPoint(77, 32, 0).addPoint(77, 101, 0).addPoint(101, 106, 0).addPoint(101, 112, 0).build());
                        paths.add(new Trajectory(101, 112, 0.4, 8).addPoint(107, 112, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(105, 115, 0.4, 8, 0).addPoint(105, 104, 180).addPointSpeed(73, 104, 90, 0.3).build());
                                paths.add(new Trajectory(73, 104, 0.3, 20).addPoint(73, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(105, 115, 0.4, 8, 0).addPoint(105, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                 */
                        //break;
                //}
                break;
            case RED_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(135, 84, 0.4, 7, 0.2).addPoint(104, 83, 90).build());
                        paths.add(new Trajectory(104, 81, 0.6, 5).addPoint(104, 90, 0).build());
                        paths.add(new Trajectory(104, 80, 0.4, 13).addPoint(104, 112, 0).build());
                        paths.add(new Trajectory(104, 112, 0.4, 10).addPoint(99, 112, 90).addPoint(99, 118, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(93, 115, 0.4, 8, 0).addPoint(93, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
                                paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(93, 115, 0.4, 8, 0).addPoint(93, 104, 180).addPoint(130, 104, -90).build());
                                paths.add(new Trajectory(130, 104, 0.3, 20).addPoint(130, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(135, 84, 0.5, 8).addPoint(97, 84, 90).addPoint(97, 91, 0).build());
                        paths.add(new Trajectory(97, 88, 0.6, 5).addPoint(97, 95, 0).build());
                        paths.add(new Trajectory(97, 95, 0.4, 10).addPoint(106, 95, -90).addPoint(106, 110, 0).build());
                        paths.add(new Trajectory(106, 110, 0.4, 8).addPoint(106, 116, 0).build()); // dud point
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(106, 109, 0.4, 8, 0).addPoint(106, 104, 180).addPointSpeed(82, 104, 90, 0.3).build());
                                paths.add(new Trajectory(82, 104, 0.3, 20).addPoint(82, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(106, 109, 0.4, 8, 0).addPoint(106, 104, 180).addPoint(130, 104, -90).build());
                                paths.add(new Trajectory(130, 104, 0.3, 20).addPoint(130, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(135, 84, 0.6, 12, 0.2).addPoint(102, 84, 90).addPoint(102, 105, 0).build());
                        paths.add(new Trajectory(102, 105, 0.6, 5).addPoint(102, 115, 0).build());
                        paths.add(new Trajectory(102, 115, 0.4, 7).addPoint(114, 115, -90).build());
                        paths.add(new Trajectory(114, 115, 0.4, 5).addPoint(114, 120, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 110, 180).addPointSpeed(83, 110, 90, 0.3).build());
                                paths.add(new Trajectory(83, 110, 0.3, 5).addPoint(83, 116, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPoint(130, 104, -90).build());
                                paths.add(new Trajectory(130, 104, 0.3, 20).addPoint(130, 115, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case BLUE_SOUTH:
                /*switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(9, 34, 0.25, 12, 0.2).addPoint(30, 34, 0).addPoint(40, 39, 0).build());
                        paths.add(new Trajectory(40, 39, 0.6, 10).addPoint(32, 34, 180).build());
                        paths.add(new Trajectory(32, 34, 0.4, 13).addPoint(65, 34, 0).addPoint(65, 100, 0).addPoint(44, 106, 0).addPoint(44, 112, 0).build());
                        paths.add(new Trajectory(44, 112, 0.4, 8).addPoint(38, 112, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(39, 115, 0.4, 8, 0).addPoint(39, 104, 180).addPointSpeed(22, 104, 90, 0.3).build());
                                paths.add(new Trajectory(22, 104, 0.3, 20).addPoint(22, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(39, 115, 0.4, 8, 0).addPoint(39, 104, 180).addPoint(68, 104, -90).build());
                                paths.add(new Trajectory(68, 104, 0.3, 20).addPoint(68, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(9, 34, 0.25, 12, 0.2).addPoint(37, 34, 0).build());
                        paths.add(new Trajectory(37, 34, 0.6, 10, 0.3).addPoint(27, 34, 180).addPoint(27, 22, -90).addPoint(32, 22, 0).build());
                        paths.add(new Trajectory(32, 22, 0.5, 13).addPoint(68, 22, 0).addPoint(68, 100, 0).addPoint(44, 105, 0).addPoint(44, 115, 0).build());
                        paths.add(new Trajectory(44, 115, 0.4, 10).addPoint(45, 115, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(44, 115, 0.4, 8, 0).addPoint(44, 104, 180).addPointSpeed(22, 104, 90, 0.3).build());
                                paths.add(new Trajectory(22, 104, 0.3, 20).addPoint(22, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(44, 115, 0.4, 8, 0).addPoint(44, 104, 180).addPoint(68, 104, -90).build());
                                paths.add(new Trajectory(68, 104, 0.3, 20).addPoint(68, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(9, 34, 0.25, 12, 0.2).addPoint(25, 34, 0).addPoint(34, 29, 0).build());
                        paths.add(new Trajectory(40, 28, 0.6, 10).addPoint(29, 34, 180).build());
                        paths.add(new Trajectory(29, 34, 0.4, 13).addPoint(65, 34, 0).addPoint(65, 98, 0).addPoint(44, 102, 0).addPoint(44, 110, 0).build());
                        paths.add(new Trajectory(44, 110, 0.4, 10).addPoint(49, 110, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(49, 115, 0.4, 8, 0).addPoint(49, 104, 180).addPointSpeed(22, 104, 90, 0.3).build());
                                paths.add(new Trajectory(22, 104, 0.3, 20).addPoint(22, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(49, 115, 0.4, 8, 0).addPoint(49, 104, 180).addPoint(68, 104, -90).build());
                                paths.add(new Trajectory(68, 104, 0.3, 20).addPoint(68, 115, 0).build());
                                break;
                        }
                        break;
                }

                 */
                break;
            case BLUE_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(9, 84, 0.25, 12, 0.2).addPoint(36, 84, -90).addPoint(36, 110, 0).build());
                        paths.add(new Trajectory(36, 110, 0.6, 5).addPoint(36, 115, 0).build());
                        paths.add(new Trajectory(36, 115, 0.4, 13).addPoint(25, 115, 90).addPoint(25, 118, 0).build());
                        paths.add(new Trajectory(25, 118, 0.4, 10).addPoint(25, 122, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(35, 115, 0.4, 8, 0).addPoint(35, 104, 180).addPointSpeed(18, 104, 90, 0.3).build());
                                paths.add(new Trajectory(18, 104, 0.3, 20).addPoint(18, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(35, 115, 0.4, 8, 0).addPoint(35, 104, 180).addPoint(66, 104, -90).build());
                                paths.add(new Trajectory(66, 104, 0.3, 20).addPoint(66, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(9, 84, 0.25, 12, 0.2).addPoint(47, 84, -90).build());
                        paths.add(new Trajectory(47, 84, 0.6, 5).addPoint(47, 88, 0).build());
                        paths.add(new Trajectory(47, 88, 0.4, 13).addPoint(34, 88, 90).addPoint(34, 96, 0).build());
                        paths.add(new Trajectory(34, 96, 0.4, 10).addPoint(34, 112, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(43, 115, 0.4, 8, 0).addPoint(43, 104, 180).addPointSpeed(18, 104, 90, 0.3).build());
                                paths.add(new Trajectory(18, 104, 0.3, 20).addPoint(18, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(43, 115, 0.4, 8, 0).addPoint(43, 104, 180).addPoint(66, 104, -90).build());
                                paths.add(new Trajectory(66, 104, 0.3, 20).addPoint(66, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(9, 84, 0.25, 12, 0.2).addPoint(37, 84, 90).addPoint(37, 88, 0).build());
                        paths.add(new Trajectory(37, 88, 0.6, 5).addPoint(37, 95, 180).build());
                        paths.add(new Trajectory(37, 95, 0.4, 13).addPoint(43, 95, -90).addPoint(43, 100, 0).build());
                        paths.add(new Trajectory(43, 100, 0.4, 10).addPoint(43, 112, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(50, 115, 0.4, 8, 0).addPoint(50, 104, 180).addPointSpeed(18, 104, 90, 0.3).build());
                                paths.add(new Trajectory(18, 104, 0.3, 20).addPoint(18, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(50, 115, 0.4, 8, 0).addPoint(50, 104, 180).addPoint(66, 104, -90).build());
                                paths.add(new Trajectory(66, 104, 0.3, 20).addPoint(66, 115, 0).build());
                                break;
                        }
                        break;
                }
                break;

        }
        return paths;
    }






    public static ArrayList<Trajectory> generateCycleAutoPaths(General.ParkLocation parkLocation, General.SpikePosition spikePosition, General.AllianceLocation allianceLocation) {
        ArrayList<Trajectory> paths = new ArrayList<>();
        switch (allianceLocation) {
            case RED_SOUTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(135, 34, 0.7, 7, 0.5).addPoint(129, 34, 90).addPoint(110, 15, 135).build());
                        paths.add(new Trajectory(110, 15, 0.3, 10).addPoint(110, 13,145).addPoint(84, 15, 90).build());
                        paths.add(new Trajectory(84, 13, 0.5, 12).addPoint(80, 13, 0).addPoint(83, 13, -90).addPoint(83, 108, 0).addPoint(99, 108, -90).addPoint(99, 116, 0).build());
                        paths.add(new Trajectory(99, 118, 0.5, 6).addPoint(101, 118, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPointSpeed(78, 104, 90, 0.3).build());
                                paths.add(new Trajectory(78, 104, 0.3, 20).addPoint(78, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(135, 34, 0.6, 12, 0.2).addPoint(99, 30, 90+6.52+2).build());
                        paths.add(new Trajectory(100, 30, 0.25, 18).addPoint(107, 24, -90-40.6).addPoint(107, 16, 180).build());
                        paths.add(new Trajectory(107, 13, 0.5, 14).addPoint(107, 18, 0).addPoint(83, 18, 90).addPoint(83, 108, 0).addPoint(106, 108, -90).addPoint(106, 116, 0).build());
                        paths.add(new Trajectory(106, 120, 0.4, 6).addPoint(106, 123, 0).build());
                        //paths.add(new Trajectory(100, 123, 0.5, 12).addPoint(100, 100, 180).addPoint(80, 100, 90).addPoint(80, 50, 180).addPoint(85, 14, 180).build()); // cycle intake/approach
                        //paths.add(new Trajectory(85, 20, 0.6, 12).addPoint(73, 40, 0).addPoint(73, 95, 0).addPoint(101, 95, -90).addPoint(101, 108, 0).build()); // cycle backdrop apprach
                        //paths.add(new Trajectory(101, 108, 0.4, 12).addPoint(96, 108, 90).build()); //cycle backdrop go to perfect position
                        //paths.add(new Trajectory(95, 115, 0.4, 8, 0).addPoint(95, 104, 180).addPointSpeed(76, 104, 90, 0.3).build());
                        //paths.add(new Trajectory(76, 104, 0.3, 20).addPoint(76, 115, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(102, 115, 0.4, 8, 0).addPoint(102, 104, 180).addPointSpeed(82, 104, 90, 0.3).build());
                                paths.add(new Trajectory(82, 104, 0.3, 20).addPoint(82, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(102, 115, 0.4, 8, 0).addPoint(102, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(135, 34, 0.5, 11, 0.2).addPoint(106, 34, 90).addPoint(106, 38.5, 0).build());
                        paths.add(new Trajectory(105, 38, 0.25, 12).addPoint(108, 25, 180).addPoint(108, 16, 180).build());
                        paths.add(new Trajectory(108, 13, 0.5, 12).addPoint(108, 18, 0).addPoint(83, 18, 90).addPoint(83, 108, 0).addPoint(105, 108, -90).addPoint(105, 116, 0).build());
                        paths.add(new Trajectory(105, 118, 0.4, 8).addPoint(112, 118, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPointSpeed(78, 104, 90, 0.3).build());
                                paths.add(new Trajectory(78, 104, 0.3, 20).addPoint(78, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case RED_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
            case BLUE_SOUTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(9, 34, 0.5, 11, 0.2).addPoint(38, 34, -90).addPoint(38, 38.5, 0).build());
                        paths.add(new Trajectory(38, 38, 0.25, 12).addPoint(38, 25, 180).addPoint(38, 16, 180).build());
                        paths.add(new Trajectory(38, 13, 0.5, 12).addPoint(38, 18, 0).addPoint(63, 18, -90).addPoint(63, 108, 0).addPoint(41, 108, 90).addPoint(41, 116, 0).build());
                        paths.add(new Trajectory(41, 118, 0.4, 8).addPoint(27, 118, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(29, 115, 0.4, 8, 0).addPoint(29, 104, 180).addPointSpeed(5, 104, 90, 0.3).build());
                                paths.add(new Trajectory(5, 104, 0.3, 20).addPoint(5, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(29, 115, 0.4, 8, 0).addPoint(29, 104, 180).addPoint(55, 104, -90).build());
                                paths.add(new Trajectory(55, 104, 0.3, 20).addPoint(55, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(9, 34, 0.6, 12, 0.2).addPoint(48, 30, -90-6.52-2).build());
                        paths.add(new Trajectory(48, 30, 0.25, 18).addPoint(60, 24, +90+40.6).addPoint(60, 16, 180).build());
                        paths.add(new Trajectory(60, 13, 0.5, 14).addPoint(60, 18, 0).addPoint(62, 18, -90).addPoint(62, 108, 0).addPoint(30, 108, 90).addPoint(30, 116, 0).build());
                        paths.add(new Trajectory(35, 120, 0.4, 6).addPoint(35, 123, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(30, 115, 0.4, 8, 0).addPoint(30, 104, 180).addPointSpeed(5, 104, 90, 0.3).build());
                                paths.add(new Trajectory(5, 104, 0.3, 20).addPoint(5, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(30, 115, 0.4, 8, 0).addPoint(30, 104, 180).addPoint(55, 104, -90).build());
                                paths.add(new Trajectory(55, 104, 0.3, 20).addPoint(55, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(9, 34, 0.4, 16, 0).addPoint(20, 34, -90).addPoint(20, 16, 180).addPoint(37, 16, -90).build());
                        paths.add(new Trajectory(37, 16, 0.3, 8).addPoint(37, 11,180).addPoint(50, 11, -90).addPoint(50, 15, 0).addPoint(60, 15, -90).build());
                        paths.add(new Trajectory(60, 13, 0.5, 14).addPoint(62, 13, 0).addPoint(62, 13, 90).addPoint(62, 106, 0).addPoint(59, 106, 90).addPoint(59, 116, 0).build());
                        paths.add(new Trajectory(59, 118, 0.5, 6).addPoint(36.5, 118, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(35, 115, 0.4, 8, 0).addPoint(35, 104, 180).addPointSpeed(5, 104, 90, 0.3).build());
                                paths.add(new Trajectory(5, 104, 0.3, 20).addPoint(5, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(40, 115, 0.4, 8, 0).addPoint(40, 104, 180).addPoint(55, 104, -90).build());
                                paths.add(new Trajectory(55, 104, 0.3, 20).addPoint(55, 115, 0).build());
                                break;
                        }

                        break;
                }
                break;
            case BLUE_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
        }
        return paths;
    }







    public static ArrayList<Trajectory> generateCycleAutoMorePaths(General.ParkLocation parkLocation, General.SpikePosition spikePosition, General.AllianceLocation allianceLocation) {
        ArrayList<Trajectory> paths = new ArrayList<>();
        switch (allianceLocation) {
            case RED_SOUTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(135, 34, 0.7, 7, 0.5).addPoint(129, 34, 90).addPoint(110, 15, 135).build());
                        paths.add(new Trajectory(110, 15, 0.3, 10).addPoint(110, 13,145).addPoint(84, 15, 90).build());
                        paths.add(new Trajectory(84, 13, 0.7, 12).addPoint(80, 13, 0).addPoint(83, 13, -90).addPoint(83, 108, 0).addPoint(99, 108, -90).addPoint(99, 116, 0).build());
                        paths.add(new Trajectory(99, 118, 0.5, 6).addPoint(101, 118, -90).build());
                        paths.add(new Trajectory(101, 118, 0.7, 14).addPoint(101,108, 180).addPoint(83, 108, 90).addPoint(83, 13, 180).build());
                        paths.add(new Trajectory(84, 13, 0.7, 12).addPoint(80, 13, 0).addPoint(83, 13, -90).addPoint(83, 108, 0).addPoint(106, 108, -90).addPoint(106, 116, 0).build());
                        paths.add(new Trajectory(106, 118, 0.5, 6).addPoint(108, 118, -90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPointSpeed(78, 104, 90, 0.3).build());
                                paths.add(new Trajectory(78, 104, 0.3, 20).addPoint(78, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(135, 34, 0.6, 12, 0.2).addPoint(99, 30, 90+6.52+2).build());
                        paths.add(new Trajectory(100, 30, 0.25, 18).addPoint(107, 24, -90-40.6).addPoint(107, 16, 180).build());
                        paths.add(new Trajectory(107, 13, 0.7, 14).addPoint(107, 18, 0).addPoint(83, 18, 90).addPoint(83, 108, 0).addPoint(106, 108, -90).addPoint(106, 116, 0).build());
                        paths.add(new Trajectory(106, 120, 0.4, 6).addPoint(106, 123, 0).build());
                        paths.add(new Trajectory(106, 123, 0.7, 12).addPoint(106, 100, 180).addPoint(83, 100, 90).addPoint(83, 15, 180).build()); // cycle intake/approach
                        paths.add(new Trajectory(83, 13, 0.7, 14).addPoint(83, 18, 0).addPoint(83, 108, 0).addPoint(101, 108, -90).addPoint(101, 116, 0).build());
                        paths.add(new Trajectory(101, 116, 0.4, 12).addPoint(101, 116, 0).build()); //cycle backdrop go to perfect position
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(102, 115, 0.4, 8, 0).addPoint(102, 104, 180).addPointSpeed(82, 104, 90, 0.3).build());
                                paths.add(new Trajectory(82, 104, 0.3, 20).addPoint(82, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(102, 115, 0.4, 8, 0).addPoint(102, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(135, 34, 0.5, 11, 0.2).addPoint(106, 34, 90).addPoint(106, 38.5, 0).build());
                        paths.add(new Trajectory(105, 38, 0.25, 12).addPoint(108, 25, 180).addPoint(108, 16, 180).build());
                        paths.add(new Trajectory(108, 13, 0.5, 12).addPoint(108, 18, 0).addPoint(83, 18, 90).addPoint(83, 108, 0).addPoint(105, 108, -90).addPoint(105, 116, 0).build());
                        paths.add(new Trajectory(105, 118, 0.4, 8).addPoint(112, 118, -90).build());
                        paths.add(new Trajectory(112, 118, 0.7, 14).addPoint(112, 100, 180).addPoint(83, 100, 90).addPoint(83, 15, 180).build());
                        paths.add(new Trajectory(83, 8, 0.7, 14).addPoint(83, 100, 0).addPoint(105, 100, -90).addPoint(105, 110, 0).build());
                        paths.add(new Trajectory(105, 110, 0.4, 6).addPoint(105, 113, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPointSpeed(78, 104, 90, 0.3).build());
                                paths.add(new Trajectory(78, 104, 0.3, 20).addPoint(78, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(110, 115, 0.4, 8, 0).addPoint(110, 104, 180).addPoint(125, 104, -90).build());
                                paths.add(new Trajectory(125, 104, 0.3, 20).addPoint(125, 115, 0).build());
                                break;
                        }
                        break;
                }
                break;
            case RED_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
            case BLUE_SOUTH:
                switch (spikePosition) {
                    case LEFT:
                        paths.add(new Trajectory(9, 34, 0.5, 11, 0.2).addPoint(38, 34, -90).addPoint(38, 38.5, 0).build());
                        paths.add(new Trajectory(38, 38, 0.25, 12).addPoint(38, 25, 180).addPoint(38, 16, 180).build());
                        paths.add(new Trajectory(38, 13, 0.5, 12).addPoint(38, 18, 0).addPoint(63, 18, -90).addPoint(63, 108, 0).addPoint(41, 108, 90).addPoint(41, 116, 0).build());
                        paths.add(new Trajectory(41, 118, 0.4, 8).addPoint(27, 118, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(29, 115, 0.4, 8, 0).addPoint(29, 104, 180).addPointSpeed(5, 104, 90, 0.3).build());
                                paths.add(new Trajectory(5, 104, 0.3, 20).addPoint(5, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(29, 115, 0.4, 8, 0).addPoint(29, 104, 180).addPoint(55, 104, -90).build());
                                paths.add(new Trajectory(55, 104, 0.3, 20).addPoint(55, 115, 0).build());
                                break;
                        }
                        break;
                    case CENTER:
                        paths.add(new Trajectory(9, 34, 0.6, 12, 0.2).addPoint(48, 30, -90-6.52-2).build());
                        paths.add(new Trajectory(48, 30, 0.25, 18).addPoint(60, 24, +90+40.6).addPoint(60, 16, 180).build());
                        paths.add(new Trajectory(60, 13, 0.5, 14).addPoint(60, 18, 0).addPoint(62, 18, -90).addPoint(62, 108, 0).addPoint(30, 108, 90).addPoint(30, 116, 0).build());
                        paths.add(new Trajectory(35, 120, 0.4, 6).addPoint(35, 123, 0).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(30, 115, 0.4, 8, 0).addPoint(30, 104, 180).addPointSpeed(5, 104, 90, 0.3).build());
                                paths.add(new Trajectory(5, 104, 0.3, 20).addPoint(5, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(30, 115, 0.4, 8, 0).addPoint(30, 104, 180).addPoint(55, 104, -90).build());
                                paths.add(new Trajectory(55, 104, 0.3, 20).addPoint(55, 115, 0).build());
                                break;
                        }
                        break;
                    case RIGHT:
                        paths.add(new Trajectory(9, 34, 0.4, 16, 0).addPoint(20, 34, -90).addPoint(20, 16, 180).addPoint(37, 16, -90).build());
                        paths.add(new Trajectory(37, 16, 0.3, 8).addPoint(37, 11,180).addPoint(50, 11, -90).addPoint(50, 15, 0).addPoint(60, 15, -90).build());
                        paths.add(new Trajectory(60, 13, 0.5, 14).addPoint(62, 13, 0).addPoint(62, 13, 90).addPoint(62, 106, 0).addPoint(59, 106, 90).addPoint(59, 116, 0).build());
                        paths.add(new Trajectory(59, 118, 0.5, 6).addPoint(36.5, 118, 90).build());
                        switch (parkLocation) {
                            case LEFT:
                                paths.add(new Trajectory(35, 115, 0.4, 8, 0).addPoint(35, 104, 180).addPointSpeed(5, 104, 90, 0.3).build());
                                paths.add(new Trajectory(5, 104, 0.3, 20).addPoint(5, 115, 0).build());
                                break;
                            case RIGHT:
                                paths.add(new Trajectory(40, 115, 0.4, 8, 0).addPoint(40, 104, 180).addPoint(55, 104, -90).build());
                                paths.add(new Trajectory(55, 104, 0.3, 20).addPoint(55, 115, 0).build());
                                break;
                        }

                        break;
                }
                break;
            case BLUE_NORTH:
                switch (spikePosition) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
        }
        return paths;
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


}