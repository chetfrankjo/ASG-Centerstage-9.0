package org.firstinspires.ftc.teamcode.DataTypes;

public class General {
    public enum SpikePosition {
        LEFT, CENTER, RIGHT, NONE
    }

    public enum CameraMode {
        PROP, APRILTAG, APLS, IDLE
    }

    public enum LocalMode {
        ODOMETRY, APRILTAG, FUSED
    }

    public enum IntakeMode {
        INTAKE, OUTTAKE, LOCK, MANUAL
    }

    public enum PlungerMode {
        LOAD, PRIME, DEPOSIT, MANUAL
    }


    public enum ClawMode {
        OPEN, LEFT, RIGHT, BOTH//, PRIMED, IDLE, INTAKING
    }

    public enum WeaponsState {
        INTAKING, HOLDING, PRIMED, EXTEND, DEPOSIT, IDLE, MANUAL
    }

    public enum FlipperState {
        STORED, READY, IDLE
    }

    public enum PixelState {
        NONE, LEFT, RIGHT, BOTH
    }

    public enum AutoState {
        VISION, PURPLE_APPROACH, SPIKE, BACKUP, APPROACH_2, APPROACH_3, CYCLE_INTAKE, CYCLE_APPROACH, CYCLE_APPROACH_2, PARK_1, PARK2
    }

    public enum AllianceLocation {
        RED_SOUTH, RED_NORTH, BLUE_SOUTH, BLUE_NORTH, NONE
    }

    public enum ParkLocation {
        LEFT, RIGHT, CENTER, NONE
    }

    public enum AutoMode {
        STANDARD, CYCLE_1, CYCLE_2, PARK
    }


}
