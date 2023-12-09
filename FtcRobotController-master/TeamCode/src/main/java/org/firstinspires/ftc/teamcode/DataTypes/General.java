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
        GRAB_L, GRAB_R, GRAB_BOTH, RELEASE_L, RELEASE_R, RELEASE_BOTH, PRIMED, INTAKING, IDLE
    }

    public enum WeaponsState {
        INTAKING, HOLDING, PRIMED, EXTEND, DEPOSIT, IDLE
    }

    public enum AUTO_RED_NORTH_1 {
        VISION, APPROACH_1, SPIKE, BACKUP, APPROACH_2, APPROACH_3, PARK_1, PARK2
    }

    public enum AllianceLocation {
        RED_SOUTH, RED_NORTH, BLUE_SOUTH, BLUE_NORTH, NONE
    }

    public enum ParkLocation {
        LEFT, RIGHT, CENTER, NONE
    }


}
