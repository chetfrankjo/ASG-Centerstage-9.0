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
        GRAB, RELEASE, IDLE
    }

    public enum WeaponsState {
        INTAKING, HOLDING, PRIMED, DEPOSIT, IDLE
    }
}
