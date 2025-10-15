package frc.utils;

public enum IntakeState {
    /** Can be passed as an argument to indicate no change in state. An intake should never be in this state directly. */
    NONE,
    /** The intake is stopped. */
    STOP,
    /** The intake is spinning to collect something. */
    INTAKE,
    /** The intake is spinning to eject something. */
    OUTTAKE,
    /** The intake is moving passively, i.e. to hold something in. */
    PASSIVE,
}
