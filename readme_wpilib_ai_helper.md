# WPILib Java API Cheat Sheet for AI Coding Assistants

> This file is intended to help an AI coding assistant understand what libraries are available in this project and how to use them when programming an FRC robot in Java with WPILib.

---

## 1. WPILib Java Version & Docs

This project uses **WPILib Java** for FRC robot code. The primary online API reference is:

- WPILib Java API (latest release): https://github.wpilib.org/allwpilib/docs/release/java/index.html
- General WPILib documentation & guides: https://docs.wpilib.org/
- WPILib example projects (Java): https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples

Unless otherwise noted in this repo, you may assume the project targets the current-season WPILib release and that all types listed in the API reference above are available.

---

## 2. Core Robot Programming Model

Typical FRC Java robots use one of these patterns:

1. **Iterative / Timed Robot** (class extends `TimedRobot`)
2. **Command-Based Framework** (using `edu.wpi.first.wpilibj2.command.*`)
3. **Simple Robot / Example Style** (less common in newer projects)

Key lifecycle concepts the AI assistant can assume exist in a standard FRC robot project:

- **Robot entry point**: `Robot` class (usually in `src/main/java/frc/robot/Robot.java`), often extending `TimedRobot`.
- **RobotContainer**: configuration class that wires up subsystems, commands, and controller bindings (in command-based projects).
- **Subsystems**: classes that represent hardware groupings (e.g., drivetrain, arm, elevator) and encapsulate control logic.
- **Commands**: actions the robot can perform (drive, shoot, intake, auto routines) using the command-based framework.

When generating code, **prefer command-based patterns** when this repository already uses `edu.wpi.first.wpilibj2.command` classes.

---

## 3. High-Level Package Map

Below is a high-level grouping of the most important WPILib Java packages for FRC robot programming. The assistant may use any of these APIs, and can reference the linked docs for details.

### 3.1 Core Robot & Hardware Abstractions

- **`edu.wpi.first.wpilibj`**
  - Core robot classes: `TimedRobot`, `RobotBase`, `RobotController`.
  - Hardware devices: `PWMVictorSPX`, `Spark`, `Talon`, `VictorSP`, `DigitalInput`, `AnalogInput`, `Encoder`, `Gyro`, `ADXRS450_Gyro`, `BuiltInAccelerometer`, etc.
  - Sensors: `Ultrasonic`, `DigitalInput`, `AnalogInput`, `Counter`, `DutyCycleEncoder`.
  - Joysticks & controllers: `Joystick`, `XboxController`, `PS4Controller`, `GenericHID`.
  - Timers & utilities: `Timer`, `DriverStation`, `RobotState`.

- **`edu.wpi.first.wpilibj.motorcontrol`**
  - Generic motor controller abstractions: `PWMVictorSPX`, `PWMSparkMax`, `MotorControllerGroup`.
  - Use these when you want vendor-agnostic motor control types.

- **`edu.wpi.first.wpilibj.drive`**
  - High-level drive helpers: `DifferentialDrive`, `MecanumDrive`, `KilloughDrive`.
  - Use for simple robot drive code given left/right (or wheel) motor groups.

- **`edu.wpi.first.wpilibj2.command`**
  - Command-based framework: `Command`, `Subsystem`, `CommandScheduler`, `SequentialCommandGroup`, `ParallelCommandGroup`, `InstantCommand`, `RunCommand`, `FunctionalCommand`, etc.
  - **Button bindings**: `edu.wpi.first.wpilibj2.command.button` (`JoystickButton`, `Trigger`).
  - Auto routines can be built as command groups and scheduled from `RobotContainer`.

- **`edu.wpi.first.wpilibj.simulation`**
  - Simulation versions of devices (encoders, gyros, power distribution, etc.) used when running robot code in simulation on a desktop.

---

### 3.2 Math, Control, and Trajectories

All math/control constructs live under `edu.wpi.first.math` and its subpackages:

- **`edu.wpi.first.math.geometry`**
  - 2D/3D geometry types: `Translation2d`, `Rotation2d`, `Pose2d`, `Transform2d`, `Twist2d`.
  - Essential for pose estimation, path following, and odometry.

- **`edu.wpi.first.math.kinematics`**
  - Kinematics for drivetrains: `DifferentialDriveKinematics`, `MecanumDriveKinematics`, `SwerveDriveKinematics`.
  - Converts chassis speeds to individual wheel speeds and back.

- **`edu.wpi.first.math.controller`**
  - Controllers: `PIDController`, `ProfiledPIDController`, `ArmFeedforward`, `SimpleMotorFeedforward`, `HolonomicDriveController`, etc.
  - Use these for closed-loop control of mechanisms and trajectories.

- **`edu.wpi.first.math.trajectory`**
  - Path generation and trajectory following: `Trajectory`, `TrajectoryGenerator`, `TrajectoryConfig`, constraints classes.
  - Use with `RamseteController` or holonomic controllers for autonomous path following.

- **`edu.wpi.first.math.filter` / `edu.wpi.first.math.estimator`**
  - Filters and estimators: `SlewRateLimiter`, `LinearFilter`, Kalman filters, pose estimators (for drivetrain + vision fusion).

- **`edu.wpi.first.units` and subpackages (`measure`, `mutable`, `collections`)**
  - Strongly-typed units (meters, seconds, volts, amps, etc.) for safer numeric calculations.
  - Use when you want compile-time unit checking (optional but recommended in newer code).

---

### 3.3 Networking, Dashboards, and Telemetry

- **`edu.wpi.first.networktables`**
  - NetworkTables client API; used for sending data between robot, driver station, and coprocessors.
  - Core types: `NetworkTableInstance`, `NetworkTable`, `DoubleEntry`, `BooleanEntry`, etc.

- **`edu.wpi.first.wpilibj.shuffleboard`**
  - Shuffleboard integration: layout of widgets on a dashboard for tuning and monitoring.

- **`edu.wpi.first.wpilibj.smartdashboard`**
  - SmartDashboard integration: quick key–value data publishing (`SmartDashboard.putNumber`, etc.).

- **`edu.wpi.first.util.sendable`**
  - `Sendable` interface and related helpers to expose objects to dashboards and tuning tools.

- **`edu.wpi.first.util.datalog`**
  - Data logging utilities for capturing time-series data on the robot for later analysis.

- **`edu.wpi.first.epilogue` & `edu.wpi.first.epilogue.logging`**
  - Structured logging/tracing tools (newer APIs for richer robot telemetry).

---

### 3.4 Vision, Cameras, and Image Processing

- **`edu.wpi.first.cameraserver`**
  - High-level helpers to start camera streams from USB cameras or other sources (`CameraServer.startAutomaticCapture`).

- **`edu.wpi.first.cscore` / `edu.wpi.first.cscore.raw`**
  - Lower-level camera capture & streaming system (`UsbCamera`, `CvSink`, `CvSource`).

- **`edu.wpi.first.vision`**
  - Helpers for integrating OpenCV pipelines in robot code.

- **`edu.wpi.first.apriltag`** and `jni` variant
  - AprilTag detection support on the roboRIO (or coprocessor) using built-in WPILib support.

---

### 3.5 HAL, Simulation, and Low-Level Utilities

Most robot code should **not** directly use the low-level HAL packages, but they are available when needed:

- **`edu.wpi.first.hal` and subpackages (`can`, `communication`, `simulation`, `util`)**
  - Hardware Abstraction Layer, CAN message primitives, simulation hooks, etc.

- **`edu.wpi.first.util` and subpackages**
  - Utility helpers: concurrency (`util.concurrent`), functions (`util.function`), protobuf integration (`util.protobuf`), cleanup & resource management, struct parsing, etc.

Use these only when you need lower-level access than the standard `wpilibj` APIs provide.

---

## 4. Typical Tasks and Where to Look

For an AI coding assistant, here is where to look for common robot tasks:

- **Drive a differential drivetrain**
  - Use `DifferentialDrive` (package `edu.wpi.first.wpilibj.drive`) with left/right `MotorControllerGroup`s.
  - Use joystick input from `Joystick` or `XboxController` (`edu.wpi.first.wpilibj`).

- **Create a command-based drivetrain subsystem**
  - Subclass `Subsystem` or `SubsystemBase` from `edu.wpi.first.wpilibj2.command`.
  - Expose methods like `arcadeDrive(double fwd, double rot)` or `tankDrive(double left, double right)` that call into `DifferentialDrive`.

- **Add joystick button bindings**
  - Use `edu.wpi.first.wpilibj2.command.button.JoystickButton` (or `Trigger`) in `RobotContainer`.
  - Bind to commands with `.whileTrue()`, `.onTrue()`, `.toggleOnTrue()`, etc.

- **Implement an autonomous trajectory**
  - Define kinematics constants with `DifferentialDriveKinematics` (`edu.wpi.first.math.kinematics`).
  - Use `TrajectoryConfig` + `TrajectoryGenerator` (`edu.wpi.first.math.trajectory`).
  - Follow the trajectory with `RamseteCommand` (`edu.wpi.first.wpilibj2.command`) and a `RamseteController` (`edu.wpi.first.math.controller`).

- **Publish sensor and state data to dashboard**
  - Use `SmartDashboard.putNumber/Boolean/String` (`edu.wpi.first.wpilibj.smartdashboard`).
  - Or set up `Shuffleboard` tabs/entries for more advanced layout.

- **Run the robot in simulation**
  - Use `wpilibj.simulation` classes (e.g., `DifferentialDrivetrainSim`, encoder/gyro sims) plus the simulation GUI described in WPILib docs.

---

## 5. How the AI Assistant Should Use This File

When generating or editing code in this repository, the AI assistant should:

1. **Prefer existing project patterns**  
   - If a `RobotContainer`, `SubsystemBase`, or command classes already exist, follow those patterns rather than inventing new structures.

2. **Use WPILib APIs where appropriate**  
   - For hardware, prefer `edu.wpi.first.wpilibj` and `edu.wpi.first.wpilibj.motorcontrol` types.
   - For commands, use `edu.wpi.first.wpilibj2.command` and its utilities.
   - For advanced control, reach into `edu.wpi.first.math.*` only when needed.

3. **Respect FRC constraints**  
   - Avoid long blocking loops in robot code; use commands, timers, or non-blocking patterns instead.
   - Do not use `Thread.sleep` in periodic methods or command `execute()`/`initialize()` without strong justification.

4. **Refer to official documentation when unclear**  
   - If an API’s exact behavior or parameters are ambiguous, consult the online Javadoc at:
     - https://github.wpilib.org/allwpilib/docs/release/java/index.html

5. **Keep code deployable on a roboRIO**  
   - Use only libraries that are part of WPILib or otherwise known to be compatible with the FRC robot environment, unless this repository explicitly includes additional vendor libraries.

---

## 6. Useful External References (for the Human and the AI)

- WPILib Docs (programming guides, command-based tutorial, etc.):  
  https://docs.wpilib.org/

- FRC Game Tools & WPILib installation instructions:  
  https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html

- WPILib Java API reference (Javadoc):  
  https://github.wpilib.org/allwpilib/docs/release/java/index.html

- WPILib Example Projects (Java):  
  https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples

These links can be included as comments in code or referenced during code generation to ensure that the AI uses valid, season-appropriate APIs.
