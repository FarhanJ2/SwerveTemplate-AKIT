

# Swerve Base Template :: Steel Hawks
![Logo](https://www.steelhawks.net/logo.svg)


## Authors

- [@steelhawks](https://www.github.com/steelhawks)
- [@farhanj2](https://www.github.com/farhanj2)


## Acknowledgements

 - [WPILIB Version: v2024.3.2](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.3.2)



## Project Structure

# Project File Structure

```plaintext
SwerveTemplate/
├── src/
│   ├── main/
│   │   │
│   │   deploy/
│   │   │
│   │   └── java/
│   │       ├── org.steelhawks/
│   │       │   ├── commands/
│   │       │   │   ├── SuperStructure.java   # The command factory for commands that require multiple subsystems
│   │       │   ├── lib/                      # The folder with all custom libraries, such as vision with Limelight, and implementations in code such as odometry.
│   │       │   ├── subsystems/               # The main folder where all the subsystem logic is written
│   │       │   │    ├── Swerve.java          # The subsystem that controls the movement of the drivetrain
│   │       │   ├── Constants.java            # Configuration for motor ports, speeds, etc.
│   │       │   ├── Autos.java                # Configuration for Autonomous commands and to use vision or not
│   │       │   ├── CTREConfigs.java          # Configuration of motors in the drivetrain
│   │       │   ├── SwerveModule.java         # Controls individual module's speed and direction
│   │       │   ├── OperatorLock.java         # Enum to control different key mappings
│   │       │   ├── RobotMode.java            # Enum to control different key mappings
│   │       │   └── RobotContainer.java       # Manages controls and subsystems
│   └── test/
│       └── java/                                 # Unit tests (if any)
├── vendordeps/                                   # External vendor libraries
├── build.gradle                                  # Gradle build file
├── settings.gradle                               # Gradle settings
└── WPILib-License.md                             # WPILib license info
```

## Explanation of Project Structure

### Root Directory
- **`build.gradle`**: Defines project dependencies, build tasks, and configuration for Gradle.
- **`settings.gradle`**: Specifies project settings and root directory info for Gradle.
- **`vendordeps/`**: Contains vendor dependency files, crucial for hardware interfaces (e.g., CTRE or Rev Robotics libraries).
- **`WPILib-License.md`**: Licensing information for WPILib, which provides the core libraries for FRC robot programming.

### `src/main/java/org/steelhawks/`
- **`Constants.java`**: Holds all constant values, such as motor ports, PID settings, and other configurations.
- **`SwerveModule.java`**: Manages each swerve module’s direction and speed, allowing independent control over each wheel.
- **`CTREConfigs.java`**: Configures settings for CTRE motor controllers, ensuring efficient and safe control of drivetrain motors.
- **`RobotContainer.java`**: Links subsystems, commands, and input devices, acting as the central command hub for the robot code.
- **`OperatorLock.java`** & **`RobotMode.java`**: Enumerations that handle different control modes and operator mappings for flexible user input setups.

### Commands
- **`commands/SuperStructure.java`**: A command factory for handling complex commands involving multiple subsystems, making it easier to coordinate between drivetrain, intake, and shooter, if applicable.

### `subsystems/`
- **`Swerve.java`**: Main subsystem that manages the swerve drive logic, allowing the robot to move omnidirectionally by coordinating individual `SwerveModule` instances.

### `lib/`
- **Library Code**: This folder is designed to store helper classes and utilities that support core functionality, like math utilities, sensor handling, or other common tasks.

### `src/test/java`
- **Testing**: Contains unit tests or simulation-based tests, allowing verification of subsystem and command behavior in a safe environment.

# Important Notes
<p>When you make a clone of this repo, make sure to change the following.</p>
<p>1. Change the CANIVORE_NAME in Constants.java to your CAN loop name.</p>
<p>2. Carefully redo ALL constants, including a redo of all vision constants.</p>
