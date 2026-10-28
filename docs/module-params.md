# Module parameters and module order

This note walks through why a drive command depends on swerve module order, the problem of keeping that order consistent, the `moduleParams` approach now in the code, and why an enum was not used.

## The Problem

### Robot Control
The robot is commanded to drive in a direction and to rotate. That desired robot motion is a `ChassisSpeeds`: a speed along X, a speed along Y, and a rotational speed.

#### SwerveDriveKinematics
The robot has four swerve modules, one at each corner. A `ChassisSpeeds` is a command for the whole robot. It has to be converted into a command for each module: how fast that wheel should drive, and which way that wheel should point. That per-module command is a `SwerveModuleState`.

`SwerveDriveKinematics` does the conversion. It is constructed with the four module locations. Each location is a `Translation2d`, the module's position relative to the center of the robot, with +X forward and +Y to the left. The locations are passed as an array, and the kinematics object stores them in that order. A slot has no corner name. The array order is the only identity it keeps.

The `DriveConstants.moduleTranslations` constant supplies the order for the module locations that are passed to the `SwerveDriveKinematics` constructor.

`SwerveDriveKinematics.toSwerveModuleStates` takes a `ChassisSpeeds` and returns a `SwerveModuleState` array. The states in this array are returned in the same order as the translations that were passed to the constructor.  The state in slot `i` is the `SwerveModuleState` for the module whose location was passed in slot `i`.

#### Swerve Modules

The robot has four swerve modules, one at each corner. These modules are defined using the `Module` class. The `Drive` class constructs these modules and stores them as an array, in the order the `ModuleIO` objects are passed in. That order is front-left, front-right, back-left, back-right.

`RobotContainer` chooses which module is passed for each constructor argument. This order is maintained independently of the order defined by the `DriveConstants.moduleTranslations` constant that is used to initialize the `SwerveDriveKinematics` object.


#### Dependencies Between SwerveDriveKinematics and Drive Module Orders

The `Drive.runVelocity` method is where agreement between the two orders becomes critical. This method does the following:
* Calls `SwerveDriveKinematics.toSwerveModuleStates` to obtain the desired state for each module
* Iterates over the `Drive.modules` array and assigns state[i] to module[i]

```java
for (int i = 0; i < 4; i++) {
  modules[i].runSetpoint(setpointStates[i]);
}
```

If the two arrays are not kept in the same order, the wrong module state will be assigned to one or more of the modules, resulting in incorrect driving behavior.

### Sensor Fusion

Data from the robot's sensors is used to estimate the robot's position and orientation.  That position and orientation is captured as a `Pose2d`: a field x, a field y, and a heading.

#### Swerve Module Position

The robot has four swerve modules, one at each corner.  Each module keeps track of how far its drive wheel has travels and the current turning angle.  This information is propvided by the `Module.getOdometryPositions` method as is retuernd as a `SwerveModulePosition`.

#### Gyro (Pigeon)

#### SwerveDrivePoseEstimator

The module positions information and the for the four modules and the gyro information needs to be combined to produce the robot's current position and orientation: a `Pose2d`.

`SwerveDrivePoseEstimator` does the conversion. It is constructed with the same `SwerveDriveKinematics` object, so the module locations it uses come from `DriveConstants.moduleTranslations`, in that order. `SwerveDrivePoseEstimator.updateWithTime` takes a gyro heading and a `SwerveModulePosition` array.

The position in slot `i` is treated as the wheel whose location was passed to kinematics in slot `i`. The gyro supplies the heading. The wheel positions supply the change in field x and field y.

#### Dependencies Between SwerveDrivePoseEstimator and Drive Module Orders

`Drive.periodic` is where agreement between this order and the `Drive.modules` order becomes critical. This method does the following:
* Reads a `SwerveModulePosition` from each entry of `Drive.modules` and stores it at the same index
* Passes that array, with the gyro heading, to `SwerveDrivePoseEstimator.updateWithTime`

```java
for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
  modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
}
poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
```

If the two arrays are not kept in the same order, the distance measured at one corner is applied as if that wheel sat at another corner. The estimator still returns a pose. The gyro supplies the heading, so the reported rotation can look right while the field position is wrong.

`getChassisSpeeds` uses the pairing in the other direction. `getModuleStates` packs the measured state of `modules[i]` into slot `i`, and `toChassisSpeeds` reads that array in location order to produce a `ChassisSpeeds`. `setPose` passes `getModulePositions()` to the estimator the same way.

PathPlanner depends on this chain. `DriveConstants` passes the location array to PathPlanner's `RobotConfig`, and PathPlanner requires that array to be front-left, front-right, back-left, back-right. When a path should be followed, PathPlanner calls `runVelocity` with a `ChassisSpeeds`, so `modules` has to be in that same order. Kinematics will accept any order, as long as both lists use it. The PathPlanner requirement is what fixes this robot's order as front-left, front-right, back-left, back-right.

These steps are correct only when `modules` is in the same order as the locations given to kinematics.

Before this change, no single definition owned that order. It was repeated in places the compiler did not connect:

- `moduleTranslations` was the location array given to kinematics and to PathPlanner.
- Each corner's drive CAN ID, turn CAN ID, CANcoder ID, and steering zero was its own named constant. Those constants were not stored in the same order as the location array. The constant names were the only association between a location and its hardware.
- `ModuleIOSpark` took an `int` and used `switch` statements to turn that index into one of those constants.
- `RobotContainer` passed `0`, `1`, `2`, and `3` into the `Drive` constructor. Those integers were assumed to mean the same corners as the matching slots of `moduleTranslations`.
- `Drive` stored the resulting objects in `modules[0]` through `modules[3]`, which is the array `runVelocity` and the odometry loop index into.

The integer was the only join between a location and a set of CAN IDs. Changing one list and leaving the others alone still compiled. The robot would drive, with the wheel at one corner commanded and measured as if it sat at another.

## The solution

`DriveConstants.ModuleParams` is a record with one module's location, turn CAN ID, drive CAN ID, CANcoder ID, and steering zero. The four corners are ordinary static fields of `DriveConstants`, written after `trackWidth` and `wheelBase`. The array that defines the order is `moduleParams`:

```java
public static final ModuleParams[] moduleParams = {
  FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
};
```

That order is front-left, front-right, back-left, back-right, which is the order PathPlanner requires.

Two values are built from this array and from nothing else:

- `moduleTranslations` maps each record to its location. `SwerveDriveKinematics`, `ppConfig`, and `stopWithX` read that array.
- `RobotContainer` maps each record to a `ModuleIO`. On the real robot that is a `ModuleIOSpark` constructed with that record. Simulation and replay walk the same array and substitute their own `ModuleIO` implementations.

`Drive` takes those `ModuleIO` objects as one array and stores them in `modules` in the order received. If the number of objects does not match `moduleTranslations`, construction throws.

`ModuleIOSpark` no longer switches on an index. It reads the CAN IDs and the steering zero from the record it was given.

## Why this keeps the order

Every ordered array starts from `moduleParams`.

`moduleTranslations` is produced by walking `moduleParams`. Kinematics is constructed from `moduleTranslations`, so its slot `i` is `moduleParams[i]`. PathPlanner receives that same location array. `stopWithX` reads headings from it.

`Drive.modules` is produced by walking `moduleParams` in `RobotContainer` and then storing the resulting array unchanged. `runVelocity`, odometry, `getModuleStates`, and `getModulePositions` use index `i` of `modules` together with index `i` of a kinematics result. Those results are in `moduleTranslations` order. The two indexes agree because both arrays were produced from `moduleParams`, not because one array was copied from the other.

Loops inside `Drive` still say `4` where they mean "every module." That number is the length of `moduleParams` today. It is not a second definition of which corner is which. Reordering `moduleParams` reorders locations, hardware, and `modules` together. The loops follow along by index.

## The enum alternative

An enum whose constants were declared front-left, front-right, back-left, back-right was considered. `values()` would have been the order, and each constant would have carried the same fields the record carries now.

The enum was dropped because of class initialization order.

Java initializes an enum as its own class. It builds the enum constants before it initializes any other static field of that enum. If a constant's constructor reads `trackWidth` or `wheelBase`, and those fields live on `DriveConstants`, constructing the enum can start `DriveConstants` while the enum is only partly built. `ppConfig` needs the module locations. Calling back into the enum at that moment does not wait. The current thread is already initializing the enum, so Java continues, and the later corners are not there yet.

The failure is not a compile error. Locations can come out as zero, or a call can see a null corner. Putting the enum text inside `DriveConstants` does not fix it. A nested enum is still a separate class. Java does not initialize it just because the source listing has reached that line.

The record avoids that split. `FRONT_LEFT` and the other three corners are static fields of `DriveConstants`. Referring to one of them starts `DriveConstants` at the top. `trackWidth` and `wheelBase` are assigned, then the four records are constructed, then `moduleParams` and `moduleTranslations` are built. One class initializer runs from top to bottom, which is the order written in the file.
