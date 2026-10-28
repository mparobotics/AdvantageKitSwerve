// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 5.0; // to tune
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(28);
  public static final double wheelBase = Units.inchesToMeters(28);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  /** Parameter values for one swerve module. */
  public record ModuleParams(
      Translation2d location,
      int turnCanId,
      int driveCanId,
      int canCoderCanId,
      Rotation2d zeroRotation) {}

  public static final ModuleParams FRONT_LEFT =
      new ModuleParams(
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // location
          10, // turnCanId
          11, // driveCanId
          12, // canCoderCanId
          new Rotation2d(-0.854)); // zeroRotation
  public static final ModuleParams FRONT_RIGHT =
      new ModuleParams(
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // location
          13, // turnCanId
          14, // driveCanId
          15, // canCoderCanId
          new Rotation2d(-2.259)); // zeroRotation
  public static final ModuleParams BACK_LEFT =
      new ModuleParams(
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // location
          16, // turnCanId
          17, // driveCanId
          18, // canCoderCanId
          new Rotation2d(-1.093)); // zeroRotation
  public static final ModuleParams BACK_RIGHT =
      new ModuleParams(
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0), // location
          19, // turnCanId
          20, // driveCanId
          21, // canCoderCanId
          new Rotation2d(2.857)); // zeroRotation

  // List of module parameters in the *required* order:
  // --> FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
  // - PathPlanner *requires* this order
  // - This order is used everywhere else to ensure consistency between all components
  public static final ModuleParams[] moduleParams = {
    FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
  };

  // Module translations array generated in the same order as moduleParams array
  public static final Translation2d[] moduleTranslations =
      Arrays.stream(moduleParams).map(ModuleParams::location).toArray(Translation2d[]::new);

  // Pigeon CAN ID
  public static final int pigeonCanId = 23;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction = (6.75 / 1);
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.01;
  public static final double driveKi = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = (12.8 / 1);
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 0.8;
  public static final double turnKi = 0.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
