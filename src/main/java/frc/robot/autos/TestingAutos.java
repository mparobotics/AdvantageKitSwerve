package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class TestingAutos {

  public static Command ExampleAuto(Drive drive) {
    return Commands.sequence(
        // write auto commands here
        DriveCommands.followPath("Forward Test", false),
        Commands.waitSeconds(1),
        DriveCommands.driveMeters(drive, 0, 0, 180, 0),
        Commands.waitSeconds(2),
        DriveCommands.followPath("Back Test", false));
  }
}
