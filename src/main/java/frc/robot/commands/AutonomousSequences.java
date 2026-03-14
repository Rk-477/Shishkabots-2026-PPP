package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.Set;

public final class AutonomousSequences {
  private static final double AUTO_BACKUP_SPEED = -0.28;
  private static final double AUTO_BACKUP_TIME_SEC = 1.35;
  private static final double AUTO_SMALL_TURN_SPEED = 0.20;
  private static final double AUTO_SMALL_TURN_TIME_SEC = 0.45;
  private static final double AUTO_ALIGN_TIMEOUT_SEC = 2.5;
  private static final double AUTO_SHOOT_TIME_SEC = 2.3;

  private AutonomousSequences() {
    throw new UnsupportedOperationException("Utility class");
  }

  public static Command middle(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    return Commands.sequence(
        driveRobotRelativeFor(driveSubsystem, AUTO_BACKUP_SPEED, 0.0, 0.0, AUTO_BACKUP_TIME_SEC),
        Commands.waitSeconds(0.10),
        shootFor(shooterSubsystem, AUTO_SHOOT_TIME_SEC),
        Commands.runOnce(driveSubsystem::stop, driveSubsystem));
  }

  public static Command left(
      LimelightSubsystem limelightSubsystem,
      DriveSubsystem driveSubsystem) {
    return Commands.sequence(
        driveRobotRelativeFor(driveSubsystem, AUTO_BACKUP_SPEED, 0.0, 0.0, AUTO_BACKUP_TIME_SEC),
        driveRobotRelativeFor(driveSubsystem, 0.0, 0.0, -AUTO_SMALL_TURN_SPEED, AUTO_SMALL_TURN_TIME_SEC),
        new AutoAllign(limelightSubsystem, driveSubsystem, Set.of(25, 9)).withTimeout(AUTO_ALIGN_TIMEOUT_SEC),
        Commands.runOnce(driveSubsystem::stop, driveSubsystem));
  }

  public static Command right(
      LimelightSubsystem limelightSubsystem,
      DriveSubsystem driveSubsystem) {
    return Commands.sequence(
        driveRobotRelativeFor(driveSubsystem, AUTO_BACKUP_SPEED, 0.0, 0.0, AUTO_BACKUP_TIME_SEC),
        driveRobotRelativeFor(driveSubsystem, 0.0, 0.0, AUTO_SMALL_TURN_SPEED, AUTO_SMALL_TURN_TIME_SEC),
        new AutoAllign(limelightSubsystem, driveSubsystem, Set.of(26, 10)).withTimeout(AUTO_ALIGN_TIMEOUT_SEC),
        Commands.runOnce(driveSubsystem::stop, driveSubsystem));
  }

  private static Command driveRobotRelativeFor(
      DriveSubsystem driveSubsystem,
      double xSpeed,
      double ySpeed,
      double rotSpeed,
      double seconds) {
    return Commands.run(
        () -> driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, false),
        driveSubsystem).withTimeout(seconds);
  }

  private static Command shootFor(ShooterSubsystem shooterSubsystem, double seconds) {
    return Commands.startEnd(
        () -> shooterSubsystem.setShooterPower(0.85),
        shooterSubsystem::stop,
        shooterSubsystem).withTimeout(seconds);
  }
}
