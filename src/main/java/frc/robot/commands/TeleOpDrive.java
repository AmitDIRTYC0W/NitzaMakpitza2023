// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleOpDrive extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final XboxController controller;

  /** Creates a new TeleOpDrive. */
  public TeleOpDrive(DrivetrainSubsystem drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = new ChassisSpeeds(
      controller.getLeftX(),
      controller.getLeftY(),
      controller.getRightX()
    );
    drivetrain.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
