// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristToPos extends CommandBase {

  private WristSubsystem wristSubsystem;
  private DoubleSupplier desiredPosition;

  /** Creates a new WristToPos. */
  public WristToPos(WristSubsystem wristSubsystem, DoubleSupplier desiredPosition) {

    this.wristSubsystem = wristSubsystem;
    this.desiredPosition = desiredPosition;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristSubsystem.wristPIDToPos(desiredPosition.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
