// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class RunWristAtVolts extends CommandBase {

  private WristSubsystem wristSubsystem;
  private DoubleSupplier desiredVolts;

  /** Creates a new WristToPos. */
  public RunWristAtVolts(WristSubsystem wristSubsystem, DoubleSupplier desiredVolts) {

    this.wristSubsystem = wristSubsystem;
    this.desiredVolts = desiredVolts;
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
    wristSubsystem.runWristAtVolts(desiredVolts.getAsDouble());;
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
