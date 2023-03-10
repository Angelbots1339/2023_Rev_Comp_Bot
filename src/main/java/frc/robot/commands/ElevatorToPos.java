// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToPos extends CommandBase {

  private ElevatorSubsystem elevatorSubsystem;
  private DoubleSupplier goal;
  private boolean isAtSetpoint;

  /** Creates a new ElevatorToBottom. */
  public ElevatorToPos(ElevatorSubsystem elevatorSubsystem, DoubleSupplier goal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.goal = goal;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isAtSetpoint = elevatorSubsystem.elevatorToPos(goal.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAtSetpoint;
  }
}
