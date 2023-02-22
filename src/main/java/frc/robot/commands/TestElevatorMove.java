// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class TestElevatorMove extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private double goal;
  private double input;

  /** Creates a new ElevatorToBottom. */
  public TestElevatorMove(ElevatorSubsystem elevatorSubsystem, boolean leftInput, boolean rightInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);

    goal = elevatorSubsystem.getGoal();
    input = (rightInput ? 1 : 0) - (leftInput ? 1 : 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    goal += input / 100;
    elevatorSubsystem.elevatorToPos(goal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
