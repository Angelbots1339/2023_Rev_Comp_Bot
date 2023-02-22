package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeMotor;

public class Intake extends CommandBase {
  double speed;
  private IntakeMotor intakeMotor;

  public Intake(IntakeMotor intakeMotor, double inputSpeed) { //when the command is initialized the speed is put straight in
    this.intakeMotor = intakeMotor;
    requires(intakeMotor);
    this.speed = inputSpeed;
  }

  private void requires(IntakeMotor intakeMotor2) {
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  /**
   * sets motor speed to the input
   * @author Ian
   */
  @Override
  public void execute() {
    intakeMotor.driveMotor(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    intakeMotor.driveMotor(0.0);
  }


}