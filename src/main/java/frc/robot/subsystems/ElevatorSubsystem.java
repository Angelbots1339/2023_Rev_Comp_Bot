package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax leadMotor;
  private CANSparkMax followMotor;
  private SparkMaxPIDController pidController;
  // make channels constant
  private final Encoder encoder = new Encoder(1, 2);

  private final TrapezoidProfile.Constraints constraints;
  private final ProfiledPIDController controller;

  public ElevatorSubsystem() {
    leadMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
    followMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);
    followMotor.follow(leadMotor, true);

    leadMotor.setIdleMode(IdleMode.kBrake);
    followMotor.setIdleMode(IdleMode.kBrake);


    pidController = leadMotor.getPIDController();
    constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
    controller =  new ProfiledPIDController(ElevatorConstants.toPosP, ElevatorConstants.toPosI, ElevatorConstants.toPosD, constraints, ElevatorConstants.kDt);

    encoder.setDistancePerPulse(Units.inchesToMeters(1.7) * Math.PI * 2 /  42);
    
    leadMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
    followMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT);

  }

  public boolean elevatorToPos(double goal) {

    double output = controller.calculate(leadMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition(), goal);
    leadMotor.setVoltage(output);

    return controller.atGoal();
  }

  public void disable() {
    leadMotor.set(0);
    followMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getGoal() {
    return controller.getGoal().position;
  }

  public void runElevatorAtPercent(double speed) {
    leadMotor.set(speed);
  }
}
