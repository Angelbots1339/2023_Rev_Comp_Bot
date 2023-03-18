// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.WristConstants.*;

import java.util.function.DoubleSupplier;

public class WristSubsystem extends SubsystemBase {

  private CANSparkMax wristMotor;
  private PIDController wristController;

  private ArmFeedforward wristFeedforward;

  private final AbsoluteEncoder wristEncoder;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {


    wristMotor = new CANSparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
    wristController = new PIDController(WRIST_P, WRIST_I, WRIST_D);
    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    wristFeedforward = new ArmFeedforward(WRIST_KS, WRIST_I, WRIST_D);

    wristMotor.setSmartCurrentLimit(WRIST_MOTOR_CURRENT_LIMIT);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", wristEncoder.getPosition());
  }

  public void disable() {
    wristMotor.set(0);
  }

  public void runWristAtPercent(double speed) {
    wristMotor.set(speed);
  }


/**
 * Make sure to call in periodic or bad things happen
 * 
 * @param position
 */
  public void wristPIDToPos(double position) {

    wristMotor.setVoltage(-(wristController.calculate(wristEncoder.getPosition(), position) + WRIST_KS));

  }

  public void runWristAtVolts(double volts) {
    wristMotor.setVoltage(volts);
    SmartDashboard.putNumber("Wrist Current", wristMotor.getOutputCurrent());
  }
}

