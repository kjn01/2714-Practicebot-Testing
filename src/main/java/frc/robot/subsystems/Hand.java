// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hand extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax wrist;
  private AbsoluteEncoder wristEncoder;

  private CANSparkMax claw;
  private AbsoluteEncoder clawEncoder;
  
  public Hand() {
    wrist = new CANSparkMax(Constants.HandConstants.kWristMotorCanId, MotorType.kBrushless);
    wrist.setIdleMode(IdleMode.kBrake);
    wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
    wristEncoder.setPositionConversionFactor(360 * Constants.HandConstants.kWristGearRatio);

    // claw = new CANSparkMax(Constants.HandConstants.kClawMotorCanId, MotorType.kBrushless);
    // claw.setIdleMode(IdleMode.kBrake);
    // clawEncoder = claw.getAbsoluteEncoder(Type.kDutyCycle);
    // clawEncoder.setPositionConversionFactor(360 * Constants.HandConstants.kClawGearRatio);

    wrist.burnFlash();

  }

  public void rotateWrist() {
    
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
