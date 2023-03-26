// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SecondJoint extends SubsystemBase {

  CANSparkMax secondJoint;
  CANSparkMax secondJointFollower;
  AbsoluteEncoder secondJointEncoder;
  ProfiledPIDController secondJointController;

  /** Creates a new Arm. */
  public SecondJoint() {
    secondJoint = new CANSparkMax(Constants.ArmConstants.SecondJointConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondJointFollower = new CANSparkMax(Constants.ArmConstants.SecondJointConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondJointFollower.follow(secondJoint, true);

    secondJoint.setIdleMode(IdleMode.kBrake);
    secondJointFollower.setIdleMode(IdleMode.kBrake);

    secondJointEncoder = secondJoint.getAbsoluteEncoder(Type.kDutyCycle);
    secondJointEncoder.setPositionConversionFactor(360 * Constants.ArmConstants.SecondJointConstants.kSecondJointGearRatio);

    secondJoint.burnFlash();
    secondJointFollower.burnFlash();

    secondJointController = new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    secondJointController.setTolerance(2);
  }

  public double convertTicksToAngle(double ticks) {
    return ticks / Constants.ArmConstants.SecondJointConstants.kSecondJointGearRatio;
  }

  public double convertAngleToTicks(double angle) {
    return (angle * Constants.ArmConstants.SecondJointConstants.kSecondJointGearRatio);
  }

  public double getAngle() {
    return convertTicksToAngle(secondJointEncoder.getPosition());
  }

  public void setPosition(double angle) {
    secondJointController.setGoal(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
