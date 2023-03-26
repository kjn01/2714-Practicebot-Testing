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

public class BaseJoint extends SubsystemBase {

  private CANSparkMax baseJoint;
  private CANSparkMax baseJointFollower;
  private AbsoluteEncoder baseJointEncoder;
  private ProfiledPIDController baseController;

  /** Creates a new Arm. */
  public BaseJoint() {
    baseJoint = new CANSparkMax(Constants.ArmConstants.BaseJointConstants.kBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    baseJointFollower = new CANSparkMax(Constants.ArmConstants.BaseJointConstants.kBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    baseJointFollower.follow(baseJoint, true);

    baseJoint.setIdleMode(IdleMode.kBrake);
    baseJointFollower.setIdleMode(IdleMode.kBrake);

    baseJointEncoder = baseJoint.getAbsoluteEncoder(Type.kDutyCycle);
    baseJointEncoder.setPositionConversionFactor(360 * Constants.ArmConstants.BaseJointConstants.kBaseJointGearRatio);

    baseJoint.burnFlash();
    baseJointFollower.burnFlash();

    baseController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    baseController.setTolerance(2);
  }

  public double convertTicksToAngle(double ticks) {
    return (ticks * 360) / Constants.ArmConstants.BaseJointConstants.kBaseJointGearRatio;
  }

  public double getAngle() {
    return convertTicksToAngle(baseJointEncoder.getPosition());
  }

  public void setPosition(double angle) {
    baseController.setGoal(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
