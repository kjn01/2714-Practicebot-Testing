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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    baseJointFollower = new CANSparkMax(Constants.ArmConstants.BaseJointConstants.kFollowerBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    baseJointFollower.follow(baseJoint, true);

    baseJoint.setIdleMode(IdleMode.kBrake);
    baseJointFollower.setIdleMode(IdleMode.kBrake);

    baseJointEncoder = baseJoint.getAbsoluteEncoder(Type.kDutyCycle);
    baseJointEncoder.setPositionConversionFactor(2 * Math.PI * Constants.ArmConstants.BaseJointConstants.kBaseJointGearRatio);
    baseJointEncoder.setZeroOffset(Constants.ArmConstants.BaseJointConstants.kBaseJointZeroOffset);

    baseJoint.burnFlash();
    baseJointFollower.burnFlash();

    baseController = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));
    baseController.setTolerance(Units.degreesToRadians(2));
  }

  public double convertTicksToAngle(double angle) {
    double newAngle = angle;
    newAngle -= Constants.ArmConstants.BaseJointConstants.kBaseJointKinematicOffset;
    return (newAngle / Constants.ArmConstants.BaseJointConstants.kBaseJointGearRatio);
  }

  public double getAngle() {
    return convertTicksToAngle(baseJointEncoder.getPosition());
  }

  public void setPosition(double angle) {
    baseController.setGoal(angle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Base Goal", Units.radiansToDegrees(baseController.getGoal().position));
    SmartDashboard.putNumber("Current Base Angle", Units.radiansToDegrees(getAngle()));
    // This method will be called once per scheduler run
  }
}
