// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SecondJoint extends SubsystemBase {

  private CANSparkMax secondJoint;
  private CANSparkMax secondJointFollower;
  private AbsoluteEncoder secondJointEncoder;
  private ProfiledPIDController secondJointController;
  private ArmFeedforward secondJointFF;

  /** Creates a new Arm. */
  public SecondJoint() {
    secondJoint = new CANSparkMax(Constants.ArmConstants.SecondJointConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondJointFollower = new CANSparkMax(Constants.ArmConstants.SecondJointConstants.kFollowerSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondJointFollower.follow(secondJoint, true);

    secondJoint.setIdleMode(IdleMode.kBrake);
    secondJointFollower.setIdleMode(IdleMode.kBrake);

    secondJointEncoder = secondJoint.getAbsoluteEncoder(Type.kDutyCycle);
    secondJointEncoder.setPositionConversionFactor(2 * Math.PI * Constants.ArmConstants.SecondJointConstants.kSecondJointGearRatio);
    secondJointEncoder.setZeroOffset(Constants.ArmConstants.SecondJointConstants.kSecondJointZeroOffset);

    secondJoint.burnFlash();
    secondJointFollower.burnFlash();

    secondJointController = new ProfiledPIDController(0.001, 0, 0, new TrapezoidProfile.Constraints(0.1, 0.1));
    secondJointController.setTolerance(Units.degreesToRadians(2));

    secondJointFF = new ArmFeedforward(0, 0.35, 4.38, 0.03);
  }

  public double convertTicksToAngle(double angle) {
    double newAngle = angle;
    newAngle -= Constants.ArmConstants.SecondJointConstants.kSecondJointKinematicOffset;
    return (newAngle / Constants.ArmConstants.SecondJointConstants.kSecondJointGearRatio);
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
    secondJoint.setVoltage(secondJointController.calculate(getAngle(),
    secondJointController.getGoal()) + 
    secondJointFF.calculate(secondJointController.getSetpoint().position, 0));

    SmartDashboard.putNumber("Second Joint Goal", Units.radiansToDegrees(secondJointController.getGoal().position));
    SmartDashboard.putNumber("Current Second Joint Angle", Units.radiansToDegrees(getAngle()));
  }
}
