// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeShooterSubsystem extends SubsystemBase {
  private IntakeShooterState m_state;

  private CANSparkMax m_frontShooter, m_backShooter, m_topIndexer, m_bottomIndexer, m_floorIntake;
  private Spark m_blinkin = new Spark(0);

  /** Creates a new IntakeShooterSubsystem. */
  public IntakeShooterSubsystem() {
    m_frontShooter = new CANSparkMax(Constants.kFrontShooterId, MotorType.kBrushless);
    m_backShooter = new CANSparkMax(Constants.kBackShooterId, MotorType.kBrushless);
    m_topIndexer = new CANSparkMax(Constants.kTopIndexerId, MotorType.kBrushed);
    m_bottomIndexer = new CANSparkMax(Constants.kBottomIndexerId, MotorType.kBrushless);
    m_floorIntake = new CANSparkMax(Constants.kFloorIntakeId, MotorType.kBrushless);

    m_frontShooter.getEncoder()
      .setVelocityConversionFactor(Constants.kShooterConversionFactor);
    m_backShooter.getEncoder()
      .setVelocityConversionFactor(Constants.kShooterConversionFactor);

    m_frontShooter.setIdleMode(IdleMode.kCoast);
    m_backShooter.setIdleMode(IdleMode.kCoast);
    m_topIndexer.setIdleMode(IdleMode.kCoast);
    m_bottomIndexer.setIdleMode(IdleMode.kCoast);
    m_floorIntake.setIdleMode(IdleMode.kCoast);

    m_backShooter.setSmartCurrentLimit(40);
    m_frontShooter.setSmartCurrentLimit(40);
    m_topIndexer.setSmartCurrentLimit(20);
    m_bottomIndexer.setSmartCurrentLimit(20);
    m_floorIntake.setSmartCurrentLimit(40);


    m_state = IntakeShooterState.kOff;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Shooter Velocity", m_frontShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("Back Shooter Velocity", m_backShooter.getEncoder().getVelocity());
    
    switch (m_state)
    {
      case kOff:
        m_frontShooter.set(0);
        m_backShooter.set(0);
        m_topIndexer.set(0);
        m_bottomIndexer.set(0);
        m_floorIntake.set(0);
        m_blinkin.set(0.91);
        break;
      case kShoot:
        m_frontShooter.set(Constants.kShooterLaunchDutyCycle);
        m_backShooter.set(Constants.kShooterLaunchDutyCycle);
        m_floorIntake.set(0);
        if (m_frontShooter.getEncoder().getVelocity() > Constants.kShooterShootSpeed
          && m_backShooter.getEncoder().getVelocity() > Constants.kShooterShootSpeed)
        {
          m_topIndexer.set(Constants.kTopIndexerLaunchDutyCycle);
          m_bottomIndexer.set(Constants.kBottomIndexerDutyCycle);
          m_blinkin.set(0.15);
        }
        else
        {
          m_topIndexer.set(0);
          m_bottomIndexer.set(0);
          m_blinkin.set(0.13);
        }
        break;
      case kSpit:
        m_frontShooter.set(0);
        m_backShooter.set(0);
        m_topIndexer.set(Constants.kTopIndexerIntakeDutyCycle);
        m_bottomIndexer.set(-Constants.kBottomIndexerDutyCycle);
        m_floorIntake.set(Constants.kFloorIntakeDutyCycle);
        m_blinkin.set(0.61);
        break;
      case kSourceIntake:
        m_frontShooter.set(Constants.kShooterIntakeDutyCycle);
        m_backShooter.set(Constants.kShooterIntakeDutyCycle);
        m_topIndexer.set(Constants.kTopIndexerIntakeDutyCycle);
        m_bottomIndexer.set(Constants.kBottomIndexerDutyCycle);
        m_floorIntake.set(0);
        m_blinkin.set(0.73);
        break;
      case kFloorIntake:
        m_frontShooter.set(0);
        m_backShooter.set(0);
        m_topIndexer.set(Constants.kTopIndexerIntakeDutyCycle);
        m_bottomIndexer.set(Constants.kBottomIndexerDutyCycle);
        m_floorIntake.set(-Constants.kFloorIntakeDutyCycle);
        m_blinkin.set(0.83);
        break;
    }
//    System.out.printf("front shooter: %.2f, back shooter: %.2f\n", m_frontShooter.getEncoder().getVelocity(), m_backShooter.getEncoder().getVelocity());
  }

  public void setDesiredState(IntakeShooterState state)
  {
    m_state = state;
  }

  public enum IntakeShooterState
  {
    kOff,
    kShoot,
    kSpit,
    kSourceIntake,
    kFloorIntake,
  }
}
