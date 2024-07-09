// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase 
{
  private ClimberState m_state;
  private CANSparkMax m_leftClimber, m_rightClimber;

  public ClimberSubsystem() 
  {
    m_leftClimber = new CANSparkMax(Constants.kLeftClimberId, MotorType.kBrushless);
    m_rightClimber = new CANSparkMax(Constants.kRightClimberId, MotorType.kBrushless);
    
    m_leftClimber.setIdleMode(IdleMode.kBrake);
    m_rightClimber.setIdleMode(IdleMode.kBrake);

    m_leftClimber.setSmartCurrentLimit(40);
    m_rightClimber.setSmartCurrentLimit(40);

    m_state = ClimberState.kOff;
  }

  @Override
  public void periodic() 
  {
    switch (m_state)
    {
      case kRetract:
        m_leftClimber.set(Constants.kClimberDutyCycle);
        m_rightClimber.set(-Constants.kClimberDutyCycle);
        break;
      case kExtend:
        m_leftClimber.set(-Constants.kClimberDutyCycle);
        m_rightClimber.set(Constants.kClimberDutyCycle);
        break;
      case kOff:
        m_leftClimber.set(0);
        m_rightClimber.set(0);
        break;
    }
  }

  public void setDesiredState(ClimberState state)
  {
    if (state == m_state)
      return;
    m_state = state;
  }

  public ClimberState getCurrentState()
  {
    return m_state;
  }

  public enum ClimberState
  {
    kOff,
    kExtend,
    kRetract,
  }
}
