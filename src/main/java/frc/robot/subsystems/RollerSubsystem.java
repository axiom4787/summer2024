// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase 
{
  private RollerState m_state;
  private CANSparkMax m_rollerClaw;

  public RollerSubsystem() 
  {
    m_rollerClaw = new CANSparkMax(Constants.kRollerClawId, MotorType.kBrushless);
    
    m_rollerClaw.setIdleMode(IdleMode.kBrake);

    m_rollerClaw.setSmartCurrentLimit(40);

    m_state = RollerState.kOff;
  }

  @Override
  public void periodic() 
  {
    switch (m_state)
    {
      case kSpit:
        m_rollerClaw.set(Constants.kRollerClawDutyCycle);
        break;
      case kIntake:
        m_rollerClaw.set(-Constants.kRollerClawDutyCycle);
        break;
      case kOff:
        m_rollerClaw.set(0);
        break;
    }
  }

  public void setDesiredState(RollerState state)
  {
    if (state == m_state)
      return;
    m_state = state;
  }

  public RollerState getCurrentState()
  {
    return m_state;
  }

  public enum RollerState
  {
    kOff,
    kIntake,
    kSpit,
  }
}
