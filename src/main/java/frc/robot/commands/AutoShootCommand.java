// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem.IntakeShooterState;

public class AutoShootCommand extends Command {
  private final Timer timer = new Timer();
  private final double duration = 5;
  
  private IntakeShooterSubsystem m_intakeShooterSubsystem;
  /** Creates a new AutoShootCommand. 
   * 
   * @param time 
  */
  public AutoShootCommand(IntakeShooterSubsystem intakeShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeShooterSubsystem = intakeShooterSubsystem;
    addRequirements(intakeShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kShoot);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
