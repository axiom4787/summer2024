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
  private final double duration = 3000;
  double startTime = 0;
  double lastCheckTime = 0;
  
  private IntakeShooterSubsystem m_intakeShooterSubsystem;
  /** Creates a new AutoShootCommand. 
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
    startTime = Timer.getFPGATimestamp();
    timer.start();
    System.out.println("started?");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kOff);
    System.out.println("ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = timer.get();
    if (currentTime - lastCheckTime >= 0.2)
      System.out.println("checking");
      lastCheckTime = currentTime;
      if (timer.hasElapsed(duration)) {
        System.out.println("ending?");
        return true;
      }
    return false;
  }
}
