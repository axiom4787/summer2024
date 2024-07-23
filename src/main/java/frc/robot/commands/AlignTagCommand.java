// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignTagCommand extends Command {
  private DriveSubsystem m_driveSubsystem;
  private NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
  private PIDController m_turningPID = new PIDController(0.1, 0, 0.01);
  private XboxController m_controller = new XboxController(0);
  /** Creates a new AlignTagCommand. */
  public AlignTagCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    m_driveSubsystem = driveSubsystem;
    m_turningPID.setTolerance(0.5, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measured = m_limelight.getEntry("tx").getDouble(0);
    m_driveSubsystem.driveRobotRelative(            
      0,
      m_turningPID.calculate(measured, 0),
      0);
      //m_turningPID.calculate(measured, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
      (m_limelight.getEntry("tid").getInteger(0) != 5) ||
      m_turningPID.atSetpoint();
  }
}
