// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.IntakeShooterSubsystem.IntakeShooterState;
import frc.robot.subsystems.RollerSubsystem.RollerState;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeShooterSubsystem m_intakeShooterSubsystem = new IntakeShooterSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController m_controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void registerNamedCommands()
  {
    NamedCommands.registerCommand("shootNote", new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kShoot);
    }, m_intakeShooterSubsystem));
    NamedCommands.registerCommand("intakeNote", new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kFloorIntake);
    }, m_intakeShooterSubsystem));
    NamedCommands.registerCommand("spitNote", new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kSpit);
    }, m_intakeShooterSubsystem));
    NamedCommands.registerCommand("disableIntakeShooter", new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kOff);
    }, m_intakeShooterSubsystem));
  }

  private void configureBindings()
  {
    Trigger spitNote = m_controller.leftTrigger()
    .onTrue(new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kSpit);
    }));
    Trigger floorIntake = m_controller.leftBumper()
    .onTrue(new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kFloorIntake);
    }));
    Trigger sourceIntake = m_controller.rightBumper()
    .onTrue(new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kSourceIntake);
    }));
    Trigger shootNote = m_controller.rightTrigger()
    .onTrue(new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kShoot);
    }));
    spitNote.or(floorIntake).or(sourceIntake).or(shootNote).negate()
    .onTrue(new InstantCommand(() -> {
      m_intakeShooterSubsystem.setDesiredState(IntakeShooterState.kOff);
    }));

    Trigger climb = m_controller.back()
    .onTrue(new InstantCommand(() -> {
      m_climberSubsystem.setDesiredState(ClimberState.kRetract);
    }));
    Trigger unclimb = m_controller.start()
    .onTrue(new InstantCommand(() -> {
      m_climberSubsystem.setDesiredState(ClimberState.kExtend);
    }));
    climb.or(unclimb).negate()
    .onTrue(new InstantCommand(() -> {
      m_climberSubsystem.setDesiredState(ClimberState.kOff);
    }));

    m_controller.a()
    .onTrue(new InstantCommand(() -> {
      m_driveSubsystem.zeroGyro();
    }));

    m_controller.b()
    .onTrue(new InstantCommand(() -> {
      m_driveSubsystem.toggleFieldRelative();
    }));

    m_controller.x()
    .onTrue(new InstantCommand(() -> {
      m_driveSubsystem.togglePoseLocked();
    }));

    // Trigger rollerIntake = m_controller.back()
    // .onTrue(new InstantCommand(() -> {
    //   m_rollerSubsystem.setDesiredState(RollerState.kIntake);
    // }));
    // Trigger rollerSpit = m_controller.start()
    // .onTrue(new InstantCommand(() -> {
    //   m_rollerSubsystem.setDesiredState(RollerState.kSpit);
    // }));
    // rollerIntake.or(rollerSpit).negate()
    // .onTrue(new InstantCommand(() -> {
    //   m_rollerSubsystem.setDesiredState(RollerState.kOff);
    // }));
  }

  public void findStartingVisionPose()
  {
    m_driveSubsystem.findStartingVisionPose();
  }

  public Command getAutonomousCommand() 
  {
    return autoChooser.getSelected();
  }

  public Command getTeleopCommand() 
  {
    return m_driveSubsystem.driveCommand(
      () -> m_controller.getHID().getPOV() != -1 
        ? Constants.kDriveAdjustSpeed * Math.cos(Math.toRadians(-m_controller.getHID().getPOV())) 
        : -m_controller.getLeftY(), 
      () -> m_controller.getHID().getPOV() != -1 
        ? Constants.kDriveAdjustSpeed * Math.sin(Math.toRadians(-m_controller.getHID().getPOV())) 
        : -m_controller.getLeftX(), 
      () -> -m_controller.getRightX());
  }
}
