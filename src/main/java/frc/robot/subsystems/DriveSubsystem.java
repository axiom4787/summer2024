// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase 
{
  /** Creates a new DriveSubsystem. */
  private final double maximumSpeed = 5.71;
  private SwerveDrive swerveDrive;
  private Field2d m_field;
  private boolean m_fieldRelative, m_poseLocked;
    
  public DriveSubsystem() 
  {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try 
    { 
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } 
    catch (IOException e) 
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(true);
    m_field = new Field2d();
    m_fieldRelative = true;
    m_poseLocked = false;
    double p = Constants.test;
    SmartDashboard.putData(m_field);

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
                    5.74, // Max module speed, in m/s
                    0.4131, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public Pose2d getPose() 
  {
    return swerveDrive.getPose();
  }
  
  public void resetPose(Pose2d startingPose) 
  {
    swerveDrive.resetOdometry(startingPose);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public void togglePoseLocked()
  {
    m_poseLocked = !m_poseLocked;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() 
  {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) 
  {
    System.out.println(robotRelativeSpeeds.omegaRadiansPerSecond);
    swerveDrive.drive(robotRelativeSpeeds);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotation) 
  {
    return run(() -> {
      if (m_poseLocked)
        swerveDrive.lockPose();
      else {
//        System.out.printf("angular velocity: %.2f\n", MathUtil.applyDeadband(angularRotation.getAsDouble() * swerveDrive.getMaximumAngularVelocity(), Constants.kTurningDeadband));
        swerveDrive.drive(
          new Translation2d(
            MathUtil.applyDeadband(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(), Constants.kDrivingDeadband),
            MathUtil.applyDeadband(translationY.getAsDouble() * swerveDrive.getMaximumVelocity(), Constants.kDrivingDeadband)),
          MathUtil.applyDeadband(angularRotation.getAsDouble() * swerveDrive.getMaximumAngularVelocity(), Constants.kTurningDeadband),
          m_fieldRelative,
          false);
      }
    });
  }

  private Pose2d getVisionPose()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    double[] values = table.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
  }

  public void toggleFieldRelative()
  {
    m_fieldRelative = !m_fieldRelative;
  }

  public void findStartingVisionPose()
  {
    Pose2d visionPose = getVisionPose();
    if (visionPose.getX() != 0.0 && visionPose.getY() != 0.0)
      swerveDrive.resetOdometry(visionPose);    
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    Pose2d visionPose = getVisionPose();
    if (visionPose.getX() != 0.0 && visionPose.getY() != 0.0)
      swerveDrive.addVisionMeasurement(getVisionPose(), Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Pose X", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("Pose Y", swerveDrive.getPose().getY());  
    SmartDashboard.putNumber("Pose Theta Degrees", swerveDrive.getPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Field Relative", m_fieldRelative);
    SmartDashboard.putBoolean("Pose Locked", m_poseLocked);
    m_field.setRobotPose(getPose());

  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }

}
