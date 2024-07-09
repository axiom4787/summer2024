// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final int kLeftClimberId = 16; // neo
    public static final int kRightClimberId = 12; // neo
    public static final int kFrontShooterId = 13; // neo
    public static final int kBackShooterId = 15; // neo
    public static final int kTopIndexerId = 11; // redline
    public static final int kBottomIndexerId = 10; // neo 550
    public static final int kFloorIntakeId = 9; // neo
    public static final int kRollerClawId = 14; // neo 
    public static final int kBlinkinPort = 0; // blinkin/spark

    public static final double kClimberDutyCycle = 0.75;
    public static final double kShooterLaunchDutyCycle = 1;
    public static final double kShooterIntakeDutyCycle = -0.5;
    public static final double kShooterShootSpeed = 5.8; // meters per second of shooter WHEELS
    public static final double kShooterConversionFactor = 0.0762 * 1/60;
    public static final double kTopIndexerLaunchDutyCycle = 1;
    public static final double kTopIndexerIntakeDutyCycle = -0.3;
    public static final double kFloorIntakeDutyCycle = 0.6;
    public static final double kBottomIndexerDutyCycle = 1;
    public static final double kRollerClawDutyCycle = 0.2;
    public static final double kDrivingDeadband = 0.1;
    public static final double kTurningDeadband = 0.1;

    // slower speeds for fine adjustments of robot position
    public static final double kDriveAdjustSpeed = 0.1;
    public static final double kTurnAdjustSpeed = 0.1;
}
