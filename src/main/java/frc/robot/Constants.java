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
    public static final int kLeftClimberId = 16;
    public static final int kRightClimberId = 12;
    public static final int kFrontShooterId = 13;
    public static final int kBackShooterId = 15;
    public static final int kTopIndexerId = 11;
    public static final int kBottomIndexerId = 10;
    public static final int kFloorIntakeId = 9;
    public static final int kRollerClawId = 14;

    public static final double kClimberDutyCycle = 0.75;
    public static final double kShooterLaunchDutyCycle = 1;
    public static final double kShooterIntakeDutyCycle = -0.5;
    public static final double kTopIndexerLaunchDutyCycle = 1;
    public static final double kTopIndexerIntakeDutyCycle = -0.6;
    public static final double kTopIndexerShootDelaySec = 2;
    public static final double kFloorIntakeDutyCycle = 0.3;
    public static final double kBottomIndexerDutyCycle = 0.6;
    public static final double kRollerClawDutyCycle = 0.2;
}
