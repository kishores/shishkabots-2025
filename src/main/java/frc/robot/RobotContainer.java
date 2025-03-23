// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorTestCommand;
import frc.robot.commands.EmergencyStopCommand;
import frc.robot.commands.FineTuneShooterIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.PrepareShooterCommand;
import frc.robot.commands.CalibrateElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  // The robot's subsystems
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(
        Constants.ElevatorConstants.ELEVATOR_PRIMARY_MOTOR_ID,
        Constants.ElevatorConstants.ELEVATOR_SECONDARY_MOTOR_ID,
        Constants.ElevatorConstants.ELEVATOR_TOP_LIMIT_SWITCH_ID,
        Constants.ElevatorConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH_ID
    );
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(
    Constants.ShooterConstants.SHOOTER_PRIMARY_MOTOR_ID,
    Constants.ShooterConstants.SHOOTER_SECONDARY_MOTOR_ID);

  // Commented out as Limelight is no longer used
  // private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // The driver's controllers
  // Primary controller (port 0) is for the main driver
  // Secondary controller (port 1) is for the operator/co-pilot
  // Both controllers have the same button mappings for redundancy
  private final XboxController driveController = new XboxController(0); // Primary controller on port 0
  private final XboxController mechanismController = new XboxController(1); // Secondary controller on port 1

  private static final double DEADBAND = 0.095;
  private static final double SHOOTER_DEADBAND = 0.06;
  
  // setup the AutoBuilder with all pathplanner paths in place
  private final SendableChooser<Command> autoChooser;


  // Commented out as Limelight is no longer used
  /*
  public LimelightSubsystem getLimelightSubsystem() {
    return limelightSubsystem;
  }
  */
  private double applyDeadband(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }
    return value;
  }

  private double getForwardInput() {
    // Use primary controller input, but if it's not moving, check secondary
    double primaryInput = -driveController.getLeftY();
    return applyDeadband(primaryInput);
  }

  private double getStrafeInput() {
    // Use primary controller input, but if it's not moving, check secondary
    double primaryInput = -driveController.getLeftX();
    return applyDeadband(primaryInput);
  }

  private double getRotationInput() {
    // Use primary controller input, but if it's not moving, check secondary
    double primaryInput = -driveController.getRightX();
    return applyDeadband(primaryInput);
  }

  // getShooterInput only works with the secondary controller. PORT 1
  private double getShooterInput() {
    double shooterInputSupplier = -mechanismController.getRightY();
    if (Math.abs(shooterInputSupplier) < SHOOTER_DEADBAND) {
      return 0;
    }
    return shooterInputSupplier * 0.4;
  }

  public RobotContainer() {
    configureBindings();
    // Set up the default command for the drive subsystem
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput(),  // Forward/backward
            () -> getStrafeInput(),   // Left/right
            () -> getRotationInput()  // Rotation
        )
    ); 
      shooterSubsystem.setDefaultCommand(
        new FineTuneShooterIntakeCommand(
        shooterSubsystem,
        () -> getShooterInput()
        )
      );
    autoChooser = AutoBuilder.buildAutoChooser("straight");

      // Register Named Commands for Auton Routines
    NamedCommands.registerCommand("shootBottomLevel", new ShootCommand(shooterSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("prepareShooter", new PrepareShooterCommand(shooterSubsystem));
  }
  private void configureBindings() {
    // Configure button bindings for both controllers
    // Both controllers have identical bindings for redundancy and flexibility
    // This allows either the driver or operator to control any function if needed
    
    // Primary Xbox Controller Bindings (port 0)

    // Emergency stop for all subsystems (Back + Start buttons together)
    new JoystickButton(driveController, XboxController.Button.kBack.value)
        .and(new JoystickButton(driveController, XboxController.Button.kStart.value))
        .onTrue(new EmergencyStopCommand(driveSubsystem, elevatorSubsystem, shooterSubsystem));

      // Slow driving mode for primary controller
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value) 
    .whileTrue(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * 0.45,
            () -> getStrafeInput() * 0.45,
            () -> getRotationInput() * 0.45
        )
    );

    // Secondary Xbox Controller Bindings (same as primary)
    new JoystickButton(mechanismController, XboxController.Button.kX.value)
        .onTrue(new ElevatorTestCommand(elevatorSubsystem, 0));
    new JoystickButton(mechanismController, XboxController.Button.kA.value)
        .onTrue(new ElevatorTestCommand(elevatorSubsystem, 1));
    new JoystickButton(mechanismController, XboxController.Button.kB.value)
        .onTrue(new ElevatorTestCommand(elevatorSubsystem, 2));
    new JoystickButton(mechanismController, XboxController.Button.kY.value)
        .onTrue(new ElevatorTestCommand(elevatorSubsystem, 3));
    
    new JoystickButton(mechanismController, XboxController.Button.kBack.value)
        .onTrue(new CalibrateElevatorCommand(elevatorSubsystem));
    
    new JoystickButton(mechanismController, XboxController.Button.kLeftBumper.value)
        .onTrue(new PrepareShooterCommand(shooterSubsystem));
    
    new JoystickButton(mechanismController, XboxController.Button.kRightBumper.value)
        .onTrue(new ShootCommand(shooterSubsystem, elevatorSubsystem));

    // Emergency stop for all subsystems (Back + Start buttons together) on secondary controller
    new JoystickButton(mechanismController, XboxController.Button.kBack.value)
        .and(new JoystickButton(mechanismController, XboxController.Button.kStart.value))
        .onTrue(new EmergencyStopCommand(driveSubsystem, elevatorSubsystem, shooterSubsystem));
  }
        
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create and return the autonomous command
    return autoChooser.getSelected();
  }
}