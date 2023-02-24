// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.SetNeutralModeCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class RobotContainer {

  // Initialize subsystems
  DrivetrainSubsystem drivetrainSubsystem;
  CommandXboxController controller;

  private final double DEADZONE = 0.25;

  public RobotContainer() {
    // Initialize controller and set bindings
    controller = new CommandXboxController(0);
    configureBindings();

    // Initialize subsystems
    drivetrainSubsystem = new DrivetrainSubsystem();

    drivetrainSubsystem.setDefaultCommand(
        new DriveCommand(() -> deadzoneModifier(controller.getLeftY(), DEADZONE),
            () -> deadzoneModifier(controller.getRightX(), DEADZONE), drivetrainSubsystem));
  }

  private void configureBindings() {
    controller.a().onTrue(new SetNeutralModeCommand(NeutralMode.Brake, drivetrainSubsystem));
    controller.y().onTrue(new SetNeutralModeCommand(NeutralMode.Coast, drivetrainSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * Apply deadzone to joystick axes
   * @param rawValue Raw joystick axis value in [-1.0, 1.0]
   * @param deadzone Deadzone as a percentage in [0, 1.0]
   * @return Joystick axis with deadzone applied
   */
  private static double deadzoneModifier(double rawValue, double deadzone) {
    double result;
    double validRange = 1.0 - deadzone;
    double value = Math.abs(rawValue);

    if (value > deadzone) {
      result = (value - deadzone) / validRange;
    } else {
      result = 0;
    }

    return rawValue < 0 ? -result : result;
  }
}
