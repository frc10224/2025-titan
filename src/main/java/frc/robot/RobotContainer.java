// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.*;

public class RobotContainer {
	private final Drivetrain drivetrain = new Drivetrain();
	private final Elevator elevator = new Elevator();
	private final Coral coral = new Coral();

	CommandXboxController driver = new CommandXboxController(0);

	public RobotContainer() {
		// set controller binds
		driver.a().whileTrue(coral.collect());
		driver.x().whileTrue(elevator.setPosition(20));
	}

	public Command getAutonomousCommand() {
		return null;
	}

	public Command getTeleopCommand() {
		return drivetrain.MecanumDrive(
			() -> { return driver.getLeftX(); },
			() -> { return driver.getLeftY(); },
			() -> { return driver.getRightX(); }
		);
	}
}
