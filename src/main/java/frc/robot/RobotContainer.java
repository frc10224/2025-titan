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
		// XXX: should this be on an operator controller?
		driver.a().whileTrue(coral.collect());
		driver.b().whileTrue(coral.spit());
		driver.x().whileTrue(coral.slurp());
		driver.rightTrigger().onTrue(elevator.setLevel(0));
		driver.rightBumper().onTrue(elevator.setLevel(1));
		driver.leftTrigger().onTrue(elevator.setLevel(2));
		driver.leftBumper().onTrue(elevator.setLevel(3));
	}

	public Command getAutonomousCommand() {
		return null;
	}

	public Command getTeleopCommand() {
		return drivetrain.MecanumDrive(
			() -> { return -driver.getLeftY(); },
			() -> { return driver.getLeftX(); },
			() -> { return driver.getRightX(); }
		);
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80