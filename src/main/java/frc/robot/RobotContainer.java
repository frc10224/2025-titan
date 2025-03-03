// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
	private final Drivetrain drivetrain = new Drivetrain();
	private final Elevator elevator = new Elevator();
	private final Coral coral = new Coral();
	private final Algae algae = new Algae();

	CommandXboxController driver = new CommandXboxController(0);
	CommandXboxController operator = new CommandXboxController(1);

	public RobotContainer() {
		// set controller binds
		driver.rightTrigger().whileTrue(drivetrain.lineupL4());

		operator.povLeft().whileTrue(algae.collect());
		operator.povRight().whileTrue(algae.spit());
		operator.povDown().onTrue(elevator.setPosition(ElevatorConstants.L2_ALGAE_HEIGHT));
		operator.povUp().onTrue(elevator.setPosition(ElevatorConstants.L3_ALGAE_HEIGHT));

		operator.a().whileTrue(coral.collect());
		operator.b().whileTrue(coral.spit());
		operator.x().whileTrue(coral.slurp());
		operator.y().whileTrue(elevator.zero());
		
		operator.rightTrigger().onTrue(elevator.setLevel(0));
		operator.rightBumper().onTrue(elevator.setLevel(1));
		operator.leftTrigger().onTrue(elevator.setLevel(2));
		operator.leftBumper().onTrue(elevator.setLevel(3));
	}

	public Command getAutonomousCommand() {
		return drivetrain.ControllerDrive(
			() -> { return 1; },
			() -> { return 0; },
			() -> { return 0; }
		);
	}

	public Command getTeleopCommand() {
		return drivetrain.ControllerDrive(
			() -> { return -driver.getLeftY(); },
			() -> { return driver.getLeftX(); },
			() -> { return driver.getRightX(); }
		);
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80