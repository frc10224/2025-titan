// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.kLowerDriveScale;
import static frc.robot.Constants.DrivetrainConstants.kLowerTurnScale;
import static frc.robot.Constants.DrivetrainConstants.kUpperDriveScale;
import static frc.robot.Constants.DrivetrainConstants.kUpperTurnScale;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
	public final Drivetrain drivetrain = new Drivetrain();
	public final Elevator elevator = new Elevator();
	public final Coral coral = new Coral(elevator);
	public final Algae algae = new Algae();
	public final Climb climb = new Climb();

	CommandXboxController driver = new CommandXboxController(0);
	CommandXboxController operator = new CommandXboxController(1);

	public RobotContainer() {
		// set controller binds
		//driver.rightTrigger().whileTrue(drivetrain.aimAtTag());

		operator.povLeft().whileTrue(algae.collect());
		operator.povRight().whileTrue(algae.spit());
		operator.povDown().onTrue(elevator.setPosition(ElevatorConstants.kAlgaeHeight1));
		operator.povUp().onTrue(elevator.setPosition(ElevatorConstants.kAlgaeHeight2));

		operator.a().whileTrue(coral.collect());
		operator.b().whileTrue(coral.spit());
		driver.b().whileTrue(coral.spit());
		operator.x().whileTrue(coral.slurp());
		operator.y().whileTrue(elevator.zero());
		
		operator.rightTrigger().onTrue(elevator.setLevel(0));
		operator.rightBumper().onTrue(elevator.setLevel(1));
		operator.leftTrigger().onTrue(elevator.setLevel(2));
		operator.leftBumper().onTrue(elevator.setLevel(3));

		driver.rightTrigger().onTrue(drivetrain.setSpeedScale(kLowerDriveScale, kLowerTurnScale));
		driver.rightTrigger().onFalse(drivetrain.setSpeedScale(1, 1));
		driver.leftTrigger().onTrue(drivetrain.setSpeedScale(kUpperDriveScale, kUpperTurnScale));
		driver.leftTrigger().onFalse(drivetrain.setSpeedScale(1, 1));
		//driver.rightTrigger().whileTrue(drivetrain.aimAtTag());

		//driver.a().onTrue(climb.changePosition());
		driver.x().whileTrue(climb.changePosition(1));
		driver.y().whileTrue(climb.changePosition(-1));
	}

	public Command getAutonomousCommand() {
		return Commands.sequence(
			elevator.setLevel(0),
			Commands.runEnd(
				() -> {
					// this seems to like to spit it out the back
					//coral.hold();
					drivetrain.SetVelocity(0.001, 0, 0);
				},
				() -> drivetrain.SetVelocity(0, 0, 0)
			).withTimeout(2.1),
			elevator.setLevel(1),
			Commands.waitSeconds(4),
			coral.spit().withTimeout(3),
			Commands.waitSeconds(2),
			elevator.setLevel(0)
		);
	}

	public Command getDriveCommand() {
		return drivetrain.ControllerDrive(
			() -> { return -driver.getLeftY(); },
			() -> { return driver.getLeftX(); },
			() -> { return driver.getRightX(); }
		);
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80