// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.kLowerDriveScale;
import static frc.robot.Constants.DrivetrainConstants.kLowerTurnScale;
import static frc.robot.Constants.DrivetrainConstants.kUpperDriveScale;
import static frc.robot.Constants.DrivetrainConstants.kUpperTurnScale;

import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
	public final Drivetrain drivetrain = new Drivetrain();
	public final Elevator elevator = new Elevator();
	public final Coral coral = new Coral(elevator);
	public final Algae algae = new Algae();
	public final Climb climb = new Climb();

	LoggedDashboardChooser<Command> autoPicker = new LoggedDashboardChooser<>("auto picker");

	Map<String, Command> autos = Map.of(
		".7 sec drive right stalk",
		Commands.sequence(
			elevator.setLevel(0),
			Commands.runEnd(
				() -> {
					// this seems to like to spit it out the back
					//coral.hold();
					drivetrain.setDriveVelocity(0.001, 0, 0, 0);
				},
				() -> drivetrain.setDriveVelocity(0, 0, 0, 0)
			).withTimeout(0.7),
			drivetrain.autoAlign(kLevel4BackOffset,
				kCoralRightOffset),
			elevator.setLevel(3),
			Commands.waitSeconds(2),
			coral.spit().withTimeout(1),
			elevator.setPosition(kAlgaeHeight1),
			Commands.waitSeconds(2)
		),
		"1.3 sec drive right stalk",
		Commands.sequence(
			elevator.setLevel(0),
			Commands.runEnd(
				() -> {
					// this seems to like to spit it out the back
					//coral.hold();
					drivetrain.setDriveVelocity(0.001, 0, 0, 0);
				},
				() -> drivetrain.setDriveVelocity(0, 0, 0, 0)
			).withTimeout(1.3),
			drivetrain.autoAlign(kLevel4BackOffset,
				kCoralRightOffset),
			elevator.setLevel(3),
			Commands.waitSeconds(2),
			coral.spit().withTimeout(1),
			elevator.setPosition(kAlgaeHeight2),
			drivetrain.autoAlign(kAlgaeBackOffset,
				0),
			coral.collect().withTimeout(1)
		)
	);

	CommandXboxController driver = new CommandXboxController(0);
	CommandXboxController operator = new CommandXboxController(1);

	public RobotContainer() {
		// set controller binds
		//driver.rightTrigger().whileTrue(drivetrain.aimAtTag());

		autos.forEach(autoPicker::addOption);

		operator.povLeft().whileTrue(algae.collect());
		operator.povRight().whileTrue(algae.spit());
		operator.povDown().onTrue(elevator.setPosition(kAlgaeHeight1));
		operator.povUp().onTrue(elevator.setPosition(kAlgaeHeight2));

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
		driver.rightBumper().whileTrue(drivetrain.autoAlign(kLevel4BackOffset,
			kCoralRightOffset));

		//driver.a().onTrue(climb.changePosition());
		driver.x().whileTrue(climb.pull());
		driver.y().whileTrue(climb.changePosition(-1));
		//driver.povLeft().onTrue(climb.releaseTray());
		driver.povRight().onTrue(climb.returnServo());
		driver.povUp().onTrue(climb.extend());
		//driver.povDown().whileTrue(climb.pull());
	}

	public Command getAutonomousCommand() {
		return autoPicker.get();
	}

	public String getAutoName() {
		return autoPicker.getSendableChooser().getSelected();
	}

	public Command getDriveCommand() {
		return drivetrain.ControllerDrive(
			() -> -driver.getLeftY(),
			() -> driver.getLeftX(),
			() -> driver.getRightX()
		);
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80
