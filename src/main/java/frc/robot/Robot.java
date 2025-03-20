// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	// this gets populated by the dashboard picker ideally
	private final RobotContainer container = new RobotContainer();
	private final Command driveCommand = container.getDriveCommand();
	private final Command autoCommand = container.getAutonomousCommand();

	public Robot() {
		// don't complain about no joysticks, it's just annoying
		DriverStation.silenceJoystickConnectionWarning(true);

		Logger.recordMetadata("ProjectName", "2025-titan");
    	Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		if (isReal()) {
			Logger.addDataReceiver(new NT4Publisher());
			// new PowerDistribution(RobotConstants.kPDHCanId, ModuleType.kRev);
		} else {
			setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
		}

		Logger.start();
	}

	@Override
	public void robotInit() {}

	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		// Update pose stuff
		Pose.getInstance().periodicUpdate();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		container.drivetrain.removeDefaultCommand();;

		if (autoCommand != null)
			autoCommand.schedule();
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (autoCommand != null)
			autoCommand.cancel();

		container.drivetrain.setDefaultCommand(driveCommand);
	}

	@Override
	public void teleopPeriodic() {} 

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}
}

// vi: sw=4 ts=4 noet tw=80 cc=80