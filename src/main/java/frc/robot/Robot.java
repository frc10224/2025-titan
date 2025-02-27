  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;

  import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.CommandScheduler;

  public class Robot extends TimedRobot {
	// this gets populated by the dashboard picker ideally
	private Command autoCommand;
	private final RobotContainer container = new RobotContainer();
	private final Command teleopCommand = container.getTeleopCommand();

	public Robot() {}

	@Override
	public void robotPeriodic() {
	  // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
	  // commands, running already-scheduled commands, removing finished or interrupted commands,
	  // and running subsystem periodic() methods.  This must be called from the robot's periodic
	  // block in order for anything in the Command-based framework to work.
	  CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
	
		autoCommand = container.getAutonomousCommand();

		if (teleopCommand != null)
			teleopCommand.cancel();

		if (autoCommand != null)
			autoCommand.schedule();
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (autoCommand != null) autoCommand.cancel();

		if (teleopCommand != null) 
			teleopCommand.schedule();
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
