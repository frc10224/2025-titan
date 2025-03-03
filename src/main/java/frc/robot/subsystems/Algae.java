package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.AlgaeConstants.*;

// TODO: fix the issue where the coral falls out of the coral box. i reckon
// this may involve doing something where when we stop it we use pid to set
// a **position setpoint** so that it does not move, and if it does it will
// move the coral back into the box

public class Algae extends SubsystemBase {
	private final SparkMax leftMotor =
		new SparkMax(LEFT_MOTOR_ID, SparkMax.MotorType.kBrushless);
		
	private final SparkMax rightMotor =
		new SparkMax(RIGHT_MOTOR_ID, SparkMax.MotorType.kBrushless);
	
	public Algae() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(SparkMaxConfig.IdleMode.kBrake);

		leftMotor.configure(config,
			SparkMax.ResetMode.kResetSafeParameters,
			SparkMax.PersistMode.kPersistParameters);

		config.follow(leftMotor, true);

		rightMotor.configure(config,
			SparkMax.ResetMode.kResetSafeParameters,
			SparkMax.PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {}

	public Command collect() {
		return Commands.runEnd(
			() -> { leftMotor.set(0.3);},
			() -> { leftMotor.set(0); }
		);
	}
	
	public Command spit() {
		return Commands.runEnd(
			() -> { leftMotor.set(-0.3); },
			() -> { leftMotor.set(0); }
		);
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80