package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.AlgaeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
	private final SparkMax leftMotor =
		new SparkMax(kLeftMotorId, SparkMax.MotorType.kBrushless);
		
	private final SparkMax rightMotor =
		new SparkMax(kRightMotorId, SparkMax.MotorType.kBrushless);
	
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
	public void periodic() {
		Logger.recordOutput("Algae/leftOutput", leftMotor.getAppliedOutput());
		Logger.recordOutput("Algae/rightOutput", rightMotor.getAppliedOutput());
	}

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