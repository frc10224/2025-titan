package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.ClimbConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
	private SparkMax motor = new SparkMax(kWinchMotorId, MotorType.kBrushless);;
	private SparkMax rampMotor = new SparkMax(kRampMotorId, MotorType.kBrushless);

	public Climb() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake);

		motor.configure(config,
			ResetMode.kResetSafeParameters,
			PersistMode.kPersistParameters);

		config.closedLoop.pid(0.3, 0, 0);
		rampMotor.configure(config,
			ResetMode.kResetSafeParameters,
			PersistMode.kPersistParameters);
	}

	public Command changePosition(int direction) {
		return Commands.runEnd(
			() -> motor.set(kSpinPercentage * direction),
			() -> motor.set(0),
			this
		);
	}

	public Command releaseTray() {
		return Commands.runOnce(() -> rampMotor
			.getClosedLoopController()
			.setReference(-0.25, ControlType.kPosition));
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Climb/RampMotorPosition", rampMotor.getEncoder().getPosition());
	}
}
