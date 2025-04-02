package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.ClimbConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
	private SparkMax motor = new SparkMax(kWinchMotorId, MotorType.kBrushless);;
	private Servo servo = new Servo(0);
	private double servoPos = 1;

	public Climb() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake);

		//servo.setBoundsMicroseconds(1520, 1520, 1520/2, 0, 0);
		//servo.

		motor.configure(config,
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

	public Command extend() {
		return changePosition(-1)
			.until(() -> motor.getEncoder().getPosition() < kClimberOutValue)
			.andThen(this.releaseTray());
	}

	public Command pull() {
		return changePosition(1).until(() -> motor.getEncoder().getPosition() > kClimberUpValue);
	}

	public Command releaseTray() {
		return Commands.runOnce(() -> servoPos = 0.4);
	}

	public Command returnServo() {
		return Commands.runOnce(() -> servoPos = 1);
	}

	@Override
	public void periodic() {
		servo.setPosition(servoPos);
		//servo.setPulseTimeMicroseconds(1520);
		Logger.recordOutput("Climber/servo", servo.getPosition());
		Logger.recordOutput("Climber/motorPos", motor.getEncoder().getPosition());
	}
}
