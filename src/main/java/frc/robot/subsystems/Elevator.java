package frc.robot.subsystems;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
	private final SparkMax leftMotor =
		new SparkMax(kLeftMotorId, SparkMax.MotorType.kBrushless);
	private final SparkMax rightMotor =
		new SparkMax(kRightMotorId, SparkMax.MotorType.kBrushless);

	public double setpoint = 0;

	public boolean isLocked = false;
	
	private SysIdRoutine sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(null, Volts.of(4), null, null),
			new SysIdRoutine.Mechanism(
				(Voltage driveVoltage) -> {
					leftMotor.setVoltage(driveVoltage);
					rightMotor.setVoltage(driveVoltage.unaryMinus());
				},
				(SysIdRoutineLog log) -> {
					RelativeEncoder enc = leftMotor.getEncoder();
					log.motor("elevator-Left")
						.voltage(Volts.of(leftMotor.get() *
									RobotController.getBatteryVoltage()))
						.angularPosition(Revolutions.of(enc.getPosition()))
						.angularVelocity(RPM.of(enc.getVelocity()));  
				},
				this
			)
	);
	private Encoder boreEncoder = new Encoder(kEncoderChA, kEncoderChB);
	
	public Elevator() {
		SparkMaxConfig config = new SparkMaxConfig();

		config.idleMode(SparkMaxConfig.IdleMode.kCoast);
		config.closedLoop.pidf(kP, 0, kD, kFF);
		config.encoder.positionConversionFactor(kGearboxRatio);

		config.smartCurrentLimit(50);

		leftMotor.configure(config,
			SparkMax.ResetMode.kResetSafeParameters,
			SparkMax.PersistMode.kPersistParameters);

		config.follow(leftMotor, true);

		rightMotor.configure(config,
			SparkMax.ResetMode.kResetSafeParameters,
			SparkMax.PersistMode.kPersistParameters);
	
		// rev bore encoder
		boreEncoder.setDistancePerPulse(1./2048.);
		boreEncoder.setSamplesToAverage(5);
		boreEncoder.setReverseDirection(true);
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Elevator/BoreEncoder", boreEncoder.getDistance());
		Logger.recordOutput("Elevator/NeoEncoder", leftMotor.getEncoder().getPosition());
		Logger.recordOutput("Elevator/Velocity", leftMotor.getEncoder().getVelocity());
		Logger.recordOutput("Elevator/Current", leftMotor.getOutputCurrent());
		Logger.recordOutput("Elevator/RightCurrent", rightMotor.getOutputCurrent());
		Logger.recordOutput("Elevator/Setpoint", setpoint);
		Logger.recordOutput("Elevator/IsLocked", isLocked);
		// zero the neo encoder with the bore encoder, seems to help fix
		// weird drift issues
		if (Math.abs(boreEncoder.getDistance()) < 0.02) {
			leftMotor.getEncoder().setPosition(0);
		}
	}

	public Command setPosition(double turns) {
		return Commands.runOnce(() -> {
				if (isLocked) return;
				setpoint = turns;
				leftMotor.getClosedLoopController()
					.setReference(turns, SparkMax.ControlType.kPosition);
			},
			this
		);
	}

	public Command zero() {
		return Commands.startEnd(
			() -> { leftMotor.set(-0.05); },
			() -> {
				leftMotor.set(0);
				leftMotor.getEncoder().setPosition(0);
				boreEncoder.reset();
			},
			this
		);
	}

	public Command adjustHeight(double turns) {
		return Commands.runOnce(() -> {
			setpoint += turns;
			leftMotor.getClosedLoopController()
				.setReference(setpoint, SparkMax.ControlType.kPosition);
			},
			this
		);
	}

	public Command setLevel(int level) {
		return setPosition(kElevatorLevels[level]);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}