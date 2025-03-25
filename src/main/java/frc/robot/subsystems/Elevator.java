package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
	private final SparkMax leftMotor =
		new SparkMax(kLeftMotorId, SparkMax.MotorType.kBrushless);
	private final SparkMax rightMotor =
		new SparkMax(kRightMotorId, SparkMax.MotorType.kBrushless);

	private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV);
	private final TrapezoidProfile trapezoidProfile =
		new TrapezoidProfile(new Constraints(kMaxVelRPS, kMaxAccelRPSPS));

	private TrapezoidProfile.State goal = new TrapezoidProfile.State();
	private TrapezoidProfile.State setpoint =
		new TrapezoidProfile.State(leftMotor.getEncoder().getPosition(), leftMotor.getEncoder().getVelocity());

	public boolean isLocked = false;
	
	private SysIdRoutine sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(7), null, null),
			new SysIdRoutine.Mechanism(
				(Voltage driveVoltage) -> {
					leftMotor.setVoltage(driveVoltage);
					rightMotor.setVoltage(driveVoltage.unaryMinus());
				},
				(SysIdRoutineLog log) -> {
					RelativeEncoder enc = leftMotor.getEncoder();
					log.motor("elevator-Left")
						.voltage(Volts.of(leftMotor.getAppliedOutput() *
									RobotController.getBatteryVoltage()))
						.angularPosition(Revolutions.of(enc.getPosition()))
						.angularVelocity(RevolutionsPerSecond.of(enc.getVelocity()));  
				},
				this
			)
	);
	private Encoder boreEncoder = new Encoder(kEncoderChA, kEncoderChB);
	
	public Elevator() {
		SparkMaxConfig config = new SparkMaxConfig();

		config.idleMode(SparkMaxConfig.IdleMode.kBrake);
		config.closedLoop.pid(kP, 0, kD);
		config.encoder.positionConversionFactor(kGearboxRatio);
		config.encoder.velocityConversionFactor(1./60.);

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
		Logger.recordOutput("Elevator/Setpoint", setpoint.position);
		Logger.recordOutput("Elevator/IsLocked", isLocked);
		// zero the neo encoder with the bore encoder, seems to help fix
		// weird drift issues
		if (Math.abs(boreEncoder.getDistance()) < 0.02) {
			leftMotor.getEncoder().setPosition(0);
		}

		// run the closed loop
		setpoint = trapezoidProfile.calculate(0.05, setpoint, goal);
		double ff = elevatorFeedforward.calculate(setpoint.velocity);
		leftMotor.getClosedLoopController()
			.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
		Logger.recordOutput("Elevator/Feedforward", ff);
	}

	public Command setPosition(double turns) {
		return Commands.runOnce(() -> {
				if (isLocked) return;
				goal.position = turns;
				goal.velocity = 0;
			}
		);
	}

	public Command zero() {
		return Commands.startEnd(
			() -> { leftMotor.set(-0.05); },
			() -> {
				leftMotor.set(0);
				leftMotor.getEncoder().setPosition(0);
				boreEncoder.reset();
			}
		);
	}

	public Command adjustHeight(double turns) {
		return Commands.runOnce(() -> {
			goal.position += turns;
		});
	}

	public Command setLevel(int level) {
		return setPosition(kElevatorLevels[level]);
	}

	public double setpoint() {
		return goal.position;
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}