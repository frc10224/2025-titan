package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
	public boolean doClosedLoop = true;
	public double currentLevel = 0;

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
		Logger.recordOutput("Elevator/CurrentLevel", currentLevel);

		// zero the neo encoder with the bore encoder, seems to help fix
		// weird drift issues
		// HACKHACK!
		if (Math.abs(boreEncoder.getDistance()) < 0.01) {
			leftMotor.getEncoder().setPosition(0);
		}

		if (doClosedLoop) {
			// run the closed loop
			setpoint = trapezoidProfile.calculate(0.02, setpoint, goal);
			double ff = elevatorFeedforward.calculateWithVelocities(leftMotor.getEncoder().getVelocity(), setpoint.velocity);
			leftMotor.getClosedLoopController()
				.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
			Logger.recordOutput("Elevator/Feedforward", ff);
		}
	}

	public Command setPosition(double turns) {
		return Commands.runOnce(() -> {
				if (isLocked) return;
				/*for (int i = 0; i < kElevatorLevels.length; i++) {
					double pos = kElevatorLevels[i];
					if (turns == pos) {
						currentLevel = i;
					
						break;
					}
				}
				if (turns == kAlgaeHeight1) {
					currentLevel = 1.5;
				}
				if (turns == kAlgaeHeight2) {
					currentLevel = 2.5;
				}*/
				goal.position = turns;
				goal.velocity = 0;
			}, this);
	}

	public Command zero() {
		return Commands.startEnd(
			() -> {
				setLevel(0);
				Commands.waitSeconds(0.5);
				leftMotor.set(-0.05);
				doClosedLoop = false;
			},
			() -> {
				doClosedLoop = true;
				leftMotor.set(0);
				leftMotor.getEncoder().setPosition(0);
				boreEncoder.reset();
			},
			this
		);
	}

	public Command adjustHeight(double turns) {
		return Commands.runOnce(() -> {
			goal.position += turns;
		}, this);
	}

	public Command setLevel(int level) {
		return setPosition(kElevatorLevels[level]);
	}

	public Command changeLevel(int direction) {
		return Commands.runOnce(() -> {
			int shiftedLevel = (int) Math.round(currentLevel + (direction * 0.75));
			setLevel(shiftedLevel);
		}, this);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}
