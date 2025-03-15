package frc.robot.subsystems;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;

import static frc.robot.Constants.CoralConstants.*;

import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
	private SparkMax leftMotor =
		new SparkMax(kLeftMotorId, SparkMax.MotorType.kBrushless);
		
	private SparkMax rightMotor =
		new SparkMax(kRightMotorId, SparkMax.MotorType.kBrushless);
	
	private SysIdRoutine sysidRoutine;
	private LaserCan frontLaser = new LaserCan(kFrontLaserId);
	private LaserCan backLaser = new LaserCan(kBackLaserId);

	private int frontLaserDist = 9999999;
	private int backLaserDist = 9999999;
	private double setpoint = 0;

	private Elevator elevator;

	public Coral(Elevator ele) {
		elevator = ele;
		sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(null, null, Seconds.of(3), null),
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

		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(SparkMaxConfig.IdleMode.kBrake);
		config.closedLoop.pidf(kP, 0, kD, kFF);

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
		LaserCanInterface.Measurement frontMeasurement = frontLaser.getMeasurement();
		LaserCanInterface.Measurement backMeasurement = backLaser.getMeasurement();

		if (backMeasurement != null)
			backLaserDist = backMeasurement.distance_mm;
		if (frontMeasurement != null)
			frontLaserDist = frontMeasurement.distance_mm;
			
		Logger.recordOutput("Coral/frontDist", frontLaserDist);
		Logger.recordOutput("Coral/backDist", backLaserDist);
		Logger.recordOutput("Coral/wheelDist", leftMotor.getEncoder().getPosition());
		elevator.isLocked = backLaserDist < 50;
	}

	public void hold() {
		setpoint = leftMotor.getEncoder().getPosition();
		Logger.recordOutput("Coral/setpoint", setpoint);
		leftMotor.getClosedLoopController()
			.setReference(setpoint, SparkMax.ControlType.kPosition);
	}

	public Command collect() {
		return Commands.runEnd(() -> leftMotor.set(0.06), () -> hold(), this)
			.until(() -> frontLaserDist < 20);
	}

	public Command slurp() {
		return Commands.runEnd(
			() -> leftMotor.set(-0.05),
			() -> hold(),
			this
		);
	}
	
	public Command spit() {
		return Commands.runEnd(
			() -> {
				if (elevator.setpoint != ElevatorConstants.kElevatorLevels[3])
					leftMotor.set(0.1);
				else
					leftMotor.set(0.05);
			},
			() -> hold(),
			this
		);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80