package frc.robot.subsystems;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;

import static frc.robot.Constants.CoralConstants.*;

// TODO: fix the issue where the coral falls out of the coral box. i reckon
// this may involve doing something where when we stop it we use pid to set
// a **position setpoint** so that it does not move, and if it does it will
// move the coral back into the box

public class Coral extends SubsystemBase {
	private SparkMax leftMotor =
		new SparkMax(kLeftMotorId, SparkMax.MotorType.kBrushless);
		
	private SparkMax rightMotor =
		new SparkMax(kRightMotorId, SparkMax.MotorType.kBrushless);
	
	private SysIdRoutine sysidRoutine;
	private LaserCan laser = new LaserCan(kLaserCanId);

	private int laserDist = 9999999; 

	public Coral() {
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

		rightMotor.configure(config,
			SparkMax.ResetMode.kResetSafeParameters,
			SparkMax.PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		laserDist = laser.getMeasurement().distance_mm;
		SmartDashboard.putNumber("Coral/LaserDist", laserDist);
	}

	void setVelocity(double lRPM, double rRPM) {
		leftMotor.getClosedLoopController()
			.setReference(lRPM, SparkMax.ControlType.kVelocity);
		rightMotor.getClosedLoopController()
			.setReference(-rRPM, SparkMax.ControlType.kVelocity);
	}

	public Command collect() {
		return Commands.runEnd(
			() -> {
				if (laserDist > 10) setVelocity(150, 150);
				else setVelocity(-0.1, -0.1);
			},
			() -> { setVelocity(-0.1, -0.1); }
		);
	}

	public Command slurp() {
		return Commands.runEnd(
			() -> { setVelocity(-100, -100); },
			() -> { setVelocity(-0.5, -0.5); }
		);
	}
	
	public Command spit() {
		return Commands.runEnd(
			() -> {
				if (laserDist < 10) setVelocity(250, 250);
				else setVelocity(0, 0);
			},
			() -> { setVelocity(0, 0); }
		);
	}

	public Command spitFastOneSide() {
		return Commands.runEnd(
			() -> {
				if (laserDist < 10) setVelocity(250, 80);
				else setVelocity(0, 0);
			},
			() -> { setVelocity(0, 0); }
		);
	}

	public Command sysIdDynamic(Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command sysIdQuasistatic(Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80