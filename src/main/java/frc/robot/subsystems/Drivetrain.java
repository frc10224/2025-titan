package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.DrivetrainConstants.*;

class DriveMotor {
		SparkMax motor;
		SparkClosedLoopController pid;
		SparkRelativeEncoder encoder;

		public DriveMotor(int id) {
			motor = new SparkMax(id, SparkMax.MotorType.kBrushless);
			pid = motor.getClosedLoopController();

			SparkMaxConfig config = new SparkMaxConfig();

			// PID should stop fine for us
			config.idleMode(SparkMaxConfig.IdleMode.kCoast);

			config.smartCurrentLimit(30);
			
			config.closedLoop.pidf(kP, 0, kD, kFF);
			
			config.encoder.positionConversionFactor(1);

			motor.configure(config,
				SparkMax.ResetMode.kResetSafeParameters,
				// if the controller power cycles we want it to remember these
				// settings
				SparkMax.PersistMode.kPersistParameters);
		}
		
		Distance getWheelDistance() {
			return kWheelRadius
				.times(kGearRatio)
				.times(motor.getEncoder().getPosition());
		}

		/// Set the target motor percentage, (attempting to) keep RPM constant
		void setVelocity(double percent) {
				pid.setReference(percent * kMaxRPM, ControlType.kVelocity);
		}

		/// Set the target motor voltage
		void setVoltage(Voltage voltage) {
				motor.setVoltage(voltage);
		}
		AngularVelocity getEncoderVelocity() {
				return RPM.of(encoder.getVelocity());
		}
		void log(SysIdRoutineLog log, String title) {
				log.motor(title)
					.voltage(Volts.of(motor.get() *
								RobotController.getBatteryVoltage()))
					.angularPosition(Revolutions.of(encoder.getPosition()))
					.angularVelocity(getEncoderVelocity());  
		}
};

public class Drivetrain extends SubsystemBase {
	private final DriveMotor motorLf = new DriveMotor(kMotorId_LF);
	private final DriveMotor motorLb = new DriveMotor(kMotorId_LB);
	private final DriveMotor motorRf = new DriveMotor(kMotorId_RF);
	private final DriveMotor motorRb = new DriveMotor(kMotorId_RB); 
	
	private final SysIdRoutine sysidRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(null, null, Seconds.of(3), null),
		new SysIdRoutine.Mechanism(
			(Voltage driveVoltage) -> {
				motorLf.setVoltage(driveVoltage.unaryMinus());
				motorLb.setVoltage(driveVoltage.unaryMinus());
				motorRf.setVoltage(driveVoltage);
				motorRb.setVoltage(driveVoltage);
			},
			(SysIdRoutineLog log) -> {
				motorLf.log(log, "drive-leftFront");
				motorLb.log(log, "drive-leftBack");
				motorRf.log(log, "drive-rightFront");
				motorRb.log(log, "drive-rightBack");
			},
			this
		)
	);
		
	public Drivetrain() {}

	@Override
	public void periodic() {
		// TODO: odometry updating stuff
	}

	public Command MecanumDrive(DoubleSupplier XSpeed, DoubleSupplier YSpeed,
			DoubleSupplier ZRotate) {
		return Commands.run(() -> {
			double xSpeed = XSpeed.getAsDouble();
			double ySpeed = YSpeed.getAsDouble();
			double zRotate = ZRotate.getAsDouble();

			// if we are in the deadzone, just set it to zero. otherwise,
			// pull the value down to start at zero
			//
			// for example, the range
			// 0.0 -----==================== 1.0
			//     dead     values we want
			//
			// becomes
			// 0.0 ==================== 0.85
		
			if (Math.abs(xSpeed) < kControllerDeadzone) xSpeed = 0;
			else xSpeed -= Math.signum(xSpeed) * kControllerDeadzone;

			if (Math.abs(ySpeed) < kControllerDeadzone) ySpeed = 0;
			else ySpeed -= Math.signum(ySpeed) * kControllerDeadzone;

			if (Math.abs(zRotate) < kControllerDeadzone) zRotate = 0;
			else zRotate -= Math.signum(zRotate) * kControllerDeadzone;
			
			// now that we are operating in that range we want from 0 to 
			// 1 - deadzone we can scale it back up so we are back to the
			// range 0 to 1
			xSpeed /= 1. - kControllerDeadzone;
			ySpeed /= 1. - kControllerDeadzone;
			zRotate /= 1. - kControllerDeadzone;

			// scale factors
			xSpeed *= kMaxDriveSpeed;
			ySpeed *= kMaxDriveSpeed;
			zRotate *= kMaxTurnSpeed;

			// curve
			xSpeed = Math.pow(xSpeed, 3);
			ySpeed = Math.pow(ySpeed, 3);
			zRotate = Math.pow(zRotate, 3);

			// we are omitting the gyro angle here because field relative
			// control on mecanum frankly is horrible
			MecanumDrive.WheelSpeeds ws =
				MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotate);

			motorLf.setVelocity(-ws.frontLeft * kMaxRPM);
			motorRf.setVelocity(ws.frontRight * kMaxRPM);
			motorLb.setVelocity(-ws.rearLeft * kMaxRPM);
			motorRb.setVelocity(ws.rearRight * kMaxRPM);
		}, this);
	}

	MecanumDriveWheelPositions GetWheelPositions() {
		return new MecanumDriveWheelPositions(
			motorLf.getWheelDistance(),
			motorRf.getWheelDistance(),
			motorLb.getWheelDistance(),
			motorRb.getWheelDistance()
		);
	}
	public Command SysIdDynamic(Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command SysIdQuasistatic(Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}