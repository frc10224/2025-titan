package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.RobotController;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

class Motor {
		SparkMax motor;
		SparkClosedLoopController pid;
		SparkRelativeEncoder encoder;

		public Motor(int id) {
			motor = new SparkMax(id, MotorType.kBrushless);
			pid = motor.getClosedLoopController();

			SparkMaxConfig config = new SparkMaxConfig();

			// PID should stop fine for us
			config.idleMode(SparkMaxConfig.IdleMode.kCoast);

			config.smartCurrentLimit(30);
			
			config.closedLoop
				.p(DrivetrainConstants.kP)
				.i(DrivetrainConstants.kI)
				.d(DrivetrainConstants.kD)
				.velocityFF(DrivetrainConstants.kVelocityFF);
			
			config.encoder.positionConversionFactor(1);

			motor.configure(config,
				SparkMax.ResetMode.kResetSafeParameters,
				// if the controller power cycles we want it to remember these settings
				SparkMax.PersistMode.kPersistParameters);
		}
		
		Distance getWheelDistance() {
			return DrivetrainConstants.kWheelRadius
				.times(DrivetrainConstants.kGearRatio * motor.getEncoder().getPosition());
		}

		void setVelocity(double percent) {
				pid.setReference(percent * DrivetrainConstants.kMaxRPM, ControlType.kVelocity);
		}
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
	private Motor motorLf = new Motor(DrivetrainConstants.kMotorId_LF);
	private Motor motorLb = new Motor(DrivetrainConstants.kMotorId_LB);
	private Motor motorRf = new Motor(DrivetrainConstants.kMotorId_RF);
	private Motor motorRb = new Motor(DrivetrainConstants.kMotorId_RB); 
	
	private SysIdRoutine sysidRoutine;
		
	public Drivetrain() {
			sysidRoutine = new SysIdRoutine(
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
	}

	@Override
	public void periodic() {
		// g_pose->UpdateFromWheelPositions(GetWheelPositions());
		/* SmartDashboard.putNumber("Drivetrain/Motor/Lf_RPS", motorLf.GetEncoderVelocity());
		SmartDashboard.putNumber("Drivetrain/Motor/Lb_RPS", motorLb.GetEncoderVelocity());
		SmartDashboard.putNumber("Drivetrain/Motor/Rf_RPS", motorRf.GetEncoderVelocity());
		SmartDashboard.putNumber("Drivetrain/Motor/Rb_RPS", motorRb.GetEncoderVelocity()); */
	}

	public Command MecanumDrive(DoubleSupplier XSpeed, DoubleSupplier YSpeed, DoubleSupplier ZRotate) {
		return Commands.run(() -> {
			double xSpeed = XSpeed.getAsDouble();
			double ySpeed = YSpeed.getAsDouble();
			double zRotate = ZRotate.getAsDouble();
		
			if (Math.abs(xSpeed) < DrivetrainConstants.kControllerDeadzone)
				xSpeed = 0;
			else 
				xSpeed -= Math.signum(xSpeed) * DrivetrainConstants.kControllerDeadzone;

			if (Math.abs(ySpeed) < DrivetrainConstants.kControllerDeadzone) 
				ySpeed = 0;
			else
				ySpeed -= Math.signum(ySpeed) * DrivetrainConstants.kControllerDeadzone;

			if (Math.abs(zRotate) < DrivetrainConstants.kControllerDeadzone)
				zRotate = 0;
			else 
				zRotate -= Math.signum(zRotate) * DrivetrainConstants.kControllerDeadzone;

			xSpeed /= 1. - DrivetrainConstants.kControllerDeadzone;
			ySpeed /= 1. - DrivetrainConstants.kControllerDeadzone;
			zRotate /= 1. - DrivetrainConstants.kControllerDeadzone;

			xSpeed *= DrivetrainConstants.kMaxDriveSpeed;
			ySpeed *= DrivetrainConstants.kMaxDriveSpeed;
			zRotate *= DrivetrainConstants.kMaxTurnSpeed;

			xSpeed = Math.pow(xSpeed, 3);
			ySpeed = Math.pow(ySpeed, 3);
			zRotate = Math.pow(zRotate, 3);

			MecanumDrive.WheelSpeeds ws =
				MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotate/*, g_pose->GyroAngle()*/);

			motorLf.setVelocity(-ws.frontLeft * DrivetrainConstants.kMaxRPM);
			motorRf.setVelocity(ws.frontRight * DrivetrainConstants.kMaxRPM);
			motorLb.setVelocity(-ws.rearLeft * DrivetrainConstants.kMaxRPM);
			motorRb.setVelocity(ws.rearRight * DrivetrainConstants.kMaxRPM);
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