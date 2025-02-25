package frc.robot.subsystems;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Coral extends SubsystemBase {
	private SparkMax leftMotor = new SparkMax(CoralConstants.kLeftMotorId, MotorType.kBrushless);
	private SparkMax rightMotor = new SparkMax(CoralConstants.kRightMotorId, MotorType.kBrushless);
	
	private SysIdRoutine sysidRoutine;
    private LaserCan laser = new LaserCan(CoralConstants.kLaserCanId);

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
                    log.motor("elevator-Left")
                        .voltage(Volts.of(leftMotor.get() *
                                    RobotController.getBatteryVoltage()))
                        .angularPosition(Revolutions.of(leftMotor.getEncoder().getPosition()))
                        .angularVelocity(RevolutionsPerSecond.of(leftMotor.getEncoder().getVelocity()));  
                },
                this
            )
        );

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kCoast);
        config.closedLoop
            .p(ElevatorConstants.kP)
            .d(ElevatorConstants.kD)
            .velocityFF(ElevatorConstants.kFF);

        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimitEnabled(false);

        config.softLimit.reverseSoftLimit(0);
        //config.softLimit.ForwardSoftLimit(ElevatorConstants::kTopLimitSpinCount);

        leftMotor.configure(config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.softLimit.forwardSoftLimit(0);
        //config.softLimit.ReverseSoftLimit(-ElevatorConstants::kTopLimitSpinCount);

        config.follow(leftMotor, true);

        rightMotor.configure(config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);
    }

	@Override
	public void periodic() {
		// g_pose->UpdateFromWheelPositions(GetWheelPositions());
		/* SmartDashboard.putNumber("Drivetrain/Motor/Lf_RPS", motorLf.GetEncoderVelocity());
		SmartDashboard.putNumber("Drivetrain/Motor/Lb_RPS", motorLb.GetEncoderVelocity());
		SmartDashboard.putNumber("Drivetrain/Motor/Rf_RPS", motorRf.GetEncoderVelocity());
		SmartDashboard.putNumber("Drivetrain/Motor/Rb_RPS", motorRb.GetEncoderVelocity()); */
        laserDist = laser.getMeasurement().distance_mm;
	}

    void setVelocity(double rpm) {
        leftMotor.getClosedLoopController()
            .setReference(rpm, SparkMax.ControlType.kVelocity);
    }

    public Command collect() {
        return Commands.runEnd(
            () -> {
                if (laserDist > 10)
                    setVelocity(150);
                else
                    setVelocity(0);
            },
            () -> { setVelocity(0); }
        );
    }
    
    public Command spit() {
        return Commands.runEnd(
            () -> {
                if (laserDist < 10)
                    setVelocity(0);
                else
                    setVelocity(250);
            },
            () -> { setVelocity(0); }
        );
    }

	public Command sysIdDynamic(Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command sysIdQuasistatic(Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}