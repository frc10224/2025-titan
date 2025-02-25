package frc.robot.subsystems;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase {
	private SparkMax leftMotor = new SparkMax(ElevatorConstants.kLeftMotorId, SparkMax.MotorType.kBrushless);
	private SparkMax rightMotor = new SparkMax(ElevatorConstants.kRightMotorId, SparkMax.MotorType.kBrushless);
	
	private SysIdRoutine sysidRoutine;
		
	public Elevator() {
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

        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.softLimit.forwardSoftLimit(0);
        //config.softLimit.ReverseSoftLimit(-ElevatorConstants::kTopLimitSpinCount);

        config.follow(leftMotor, true);

        rightMotor.configure(config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);
    

        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimitEnabled(false);

        config.softLimit.reverseSoftLimit(0);
        //config.softLimit.ForwardSoftLimit(ElevatorConstants::kTopLimitSpinCount);

        leftMotor.configure(config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);
    }

	@Override
	public void periodic() {
		// g_pose->UpdateFromWheelPositions(GetWheelPositions());
		SmartDashboard.putNumber("Drivetrain/Motor/RPM", leftMotor.getEncoder().getVelocity());
	}
 
    public Command setPosition(double turns) {
        return Commands.startEnd(() -> {
                leftMotor.getClosedLoopController()
                    .setReference(turns + SmartDashboard.getNumber("Elevator/Compensation", 0), SparkMax.ControlType.kPosition);
            },
            () -> {
                // leftMotor.getClosedLoopController()
                //    .setReference(0, SparkMax.ControlType.kPosition);
                leftMotor.set(0);
            }
        );
    }

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysidRoutine.dynamic(direction);
	}
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysidRoutine.quasistatic(direction);
	}
}