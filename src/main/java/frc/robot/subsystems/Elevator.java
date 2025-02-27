package frc.robot.subsystems;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase {
	private SparkMax leftMotor = new SparkMax(ElevatorConstants.kLeftMotorId, SparkMax.MotorType.kBrushless);
	private SparkMax rightMotor = new SparkMax(ElevatorConstants.kRightMotorId, SparkMax.MotorType.kBrushless);
	
	private SysIdRoutine sysidRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null, null),
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
                        .angularVelocity(RPM.of(leftMotor.getEncoder().getVelocity()));  
                },
                this
            )
    );
    private Encoder boreEncoder = new Encoder(ElevatorConstants.kEncoderChA, ElevatorConstants.kEncoderChB);

	public Elevator() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(SparkMaxConfig.IdleMode.kCoast);
        config.closedLoop
            .p(ElevatorConstants.kP)
            .d(ElevatorConstants.kD)
            .velocityFF(ElevatorConstants.kFF);

        config.encoder.positionConversionFactor(ElevatorConstants.kGearboxRatio);

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
		// g_pose->UpdateFromWheelPositions(GetWheelPositions());
		SmartDashboard.putNumber("Drivetrain/Motor/RPM", leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator/BoreEncoderValue", boreEncoder.getDistance());
        SmartDashboard.putNumber("Elevator/NeoEncoderValue", leftMotor.getEncoder().getPosition());

        if (Math.abs(boreEncoder.getDistance()) < 0.02) {
            leftMotor.getEncoder().setPosition(0);
        }
	}
 
    public Command setPosition(double turns) {
        return Commands.runOnce(() -> {
                leftMotor.getClosedLoopController()
                    .setReference(turns, SparkMax.ControlType.kPosition);
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