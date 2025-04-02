package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
	public static final class RobotConstants {
		public static final int kPDHCanId = 63;
	}

	public static final class DrivetrainConstants {
		public static final double kMaxDriveSpeed = 1;
		public static final double kMaxTurnSpeed = 0.8;
		public static final double kGearRatio = 1/5.71;
		public static final Distance kWheelRadius = Inches.of(2 * 1.11);
		public static final double kP = 0.0000026155;
		public static final double kD = 0;
		public static final double kFF = 0.11911;
		public static final double kControllerDeadzone = 0.05;
		public static final double kLowerDriveScale = 0.001;
		public static final double kLowerTurnScale = 0.0005;
		public static final double kUpperDriveScale = 1.5;
		public static final double kUpperTurnScale = 1;
		// Max power RPM
		public static final double kMaxRPM = 2900;
		// CAN bus IDs
		public static final int kMotorId_LB = 2;
		public static final int kMotorId_LF = 1;
		public static final int kMotorId_RF = 3;
		public static final int kMotorId_RB = 4;
		// translations for motors
		public static final Translation2d kFrontLeftLocation = new Translation2d(Inches.of(14), Inches.of(10.5));
		public static final Translation2d kFrontRightLocation = new Translation2d(Inches.of(14), Inches.of(-10.5));
		public static final Translation2d kBackLeftLocation = new Translation2d(Inches.of(-14), Inches.of(10.5));
		public static final Translation2d kBackRightLocation = new Translation2d(Inches.of(-14), Inches.of(-10.5));
		public static final double kYawP = 0.003;
		public static final double kYawD = 0.0001;
		public static final double kDriveP = 0.0023;
		public static final double kDriveD = 0.001;
		public static final double kStrafeP = 0.004;
		public static final double kStrafeD = 0.001;
	};

	public static final class PoseConstants {
		// +X is forward, +Y is left, +Z is up
		// This is the transform FROM the camera TO the origin
		public static final Transform3d kFrontCameraLocation = new Transform3d(
			new Translation3d(Inches.of(7.08), Inches.of(-10), Inches.of(-25.75)),
			new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(13.3 + 180.0))
		);
		public static final double kVisPositionStdev = 0.02;
		public static final double kVisYawStdev = 0;
	}

	public static final class ElevatorConstants {
		public static final int kLeftMotorId = 6;
		public static final int kRightMotorId = 5;
		public static final double kP = 1.7; // 0.88599; // 0.086188;
		public static final double kD = 0; // 8.9956;
		public static final double kS = 0.15895;
		public static final double kG = 0.44337;
		public static final double kV = 0.12518;
		public static final double kMaxVelRPS = 12;
		public static final double kMaxAccelRPSPS = 22;
		public static final double kTopLimitSpinCount = 89;
		public static final int kEncoderChA = 8;
		public static final int kEncoderChB = 9;
		public static final double kGearboxRatio = 1./20.;
		public static final double[] kElevatorLevels = { 0, 0.5, 1.82, 4.4 };
		public static final double kAlgaeHeight2 = 3.5;
		public static final double kAlgaeHeight1 = 2.2;
	}

	public static final class CoralConstants {
		public static final int kLeftMotorId = 7;
		public static final int kRightMotorId = 8;
		public static final double kP = 0.9;
		public static final double kD = 0;
		public static final double kFF = 0;
		public static final int kFrontLaserId = 11;
		public static final int kBackLaserId = 12;
	}

	public static final class AlgaeConstants {
		public static final int kLeftMotorId = 10;
		public static final int kRightMotorId = 9;
	}

	public static final class ClimbConstants {
		public static final int kWinchMotorId = 13;
		public static final double kHoldPosition = 6;
		public static final double kSpinPercentage = 1;
		public static final int kRampMotorId = 21;
		public static final double kClimberOutValue = -266;
		public static final double kClimberUpValue = 160;
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80
