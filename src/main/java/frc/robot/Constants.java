package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
	public static final class DrivetrainConstants {
		public static final double kMaxDriveSpeed = 0.6;
		public static final double kMaxTurnSpeed = 0.1;
		public static final double kGearRatio = 1/5.71;
		public static final Distance kWheelRadius = Inches.of(2);
		public static final double kP = 0.0000026155;
		public static final double kD = 0;
		public static final double kFF = 0.11911;
		public static final double kControllerDeadzone = 0.02;
		// Max power RPM
		public static final double kMaxRPM = 2900;
		// CAN bus IDs
		public static final int kMotorId_LB = 2;
		public static final int kMotorId_LF = 1;
		public static final int kMotorId_RF = 3;
		public static final int kMotorId_RB = 4;
		public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 11);
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, 11);
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
		public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);
	};

	public static final class PoseConstants { 
		// TODO: fix these!
		public static final Transform3d kBackCameraLocation = new Transform3d(
			new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
	}

	public static final class ElevatorConstants {
		public static final int kLeftMotorId = 6;
		public static final int kRightMotorId = 5;
		public static final double kP = 0.5; // 0.086188;
		public static final double kD = 0.92;
		public static final double kFF = 0.0869;   
		public static final double kTopLimitSpinCount = 89;
		public static final int kEncoderChA = 8;
		public static final int kEncoderChB = 9; 
		public static final double kGearboxRatio = 1./20.;
		public static final double[] kElevatorLevels = { 0, 0.4, 1.65, 3.75 };
	}

	public static final class CoralConstants {
		public static final int kLeftMotorId = 7;
		public static final int kRightMotorId = 8;
		public static final double kP = 0.0001;
		public static final double kD = 0;
		public static final double kFF = 0.0003;
		public static final int kLaserCanId = 9;
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80