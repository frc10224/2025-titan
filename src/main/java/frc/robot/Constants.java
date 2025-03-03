package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
	public static final class DrivetrainConstants {
		public static final double MAX_DRIVE_SPEED = 1;
		public static final double MAX_TURN_SPEED = 0.8;
		public static final double GEAR_RATIO = 1/5.71;
		public static final Distance WHEEL_RADIUS = Inches.of(2);
		public static final double KP = 0.0000026155;
		public static final double KD = 0;
		public static final double KFF = 0.11911;
		public static final double CONTROLLER_DEADZONE = 0.05;
		// Max power RPM
		public static final double MAX_RPM = 2900;
		// CAN bus IDs
		public static final int MOTORID_LB = 2;
		public static final int MOTORID_LF = 1;
		public static final int MOTORID_RF = 3;
		public static final int MOTORID_RB = 4;
		public static final Translation2d WHEELLOC_FL = new Translation2d(0.381, 11);
		public static final Translation2d WHEELLOC_FR = new Translation2d(0.381, 11);
		public static final Translation2d WHEELLOC_BL = new Translation2d(-0.381, 0.381);
		public static final Translation2d WHEELLOC_BR = new Translation2d(-0.381, -0.381);
		// for the L4 distance auto
		public static final int LASERCAN_ID = 11;
		public static final double RANGEDIST_P = 0.0001;
		public static final double RANGEDIST_D = 1;
		public static final Distance L4_DISTANCE = Centimeters.of(8);
	};

	public static final class PoseConstants { 
		// TODO: fix these!
		public static final Transform3d kBackCameraLocation = new Transform3d(
			new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
	}

	public static final class ElevatorConstants {
		public static final int LEFT_MOTOR_ID = 6;
		public static final int RIGHT_MOTOR_ID = 5;
		public static final double KP = 0.5; // 0.086188;
		public static final double KD = 0.92;
		public static final double KFF = 0.0869;   
		public static final double TOP_LIMIT_SPIN_COUNT = 89;
		public static final int ENCODER_CH_A = 8;
		public static final int ENCODER_CH_B = 9; 
		public static final double GEARBOX_RATIO = 1./20.;
		public static final double[] ELEVATOR_LEVELS = { 0, 0.65, 1.86, 3.78 };
		public static final double L3_ALGAE_HEIGHT = 3.05;
		public static final double L2_ALGAE_HEIGHT = 1.85;
	}

	public static final class CoralConstants {
		public static final int LEFT_MOTOR_ID = 7;
		public static final int RIGHT_MOTOR_ID = 8;
		public static final double KP = 0.0001;
		public static final double KD = 0;
		public static final double KFF = 0.0003;
		public static final int LASERCAN_ID = 9;
	}

	public static final class AlgaeConstants {
		public static final int LEFT_MOTOR_ID = 9;
		public static final int RIGHT_MOTOR_ID = 10;
	}
}

// vi: sw=4 ts=4 noet tw=80 cc=80