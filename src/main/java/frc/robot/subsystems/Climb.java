// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  // private double currentPos = 0;
  private SparkMax motor;

  /** Creates a new Winch. */
  public Climb() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    motor = new SparkMax(kWinchMotorId, MotorType.kBrushless);
    motor.configure(config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  /* private void setPosition(double pos) {
    motor.getClosedLoopController()
      .setReference(pos,
        ControlType.kPosition);
  } */

  /* public Command changePosition() {
    return Commands.runOnce(
      () -> {
        if (currentPos == 0) {
          currentPos = kHoldPosition;
        } else {
          currentPos = 0;
        }
        setPosition(currentPos);
      },
      this
    );
  } */

  public Command changePosition() {
    return Commands.runEnd(
      () -> {
        motor.set(0.1);
      },
      () -> {
        motor.set(0);
      },
      this
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
