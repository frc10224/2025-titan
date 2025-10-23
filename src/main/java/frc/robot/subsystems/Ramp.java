// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.RampConstants.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ramp extends SubsystemBase {
  private SparkMax motor = new SparkMax(kPulleyMotorId, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig(); 

  /** Creates a new Ramp. */
  public Ramp() {
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid(kP, kI, kD);
    config.closedLoop.minOutput(kMinOutput);
    config.closedLoop.maxOutput(kMaxOutput);

    motor.getEncoder().setPosition(0);
    motor.configure(config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double rotations) {
    System.out.println("posistion = " + rotations);
    motor.getClosedLoopController().setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public Command lift() {
    return Commands.startEnd(
      () -> {
        setPosition(kLiftPosition);
      },
      () -> {
        setPosition(kDefaultPosition);
      },
      this
    );
  }

  public Command drop() {
    return Commands.run(
      () -> {
        // config.closedLoop.minOutput(-0.15);
        // motor.configure(config,
        //   ResetMode.kResetSafeParameters,
        //   PersistMode.kPersistParameters
        // );

        setPosition(kDropPosition);
      },
      this
    );
  }
}
