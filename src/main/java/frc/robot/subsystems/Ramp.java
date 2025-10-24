// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.RampConstants.kD;
import static frc.robot.Constants.RampConstants.kDefaultPosition;
import static frc.robot.Constants.RampConstants.kDropPosition;
import static frc.robot.Constants.RampConstants.kI;
import static frc.robot.Constants.RampConstants.kLiftPosition;
import static frc.robot.Constants.RampConstants.kMaxOutput;
import static frc.robot.Constants.RampConstants.kMinOutput;
import static frc.robot.Constants.RampConstants.kP;
import static frc.robot.Constants.RampConstants.kPulleyMotorId;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

    var error = motor.getEncoder().setPosition(0);
    if (!REVLibError.kOk.equals(error)) {
      System.out.format("ERROR setting inital ramp motor position: $s", error);
    }
    error = motor.configure(config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    if (!REVLibError.kOk.equals(error)) {
      System.out.format("ERROR configuring ramp motor: $s", error);
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Coral/motorAbsoluteEncoderPosition", motor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Coral/motorAbsoluteEncoderVelocity", motor.getAbsoluteEncoder().getVelocity());
    Logger.recordOutput("Coral/motorAppledOutput", motor.getAppliedOutput());
    Logger.recordOutput("Coral/motorOutputCurrent", motor.getOutputCurrent());
  }

  public void setPosition(double position) {
    var error = motor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    if (!REVLibError.kOk.equals(error)) {
      System.out.format("ERROR setting ramp motor position to %f: $s", position, error);
    }
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
