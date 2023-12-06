// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.TunePIDSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunePIDSubsystem extends SubsystemBase {

  private final PIDController pid;
  private final SimpleWidget pWidget;
  private final SimpleWidget iWidget;
  private final SimpleWidget dWidget;

  public TunePIDSubsystem(PIDController pid, shu) {
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
