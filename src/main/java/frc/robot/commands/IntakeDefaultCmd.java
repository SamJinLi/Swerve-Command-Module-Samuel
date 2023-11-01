// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
// import frc.robot.Constants.WristConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCmd extends CommandBase {
    private final Supplier<Boolean> trigger, thumbButton;
    private final IntakeSubsystem intakeSubsystem;
    int lastGamePiece;
    static final int CONE = 1;
    static final int CUBE = 2;
  /** Creates a new IntakeDefaultCmd. */
  public IntakeDefaultCmd(IntakeSubsystem intakeSubsystem, Supplier<Boolean> trigger, Supplier<Boolean> thumbButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.trigger = trigger;
    this.thumbButton = thumbButton;

    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakePower = 0.0;
        if (trigger.get() && thumbButton.get()) {
            // cube in
            intakePower = IntakeConstants.k_CUBE_INTAKE_SPEED;
            lastGamePiece = CUBE;
        }
        else if (trigger.get())
        {
            // CONE out aka full power
            intakePower = IntakeConstants.INTAKE_OUTPUT_POWER;
            lastGamePiece = CUBE;
        }
        else if (thumbButton.get()) {
            // cone in or cube out
            intakePower = -IntakeConstants.INTAKE_OUTPUT_POWER;
            lastGamePiece = CONE;
        } else if (lastGamePiece == CUBE) {
            intakePower = IntakeConstants.INTAKE_HOLD_POWER;
        } else if (lastGamePiece == CONE) {
            intakePower = -IntakeConstants.INTAKE_HOLD_POWER;
        } else {
            intakePower = 0.0;
        }
        intakeSubsystem.setIntakeMotor(intakePower);
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
