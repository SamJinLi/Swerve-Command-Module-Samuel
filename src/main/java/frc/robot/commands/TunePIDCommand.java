// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// import java.util.Map;

// import edu.wpi.first.math.controller.PIDController;
// // import edu.wpi.first.networktables.NetworkTableEntry;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
// import frc.robot.subsystems.TunePIDSubsystem;

// // import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// // import edu.wpi.first.wpilibj2.command.CommandBase;

// public class TunePIDCommand extends CommandBase {

//   private final PIDController pid;
//   private final SimpleWidget pWidget;
//   private final SimpleWidget iWidget;
//   private final SimpleWidget dWidget;

//   public TunePIDCommand(TunePIDSubsystem tunepidsubsystem, PIDController pid, ShuffleboardTab tab) {
//     this.pid = pid;
//     this.pWidget = tab.add("P Value", 0).withWidget(BuiltInWidgets.kNumberSlider);
//     this.iWidget = tab.add("I Value", 0).withWidget(BuiltInWidgets.kNumberSlider);
//     this.dWidget = tab.add("D Value", 0).withWidget(BuiltInWidgets.kNumberSlider);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double p = pWidget.getEntry().getDouble(0);
//     double i = iWidget.getEntry().getDouble(0);
//     double d = dWidget.getEntry().getDouble(0);
//     pid.setPID(p, i, d);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
