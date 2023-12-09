// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import java.util.ArrayList;
import java.util.List;

public class TuningPIDConstantsSubsystem extends SubsystemBase {
  
  private final ShuffleboardTab pidTab = Shuffleboard.getTab("Tune PID");
  private final List<double[]> pidConstants = new ArrayList<>();
  private final List<SimpleWidget> pWidgets = new ArrayList<>();
  private final List<SimpleWidget> iWidgets = new ArrayList<>();
  private final List<SimpleWidget> dWidgets = new ArrayList<>();

  public void addNewPIDConstants(double[] constants, String name) {
    pidConstants.add(constants);
    ShuffleboardLayout pidLayout = pidTab.getLayout(name, BuiltInLayouts.kList);
    pWidgets.add(pidLayout.add("P Value", constants[0]).withWidget(BuiltInWidgets.kTextView));
    iWidgets.add(pidLayout.add("I Value", constants[1]).withWidget(BuiltInWidgets.kTextView));
    dWidgets.add(pidLayout.add("D Value", constants[2]).withWidget(BuiltInWidgets.kTextView));
  }

  public void updatePIDValues() {
    for (int i = 0; i < pidConstants.size(); i++) {
      pidConstants.get(i)[0] = pWidgets.get(i).getEntry().getDouble(pidConstants.get(i)[0]);
      pidConstants.get(i)[1] = iWidgets.get(i).getEntry().getDouble(pidConstants.get(i)[1]);
      pidConstants.get(i)[2] = dWidgets.get(i).getEntry().getDouble(pidConstants.get(i)[2]);
    }
  }
  
  public TuningPIDConstantsSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
