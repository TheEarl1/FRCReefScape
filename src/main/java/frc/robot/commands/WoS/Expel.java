
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WoS;

import frc.robot.Constants;
import frc.robot.subsystems.WoS;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;

public class Expel extends Command {
  WoS m_WoS;

  TDNumber m_RPM;
  TDNumber m_enablePID;

  TDNumber m_WoSSpeed;

  /** Creates a new Expel. */
  public Expel() {
    super(WoS.getInstance(), "WoS", "Expel");
    m_WoS = WoS.getInstance();

    m_RPM = new TDNumber(m_WoS, "WoS Speed (RPM)", "RPM", Constants.WoSConstants.kWoSSpeedRPM);
    m_enablePID = new TDNumber(m_WoS, "WoS Speed (RPM)", "Enable PID w 1");
    
    m_WoSSpeed = new TDNumber(m_WoS, "WoS Speed (Power)", "Speed", Constants.WoSConstants.kWoSSpeed);

    addRequirements(m_WoS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_enablePID.get() == 1) {
      m_WoS.setSpeeds(m_RPM.get(), true);
    }
    else {
      m_WoS.spinOut(m_WoSSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WoS.noSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

