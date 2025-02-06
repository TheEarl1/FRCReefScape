// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.funnel;

import frc.robot.Constants;
import frc.robot.subsystems.Funnel;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Explode extends Command {
  Funnel m_Funnel;

  TDNumber m_RPM;
  TDNumber m_enablePID;

  TDNumber m_FunnelSpeed;

  /** Creates a new implode. */
  public Explode() {
    super(Funnel.getInstance(),"Funnel","Explode");
    m_Funnel = Funnel.getInstance();

    m_RPM = new TDNumber(m_Funnel, "Funnel Speed (RPM)", "RPM", Constants.FunnelConstants.kFunnelSpeedRPM);
    m_enablePID = new TDNumber(m_Funnel, "Funnel Speed (RPM)", "Enable PID w 1");

    m_FunnelSpeed = new TDNumber(m_Funnel, "Funnel Speed (Power)", "Speed", Constants.FunnelConstants.kFunnelSpeed);

    addRequirements(m_Funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_enablePID.get() == 1) {
      m_Funnel.setSpeeds(m_RPM.get(), true);
    }
    else {
      m_Funnel.spinOut(m_FunnelSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Funnel.noSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
