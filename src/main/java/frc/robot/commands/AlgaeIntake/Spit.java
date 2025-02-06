// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake;

import frc.robot.Constants;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.subsystems.AlgaeIntake;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Spit extends Command {
  AlgaeIntake m_algaeIntake;

  TDNumber m_RPM;
  TDNumber m_enablePID;

  TDNumber m_rollerSpeed;

  /** Creates a new Consume. */
  public Spit() {
    super(AlgaeIntake.getInstance(),"AlgaeIntake","Fast");
    m_algaeIntake = AlgaeIntake.getInstance();

    m_RPM = new TDNumber(m_algaeIntake, "Roller Speed (RPM)", "RPM", Constants.AlgaeIntakeConstants.kRollerSpeedRPM);
    m_enablePID = new TDNumber(m_algaeIntake, "Roller Speed (RPM)", "Enable PID w 1");

    m_rollerSpeed = new TDNumber(m_algaeIntake, "Roller Speed (Power)", "Speed", Constants.AlgaeIntakeConstants.kAngleSpeed);

    addRequirements(m_algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_enablePID.get() == 1) {
      m_algaeIntake.setRollerSpeeds(m_RPM.get(), false);
    }
    else {
      m_algaeIntake.spinRollerIn(m_rollerSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeIntake.stopRollerSpin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
