// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.Constants;
import frc.robot.testingdashboard.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.testingdashboard.TDNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lucifer extends Command {
  Elevator m_Elevator;

  TDNumber m_RPM;
  TDNumber m_enablePID;
  
  TDNumber m_ElevatorSpeed;

  /** Creates a new Lucifer. */
  public Lucifer() {
    super(Elevator.getInstance(), "Elevator", "Lucifer");
    m_Elevator = Elevator.getInstance();

    m_RPM = new TDNumber(m_Elevator, "Elevator Speed (RPM)", "RPM", Constants.ElevatorConstants.kElevatorSpeedRPM);
    m_enablePID = new TDNumber(m_Elevator, "Elevator Speed (RPM)", "Enable PID w 1");

    m_ElevatorSpeed = new TDNumber(m_Elevator, "Elevator Speed (Power)", "Speed", Constants.ElevatorConstants.kElevatorSpeed);

    addRequirements(m_Elevator);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_enablePID.get() == 1) {
      m_Elevator.setSpeeds(m_RPM.get(), true);
    }
    else {
      m_Elevator.down(m_ElevatorSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.stop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}