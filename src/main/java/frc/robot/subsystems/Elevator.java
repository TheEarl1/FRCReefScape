// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;

public class Elevator extends SubsystemBase {
  private static Elevator m_Elevator;

  TDNumber m_TDelevatorP;
  TDNumber m_TDelevatorI;
  TDNumber m_TDelevatorD;
  double m_elevatorP = Constants.ElevatorConstants.kElevatorP;
  double m_elevatorI = Constants.ElevatorConstants.kElevatorI;
  double m_elevatorD = Constants.ElevatorConstants.kElevatorD;
  TDNumber m_encoderValueRotations;

  SparkMax m_ELeftSparkMax;
  SparkMax m_ERightSparkMax;

  TDNumber m_leftCurrentOutput;
  TDNumber m_rightCurrentOutput;

  AbsoluteEncoder m_AbsoluteEncoder;
  /** Creates a new Elevator. */
  private Elevator() {
    super("Elevator");

    if (RobotMap.E_ENABLED) {
      m_ELeftSparkMax = new SparkMax(RobotMap.E_LEFTMOTOR, MotorType.kBrushless);
      m_ERightSparkMax = new SparkMax(RobotMap.E_RIGHTMOTOR, MotorType.kBrushless);

      SparkMaxConfig leftElevatorSparkMaxConfig = new SparkMaxConfig();
      SparkMaxConfig rightElevatorSparkMaxConfig = new SparkMaxConfig();

      rightElevatorSparkMaxConfig.follow(m_ELeftSparkMax, true);

      m_TDelevatorP = new TDNumber(this, "Elevator PID", "P", Constants.ElevatorConstants.kElevatorP);
      m_TDelevatorI = new TDNumber(this, "Elevator PID", "I", Constants.ElevatorConstants.kElevatorI);
      m_TDelevatorD = new TDNumber(this, "Elevator PID", "D", Constants.ElevatorConstants.kElevatorD);

      leftElevatorSparkMaxConfig.closedLoop.pid(Constants.ElevatorConstants.kElevatorP, Constants.ElevatorConstants.kElevatorI,
          Constants.ElevatorConstants.kElevatorD);
      m_ELeftSparkMax.configure(leftElevatorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      rightElevatorSparkMaxConfig.closedLoop.pid(Constants.ElevatorConstants.kElevatorP, Constants.ElevatorConstants.kElevatorI,
      Constants.ElevatorConstants.kElevatorD);
      m_ERightSparkMax.configure(rightElevatorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_AbsoluteEncoder = m_ELeftSparkMax.getAbsoluteEncoder();

      m_leftCurrentOutput = new TDNumber(this, "Elevator", "Left Motor Current");
      m_rightCurrentOutput = new TDNumber(this, "Elevator", "Right Motor Current");
    
    } 
  }

  public static Elevator getInstance() {
    if (m_Elevator == null) {
      m_Elevator = new Elevator();
    }
    return m_Elevator;
  }

  public void setSpeeds(double RPM, boolean backwards) {
    if (!backwards) {
      m_ELeftSparkMax.getClosedLoopController().setReference(RPM, ControlType.kVelocity);
    } else {
      m_ELeftSparkMax.getClosedLoopController().setReference(-RPM, ControlType.kVelocity);
    }
  }

  public void up(double speed) {
    if (m_ELeftSparkMax != null) {
      m_ELeftSparkMax.set(speed);
    }
  }

  public void down(double speed) {
    if (m_ELeftSparkMax != null) {
      m_ELeftSparkMax.set(-speed);
    }
  }

  public void stop(double speed) {
    if (m_ELeftSparkMax != null) {
      m_ELeftSparkMax.set(0);
    }
  }

  @Override
  public void periodic() {
    if (Constants.ElevatorConstants.kEnableElevatorPIDTuning &&
        m_ELeftSparkMax != null) {
      SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
      double tmp = m_TDelevatorP.get();
      if (tmp != m_elevatorP) {
        m_elevatorP = tmp;
      }
      sparkMaxConfig.closedLoop.p(m_elevatorP);
      tmp = m_TDelevatorI.get();
      if (tmp != m_elevatorI) {
        m_elevatorI = tmp;
      }
      sparkMaxConfig.closedLoop.i(m_elevatorI);
      tmp = m_TDelevatorD.get();
      if (tmp != m_elevatorD) {
        m_elevatorD = tmp;
      }
      sparkMaxConfig.closedLoop.d(m_elevatorD);
      m_ELeftSparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    if (RobotMap.W_ENABLED) {
      m_leftCurrentOutput.set(m_ELeftSparkMax.getEncoder().getVelocity());
      m_rightCurrentOutput.set(m_ERightSparkMax.getEncoder().getVelocity());
    }

    super.periodic();
  }
}
