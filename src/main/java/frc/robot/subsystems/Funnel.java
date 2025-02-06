// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;

public class Funnel extends SubsystemBase {
  private static Funnel m_Funnel;

  TDNumber m_TDwheelP;
  TDNumber m_TDwheelI;
  TDNumber m_TDwheelD;
  double m_wheelP = Constants.FunnelConstants.kFunnelP;
  double m_wheelI = Constants.FunnelConstants.kFunnelI;
  double m_wheelD = Constants.FunnelConstants.kFunnelD;

  SparkMax m_FLeftSparkMax;
  SparkMax m_FRightSparkMax;

  SparkMaxConfig m_SparkMaxConfig;

  TDNumber m_FunnelCurrentOutput;

  /** Creates a new Funnel. */
  public Funnel() {
    super("Funnel");

    if (RobotMap.F_ENABLED) {
      m_FLeftSparkMax = new SparkMax(RobotMap.F_LEFTMOTOR, MotorType.kBrushless);
      m_FRightSparkMax = new SparkMax(RobotMap.F_RIGHTMOTOR, MotorType.kBrushless);

      SparkMaxConfig LeftFunnelSparkMaxConfig = new SparkMaxConfig();
      SparkMaxConfig RightFunnelSparkMaxConfig = new SparkMaxConfig();

      RightFunnelSparkMaxConfig.follow(m_FLeftSparkMax, true);

      m_TDwheelP = new TDNumber(this, "Wheel PID", "P", Constants.FunnelConstants.kFunnelP);
      m_TDwheelI = new TDNumber(this, "Wheel PID", "I", Constants.FunnelConstants.kFunnelI);
      m_TDwheelD = new TDNumber(this, "Wheel PID", "D", Constants.FunnelConstants.kFunnelD);
      
      LeftFunnelSparkMaxConfig.closedLoop.pid(Constants.FunnelConstants.kFunnelP, Constants.FunnelConstants.kFunnelI, 
          Constants.FunnelConstants.kFunnelD);
      m_FLeftSparkMax.configure(LeftFunnelSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_FunnelCurrentOutput = new TDNumber(this, "Funnel", "Motor Current");
    }
  }

  public static Funnel getInstance() {
    if (m_Funnel == null) {
      m_Funnel = new Funnel();
    }
    return m_Funnel;
  }

  public void setSpeeds(double RPM, boolean backwards) {
    if (!backwards) {
      m_FLeftSparkMax.getClosedLoopController().setReference(RPM, ControlType.kVelocity);
    } else {
      m_FLeftSparkMax.getClosedLoopController().setReference(-RPM, ControlType.kVelocity);
    }
  }

  public void spinIn(double speed) {
    if (m_FLeftSparkMax != null) {
      m_FLeftSparkMax.set(speed);
    }
  }

  public void spinOut(double speed) {
    if (m_FLeftSparkMax != null) {
      m_FLeftSparkMax.set(-speed);
    }
  }

  public void noSpin(double speed) {
    if (m_FLeftSparkMax != null) {
      m_FLeftSparkMax.set(0);
    }
  }

  @Override
  public void periodic() {
    if (Constants.FunnelConstants.kEnableFunnelPIDTuning &&
        m_FLeftSparkMax != null) {
      double tmp = m_TDwheelP.get();
      boolean changed = false;
      if (tmp != m_wheelP) {
        m_wheelP = tmp;
        m_SparkMaxConfig.closedLoop.p(m_wheelP);
        changed = true;
      }
      tmp = m_TDwheelI.get();
      if (tmp != m_wheelI) {
        m_wheelI = tmp;
        changed = true;
        m_SparkMaxConfig.closedLoop.i(m_wheelI);
      }
      tmp = m_TDwheelD.get();
      if (tmp != m_wheelD) {
        m_wheelD = tmp;
        changed = true;
        m_SparkMaxConfig.closedLoop.d(m_wheelD);
      }
      if(changed) {
        m_FLeftSparkMax.configure(m_SparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (RobotMap.E_ENABLED) {
      m_FunnelCurrentOutput.set(m_FLeftSparkMax.getEncoder().getVelocity());
    }

    super.periodic();
  }
}
