// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.AlgaeIntakeSensor;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class AlgaeIntake extends SubsystemBase {
  private static AlgaeIntake m_algaeIntake;

  TDNumber m_targetAngle;
  TDNumber m_angleEncoderValueRotations;
  TDNumber m_angleEncoderValueDegrees;
  TDNumber m_angleCurrentOutput;
  TDNumber m_TDangleP;
  TDNumber m_TDangleI;
  TDNumber m_TDangleD;
  double m_angleP = Constants.AlgaeIntakeConstants.kAngleP;
  double m_angleI = Constants.AlgaeIntakeConstants.kAngleI;
  double m_angleD = Constants.AlgaeIntakeConstants.kAngleD;
  private double m_lastAngle = 0;

  TDNumber m_TDrollerP;
  TDNumber m_TDrollerI;
  TDNumber m_TDrollerD;
  double m_rollerP = Constants.AlgaeIntakeConstants.kRollerP;
  double m_rollerI = Constants.AlgaeIntakeConstants.kRollerI;
  double m_rollerD = Constants.AlgaeIntakeConstants.kRollerD;

  SparkMax m_angleSparkMax;
  SparkMaxConfig m_angleSparkMaxConfig;
  SparkAbsoluteEncoder m_angleAbsoluteEncoder;
  SparkClosedLoopController m_angleClosedLoopController;
  
  SparkMax m_rollerSparkMax;
  SparkMaxConfig m_rollerSparkMaxConfig;

  AlgaeIntakeSensor m_algaeIntakeSensor;

  TDNumber m_rollerCurrentOutput;

  /** Creates a new ExampleSubsystem. */
  private AlgaeIntake() {
    super("AlgaeIntake");

    if (RobotMap.A_ENABLED) {
      // setup pivot on algae intake mechanism
      m_angleSparkMax = new SparkMax(RobotMap.A_ANGLEMOTOR, MotorType.kBrushless);
      m_angleSparkMaxConfig = new SparkMaxConfig();

      m_TDangleP = new TDNumber(this, "AlgaeIntake Angle PID", "P", Constants.AlgaeIntakeConstants.kAngleP);
      m_TDangleI = new TDNumber(this, "AlgaeIntake Angle PID", "I", Constants.AlgaeIntakeConstants.kAngleI);
      m_TDangleD = new TDNumber(this, "AlgaeIntake Angle PID", "D", Constants.AlgaeIntakeConstants.kAngleD);

      m_angleSparkMaxConfig.idleMode(IdleMode.kBrake);

      m_angleSparkMaxConfig.closedLoop.pid(Constants.AlgaeIntakeConstants.kAngleP, Constants.AlgaeIntakeConstants.kAngleI,
          Constants.AlgaeIntakeConstants.kAngleD);
      m_angleSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      m_angleSparkMaxConfig.closedLoop.positionWrappingEnabled(true);
      m_angleSparkMaxConfig.closedLoop.positionWrappingMinInput(0);
      m_angleSparkMaxConfig.closedLoop.positionWrappingMaxInput(Constants.AlgaeIntakeConstants.DEGREES_PER_REVOLUTION);

      m_angleSparkMaxConfig.absoluteEncoder.positionConversionFactor(Constants.AlgaeIntakeConstants.kAngleEncoderPositionFactor);
      m_angleSparkMaxConfig.absoluteEncoder.inverted(false);

      m_angleSparkMax.configure(m_angleSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_angleAbsoluteEncoder = m_angleSparkMax.getAbsoluteEncoder();

      m_targetAngle = new TDNumber(this, "AlgaeIntake Encoder Values", "Target Angle", getAngle());
      m_angleEncoderValueRotations = new TDNumber(this, "AlgaeIntake Encoder Values", "Rotations", getAngle() / Constants.AlgaeIntakeConstants.kAngleEncoderPositionFactor);
      m_angleEncoderValueDegrees = new TDNumber(this, "AlgaeIntake Encoder Values", "Angle (degrees)", getAngle());
      m_angleCurrentOutput = new TDNumber(Drive.getInstance(), "Current", "AlgaeIntake Angle Output", m_angleSparkMax.getOutputCurrent());
      
      // setup spinner to grab algae
      m_rollerSparkMax = new SparkMax(RobotMap.A_ROLLERMOTOR, MotorType.kBrushless);
      m_rollerSparkMaxConfig = new SparkMaxConfig();

      m_TDrollerP = new TDNumber(this, "Roller PID", "P", Constants.AlgaeIntakeConstants.kRollerP);
      m_TDrollerI = new TDNumber(this, "Roller PID", "I", Constants.AlgaeIntakeConstants.kRollerI);
      m_TDrollerD = new TDNumber(this, "Roller PID", "D", Constants.AlgaeIntakeConstants.kRollerD);

      m_rollerSparkMaxConfig.closedLoop.pid(Constants.AlgaeIntakeConstants.kRollerP, Constants.AlgaeIntakeConstants.kRollerI,
          Constants.AlgaeIntakeConstants.kRollerD);
      m_rollerSparkMax.configure(m_rollerSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_rollerCurrentOutput = new TDNumber(this, "WoS", "Motor Current");

      m_algaeIntakeSensor = new AlgaeIntakeSensor(RobotMap.A_INTAKESENSOR, m_algaeIntake);
    }
  }

  public static AlgaeIntake getInstance() {
    if (m_algaeIntake == null) {
      m_algaeIntake = new AlgaeIntake();
    }
    return m_algaeIntake;
  }

  public double getAngle() {
    return m_angleAbsoluteEncoder.getPosition();
  }

  public void setTargetAngle(double angle) {
    double setpoint = angle % Constants.AlgaeIntakeConstants.DEGREES_PER_REVOLUTION;
    setpoint = MathUtil.clamp(setpoint,
                              Constants.AlgaeIntakeConstants.kAngleLowerLimitDegrees, 
                              Constants.AlgaeIntakeConstants.kAngleUpperLimitDegrees);
    if (setpoint != m_lastAngle) {
      m_targetAngle.set(setpoint);
      m_lastAngle = setpoint;
      m_angleClosedLoopController.setReference(setpoint, ControlType.kPosition);
    }
  }

  public void setRollerSpeeds(double RPM, boolean backwards) {
    if (!backwards) {
      m_rollerSparkMax.getClosedLoopController().setReference(RPM, ControlType.kVelocity);
    } else {
      m_rollerSparkMax.getClosedLoopController().setReference(-RPM, ControlType.kVelocity);
    }
  }

  public void spinRollerIn(double speed) {
    if (m_rollerSparkMax != null) {
      m_rollerSparkMax.set(speed);
    }
  }

  public void spinRollerOut(double speed) {
    if (m_rollerSparkMax != null) {
      m_rollerSparkMax.set(-speed);
    }
  }

  public void stopRollerSpin() {
    if (m_rollerSparkMax != null) {
      m_rollerSparkMax.set(0);
    }
  }

  public boolean seesAlgae() {
    return m_algaeIntakeSensor.seesAlgae();
  }

  @Override
  public void periodic() {
    if (Constants.AlgaeIntakeConstants.kEnableRollerPIDTuning && m_rollerSparkMax != null) {
      double tmp = m_TDrollerP.get();
      boolean changed = false;
      if (tmp != m_rollerP) {
        m_rollerP = tmp;
        m_rollerSparkMaxConfig.closedLoop.p(m_rollerP);
        changed = true;
      }
      tmp = m_TDrollerI.get();
      if (tmp != m_rollerI) {
        m_rollerI = tmp;
        changed = true;
        m_rollerSparkMaxConfig.closedLoop.i(m_rollerI);
      }
      tmp = m_TDrollerD.get();
      if (tmp != m_rollerD) {
        m_rollerD = tmp;
        changed = true;
        m_rollerSparkMaxConfig.closedLoop.d(m_rollerD);
      }
      if(changed) {
        m_rollerSparkMax.configure(m_rollerSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (Constants.AlgaeIntakeConstants.kEnableAnglePIDTuning && m_angleSparkMax != null) {
      double tmp = m_TDangleP.get();
      boolean changed = false;
      if (tmp != m_angleP) {
        m_angleP = tmp;
        m_angleSparkMaxConfig.closedLoop.p(m_angleP);
        changed = true;
      }
      tmp = m_TDangleI.get();
      if (tmp != m_angleI) {
        m_angleI = tmp;
        changed = true;
        m_angleSparkMaxConfig.closedLoop.i(m_angleI);
      }
      tmp = m_TDangleD.get();
      if (tmp != m_angleD) {
        m_angleD = tmp;
        changed = true;
        m_angleSparkMaxConfig.closedLoop.d(m_angleD);
      }
      if(changed) {
        m_angleSparkMax.configure(m_angleSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (RobotMap.A_ENABLED) {
      m_rollerCurrentOutput.set(m_rollerSparkMax.getEncoder().getVelocity());
      m_angleCurrentOutput.set(m_angleSparkMax.getOutputCurrent());
      m_angleEncoderValueDegrees.set(getAngle()/Constants.AlgaeIntakeConstants.kAngleEncoderPositionFactor);
      m_angleEncoderValueRotations.set(getAngle());
    }

    super.periodic();
  }
}
