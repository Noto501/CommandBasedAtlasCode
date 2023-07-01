// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.*;
//import frc.robot.Robot.mechMode;

public class CatzArmSubsystem extends SubsystemBase {
  /** Creates a new CatzArmSubsystem. */
  private WPI_TalonFX armMtr;

  private final int ARM_MC_ID = 20;

  private final double EXTEND_PWR  = 0.2;
  private final double RETRACT_PWR = -0.2;

  //Conversion factors

  //current limiting
  private SupplyCurrentLimitConfiguration armCurrentLimit;
  private final int     CURRENT_LIMIT_AMPS            = 55;
  private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
  private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  private final boolean ENABLE_CURRENT_LIMIT          = true;

  //gear ratio
  private final double VERSA_RATIO  = 7.0/1.0;

  private final double PUILEY_1      = 24.0;
  private final double PUILEY_2      = 18.0;
  private final double PUILEY_RATIO  = PUILEY_1 / PUILEY_2;
    
  private final double FINAL_RATIO   = VERSA_RATIO * PUILEY_RATIO;
  private final double FINAL_CIRCUMFERENCE = 3.54; 


  private final boolean LIMIT_SWITCH_IGNORED = false;
  private final boolean LIMIT_SWITCH_MONITORED = true;

  private final double CNTS_OVER_REV = 2048.0 / 1.0;

  private final double CNTS_PER_INCH_CONVERSION_FACTOR = CNTS_OVER_REV/FINAL_CIRCUMFERENCE;

  private final double POS_ENC_INCH_RETRACT = 0.0;
  private final double POS_ENC_INCH_EXTEND = 8.157;
  private final double POS_ENC_INCH_PICKUP = 4.157;

  private final double POS_ENC_CNTS_RETRACT  = 0.0+154;//POS_ENC_INCH_RETRACT * CNTS_PER_INCH_CONVERSION_FACTOR;
  private final double POS_ENC_CNTS_EXTEND  = 44000.0+154;//POS_ENC_INCH_EXTEND * CNTS_PER_INCH_CONVERSION_FACTOR;
  private final double POS_ENC_CNTS_PICKUP = 22000.0+154;//POS_ENC_INCH_PICKUP * CNTS_PER_INCH_CONVERSION_FACTOR;


  private boolean extendSwitchState = false;

  private int SWITCH_CLOSED = 1;

  private final double ARM_KP = 0.15;
  private final double ARM_KI = 0.0001;
  private final double ARM_KD = 0.0;

  private final double ARM_CLOSELOOP_ERROR = 3000;

  private final double MANUAL_CONTROL_PWR_OFF = 0.0;

  private boolean highExtendProcess = false;

  private double targetPosition = -999.0;
  private double currentPosition = -999.0;
  private double positionError = -999.0; 
  private double elevatorPosition = -999.0;

  private boolean armInPosition = false;
  private int numConsectSamples = 0;

  private final double ARM_POS_ERROR_THRESHOLD = 2700.0; //0.5 inches    previously 500 enc counts
  private final double NO_TARGET_POSITION = -999999.0;


  public CatzArmSubsystem() 
  {

    armMtr = new WPI_TalonFX(ARM_MC_ID);

    armMtr.configFactoryDefault();

    armMtr.setNeutralMode(NeutralMode.Brake);
    armMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armMtr.overrideLimitSwitchesEnable(LIMIT_SWITCH_MONITORED);

    armMtr.config_kP(0, ARM_KP);
    armMtr.config_kI(0, ARM_KI);
    armMtr.config_kD(0, ARM_KD);

    armCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

    armMtr.configSupplyCurrentLimit(armCurrentLimit);

    armMtr.configAllowableClosedloopError(0, ARM_CLOSELOOP_ERROR);

    armMtr.set(ControlMode.PercentOutput, MANUAL_CONTROL_PWR_OFF);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(highExtendProcess == true) {

    }

    currentPosition = armMtr.getSelectedSensorPosition();
    positionError = currentPosition - targetPosition;
    if  ((Math.abs(positionError) <= ARM_POS_ERROR_THRESHOLD) && targetPosition != NO_TARGET_POSITION) {

        targetPosition = NO_TARGET_POSITION;
        numConsectSamples++;
            if(numConsectSamples >= 10) {   
                armInPosition = true;
            }
    }
    else {
        numConsectSamples = 0;
    }
  }

  
  public void checkLimitSwitches() {

        if(armMtr.getSensorCollection().isRevLimitSwitchClosed() == SWITCH_CLOSED) 
        {
            armMtr.setSelectedSensorPosition(POS_ENC_CNTS_RETRACT);
            extendSwitchState = true;
        }
        else 
        {
            extendSwitchState = false;
        }


    }

    public void setArmPwr(double pwr)
    {        
        armMtr.set(ControlMode.PercentOutput, pwr);
    }

    public double getArmEncoder()
    {
        return armMtr.getSelectedSensorPosition();
    }

    public void smartDashboardARM()
    {
        SmartDashboard.putNumber("arm encoder position", armMtr.getSelectedSensorPosition());
    }
    public boolean isArmInPos()
    {
        return armInPosition;
    }

    public void armSetFullExtendPos()
    {
        armMtr.set(ControlMode.Position, POS_ENC_CNTS_EXTEND);
    }

    public void armSetRetractPos()
    {
        armMtr.set(ControlMode.Position, POS_ENC_CNTS_RETRACT);
    }

    public void armSetPickupPos()
    {
        armMtr.set(ControlMode.Position, POS_ENC_CNTS_PICKUP);
    }
}
