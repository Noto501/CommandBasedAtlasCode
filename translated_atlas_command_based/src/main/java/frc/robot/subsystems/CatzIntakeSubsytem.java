// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;


public class CatzIntakeSubsytem extends SubsystemBase {
    //----------------------------------------------------------------------------------------------
    //
    //  Roller
    //
    //----------------------------------------------------------------------------------------------
    private WPI_TalonFX rollersMtr;

    private final int ROLLERS_MC_ID  = 30;

    private final double ROLLERS_PWR_CUBE_IN = -0.8;   
    private final double ROLLERS_PWR_CONE_IN =  1.0; //TBD decide pwrs for all cube cone scoring rollers

    private final double ROLLERS_PWR_CUBE_OUT =  1.0;   
    private final double ROLLERS_PWR_CONE_OUT = -0.5;

    private SupplyCurrentLimitConfiguration rollerCurrentLimit;

    private final int     CURRENT_LIMIT_AMPS_ROLLER            = 40;    
    private final int     CURRENT_LIMIT_TRIGGER_AMPS_ROLLER    = 40;

    //----------------------------------------------------------------------------------------------
    //
    //  Wrist
    //
    //----------------------------------------------------------------------------------------------
    private WPI_TalonFX wristMtr;

    private final int    WRIST_MC_ID   = 31;

    private final double WRIST_MAX_PWR = 0.3;

    //current limiting
    private SupplyCurrentLimitConfiguration wristCurrentLimit;

    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;
    

    //----------------------------------------------------------------------------------------------
    //  Wrist encoder & Position Values
    //----------------------------------------------------------------------------------------------
    private final int    WRIST_ENC_CAN_ID = 13; 


    private final double ENC_TO_INTAKE_GEAR_RATIO =  46.0/18.0;
    private final double WRIST_CNTS_PER_DEGREE    = 46.459; //(4096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;


    private final double MANUAL_HOLD_STEP_SIZE = 1.5;       

    //TBD - ADD comment for ref point
    //Reference Point = wrist would be slight above "Parallel to the ground"
    private final double CENTER_OF_MASS_OFFSET_DEG     = 177.0; 
    private final double WRIST_ABS_ENC_OFFSET_DEG = 0.0; //Set to make stow pos equal to 0
    private final double WRIST_ABS_ENC_OFFSET = WRIST_ABS_ENC_OFFSET_DEG * WRIST_CNTS_PER_DEGREE;//-989.0; //Negative value means abs enc 0 is above intake angle 0   
    
    private final double STOW_ENC_POS               =  0.0 + WRIST_ABS_ENC_OFFSET_DEG;//4872.0 + WRIST_ABS_ENC_OFFSET; //3883
    private final double STOW_CUTOFF                =  -7.232 + WRIST_ABS_ENC_OFFSET_DEG;// + WRIST_ABS_ENC_OFFSET; //3670

    private final double INTAKE_CUBE_ENC_POS        =  -147.000 + WRIST_ABS_ENC_OFFSET_DEG;//1324.0 + WRIST_ABS_ENC_OFFSET;    //-335
    private final double INTAKE_PICKUP_CONE_ENC_POS_GROUND =  -184.524 + WRIST_ABS_ENC_OFFSET_DEG;//-306.0  + WRIST_ABS_ENC_OFFSET;  //-1295  
    private final double INTAKE_PICKUP_CONE_ENC_POS_SINGLE =  -116.400 + WRIST_ABS_ENC_OFFSET_DEG;//2089.0 + WRIST_ABS_ENC_OFFSET;  //1100

    private final double SCORE_CUBE_ENC_POS         =  -104.000 + WRIST_ABS_ENC_OFFSET_DEG;//1859.0 + WRIST_ABS_ENC_OFFSET;  //870     // Applies to low-mid-high

    private final double SCORE_CONE_HIGH_ENC_POS    =  -153.000 + WRIST_ABS_ENC_OFFSET_DEG;//289.0 + WRIST_ABS_ENC_OFFSET;  //-700
    private final double SCORE_CONE_MID_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD verify if its the same as high
    private final double SCORE_CONE_LOW_ENC_POS     = INTAKE_PICKUP_CONE_ENC_POS_GROUND; //TBD


    private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
    private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD

    private final double GROSS_kP = 0.002472;//0.00009; 
    private final double GROSS_kI = 0.0;//000040;
    private final double GROSS_kD = 0.000291;//0.000007;

    private final double FINE_kP = 0.005234;//0.00009; 
    private final double FINE_kI = 0.0;//000008;
    private final double FINE_kD = 0.000291;//0.000007;
    
    private final double MAX_GRAVITY_FF = 0.055; //0.09

    
    private PIDController pid;
    
    public Boolean  pidEnable = false;

    public double   targetPositionDeg = STOW_ENC_POS;

    private double   targetPower = 0.0;
    private double   prevTargetPwr = 0.0;

    private double currentPosition = -999.0;
    private double positionError = -999.0; 
    public double prevCurrentPosition = -999.0;

    private boolean intakeInPosition = false;

    private final double INTAKE_POS_ERROR_THRESHOLD_DEG = 5.0;
    private final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;

    private double pidPower = 0.0;
    private double ffPower = 0.0;

    private int numConsectSamples = 0;


    //Datalogging
    private CatzDataLogger dataLogger;
    private DoubleLogEntry log_targetPwr;

    //button overides
    private Supplier<Boolean> softLimitOverideBumperEnabledRight;
    private Supplier<Boolean> softLimitOverideBumperEnabledLeft;

  public CatzIntakeSubsytem(CatzDataLogger dataLogger, Supplier<Boolean> softLimitOverideBumperEnabledRight, Supplier<Boolean> softLimitOverideBumperEnabledLeft) 
  {
        //datalogging
        this.dataLogger = dataLogger;
        this.softLimitOverideBumperEnabledRight = softLimitOverideBumperEnabledRight;
        this.softLimitOverideBumperEnabledLeft = softLimitOverideBumperEnabledLeft;

        log_targetPwr = new DoubleLogEntry(dataLogger.log, "targetPwr");
        //----------------------------------------------------------------------------------------------
        //  Roller
        //----------------------------------------------------------------------------------------------
        rollersMtr = new WPI_TalonFX(ROLLERS_MC_ID);
        rollersMtr.configFactoryDefault();
        rollersMtr.setNeutralMode(NeutralMode.Brake);

        rollerCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS_ROLLER, 
                                                                                       CURRENT_LIMIT_TRIGGER_AMPS_ROLLER, 
                                                                                       CURRENT_LIMIT_TIMEOUT_SECONDS);

        rollersMtr.configSupplyCurrentLimit(rollerCurrentLimit);

        //----------------------------------------------------------------------------------------------
        //  Wrist
        //----------------------------------------------------------------------------------------------
        wristMtr = new WPI_TalonFX(WRIST_MC_ID);

        wristMtr.configFactoryDefault();

        wristMtr.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        wristMtr.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
        wristMtr.configIntegratedSensorOffset(0.0);
        
        wristMtr.setNeutralMode(NeutralMode.Brake);

        wristMtr.configForwardSoftLimitThreshold(SOFT_LIMIT_FORWARD);
        wristMtr.configReverseSoftLimitThreshold(SOFT_LIMIT_REVERSE);

        wristMtr.configForwardSoftLimitEnable(true);                  
        wristMtr.configReverseSoftLimitEnable(true);

        wristCurrentLimit  = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, 
                                                                                       CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                                       CURRENT_LIMIT_TIMEOUT_SECONDS);        
        wristMtr.configSupplyCurrentLimit(wristCurrentLimit);

        pid = new PIDController(GROSS_kP, GROSS_kI, GROSS_kD);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    if(pidEnable)
    {
                    //----------------------------------------------------------------------------------
                    //  Chk if at final position
                    //----------------------------------------------------------------------------------
                    currentPosition = wristMtr.getSelectedSensorPosition() / WRIST_CNTS_PER_DEGREE;
                    positionError = currentPosition - targetPositionDeg;


                    if  ((Math.abs(positionError) <= INTAKE_POS_ERROR_THRESHOLD_DEG))
                    {
                        numConsectSamples++;
                        if(numConsectSamples >= 1)
                        {   
                            intakeInPosition = true;
                        }
                    }
                    else
                    {
                        numConsectSamples = 0;
                    }
                    
                    
                    if(Math.abs(positionError) >= PID_FINE_GROSS_THRESHOLD_DEG)
                    {
                        pid.setP(GROSS_kP);
                        pid.setI(GROSS_kI);
                        pid.setD(GROSS_kD);
                    }
                    else if(Math.abs(positionError) < PID_FINE_GROSS_THRESHOLD_DEG)
                    {
                        pid.setP(FINE_kP);
                        pid.setI(FINE_kI);
                        pid.setD(FINE_kD);
                    }

                    pidPower = pid.calculate(currentPosition, targetPositionDeg);
                    ffPower = calculateGravityFF();
                    targetPower = pidPower + ffPower;

                    //-------------------------------------------------------------
                    //  checking if we did not get updated position value(Sampling Issue).
                    //  If no change in position, this give invalid target power(kD issue).
                    //  Therefore, go with prev targetPower Value.
                    //-------------------------------------------------------------------
                    if(prevCurrentPosition == currentPosition)
                    {
                        targetPower = prevTargetPwr;
                    }

                    //----------------------------------------------------------------------------------
                    //  If we are going to Stow Position & have passed the power cutoff angle, set
                    //  power to 0, otherwise calculate new motor power based on position error and 
                    //  current angle
                    //----------------------------------------------------------------------------------
                    if(targetPositionDeg == STOW_ENC_POS && currentPosition > STOW_CUTOFF)
                    {
                        targetPower = 0.0;
                    }
                    wristMtr.set(ControlMode.PercentOutput, targetPower);

                    prevCurrentPosition = currentPosition;
                    prevTargetPwr = targetPower;
                   
                    if((CatzDataLogger.chosenDataID.getSelected() == CatzDataLogger.LOG_ID_INTAKE)) 
                    {        
                        log_targetPwr.append(targetPower, (long) RobotContainer.currentTime.get());
                    }
    }

    if(softLimitOverideBumperEnabledRight.get() && softLimitOverideBumperEnabledLeft.get()){
        wristMtr.configForwardSoftLimitEnable(false);
        wristMtr.configReverseSoftLimitEnable(false);
    }
    else{
        wristMtr.configForwardSoftLimitEnable(true);
        wristMtr.configReverseSoftLimitEnable(true);
    }
    smartdashboardIntakeDebug();


  }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - PID 
    *
    *---------------------------------------------------------------------------------------------*/
    public void resetPID(){
      pidEnable = false;
      pid.reset();
  }

  public void enablePID(boolean set){
      pidEnable = set;
  }

  public boolean getPIDEnabled(){
      return pidEnable;
  }
    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Rollers
    *
    *---------------------------------------------------------------------------------------------*/
    public void rollersOff()
    {
        rollersMtr.set(ControlMode.PercentOutput, 0.0);
    }

    public void rollersInCube()
    {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CUBE_IN);
    }

    public void rollersOutCube()
    {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CUBE_OUT);
    }

    public void rollersInCone()
    {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CONE_IN);
    }

    public void rollersOutCone()
    {
        rollersMtr.set(ControlMode.PercentOutput, ROLLERS_PWR_CONE_OUT);
    }
    

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Wrist
    *
    *---------------------------------------------------------------------------------------------*/
    public void intakeManualHolding(double targetHoldingPwr)
    {
        wristMtr.set(ControlMode.PercentOutput, targetHoldingPwr);
    }
    public double calcWristAngle()
    {
        double wristAngle = ((wristMtr.getSelectedSensorPosition() / WRIST_CNTS_PER_DEGREE) - WRIST_ABS_ENC_OFFSET_DEG);
        return wristAngle;
    }

    public double getWristPosition(){
        return wristMtr.getSelectedSensorPosition();
    }
    
    public double calculateGravityFF()
    {
        double radians = Math.toRadians(calcWristAngle() - CENTER_OF_MASS_OFFSET_DEG);
        double cosineScalar = Math.cos(radians);
        
        return MAX_GRAVITY_FF * cosineScalar;
    }

    public double intakeWristTemp()
    {
        return wristMtr.getTemperature();
    }
    
    public void shuffleboardIntake()
    {
    
    }

    public void smartdashboardIntakeDebug()
    {
        SmartDashboard.putNumber ("wrist ang",  calcWristAngle());
        SmartDashboard.putNumber ("GravityFF",       calculateGravityFF());
        SmartDashboard.putNumber ("IntakeClosedLoopError", pid.getPositionError());
        SmartDashboard.putNumber ("applied output",  wristMtr.getMotorOutputPercent() );
        SmartDashboard.putBoolean("pid",             pidEnable);
        SmartDashboard.putNumber ("mtr abs", wristMtr.getSelectedSensorPosition());
    }

    public boolean isIntakeInPos()
    {
        return intakeInPosition;
    }
/*-------------------------------------------------------
 * Return functions for obtaining Encoder constants from intake subsystem
 * 
 -----------------------------------------------------------*/

    public double getIntakeStowPos()
    {
        return STOW_ENC_POS;
    }

    public double getIntakePickupGroundCubePos()
    {
        return INTAKE_CUBE_ENC_POS;
    }

    public double getIntakePickupGroundConePos()
    {
        return INTAKE_PICKUP_CONE_ENC_POS_GROUND;
    }

    public double getIntakePickupSingleConePos()
    {
        return INTAKE_PICKUP_CONE_ENC_POS_SINGLE;
    }

    public double getIntakeScoreCubePos()
    {
        return SCORE_CUBE_ENC_POS;
    }
    
    public double getIntakeScoreConeHighPos()
    {
        return SCORE_CONE_HIGH_ENC_POS;
    }

    public double getIntakeScoreConeMidPos()
    {
        return SCORE_CONE_MID_ENC_POS;
    }

    public double getIntakeScoreConeLowPos()
    {
        return SCORE_CONE_LOW_ENC_POS;
    }


}
