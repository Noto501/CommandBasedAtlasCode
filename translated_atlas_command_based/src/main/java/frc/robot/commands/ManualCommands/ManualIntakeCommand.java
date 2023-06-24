// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzIntakeSubsytem;


public class ManualIntakeCommand extends CommandBase {

  private CatzIntakeSubsytem INTAKE_SUBSYSTEM;

  private Supplier<Double> IntakeMtrPwr; 
  private Supplier<Boolean> fullManualActiveFunction;

  private double targetPower;
  private double targetPositionDeg;
  private double prevCurrentPosition;
  
  //Constants
  private final double MANUAL_HOLD_STEP_SIZE = 1.5;  
  private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
  private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD  
  private final double WRIST_MAX_PWR = 0.3;

  
  /** Creates a new DefaultHoldingIntakeCommand. */
  public ManualIntakeCommand(CatzIntakeSubsytem INTAKE_SUBSYSTEM, Supplier<Double> IntakeMtrPwr, 
                                                                  Supplier<Boolean> fullManualActiveFunction) 
  {
    this.INTAKE_SUBSYSTEM = INTAKE_SUBSYSTEM;
    this.IntakeMtrPwr = IntakeMtrPwr;
    this.fullManualActiveFunction = fullManualActiveFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double wristPwr = IntakeMtrPwr.get();
    boolean manualMode = fullManualActiveFunction.get();


  if(Math.abs(wristPwr) >= 0.1)//if we are apply wrist power manually
  {
      if (manualMode)//check if in full manual mode
      {
        targetPower = wristPwr * WRIST_MAX_PWR;    
        INTAKE_SUBSYSTEM.intakeManualHolding(targetPower);
      }
      else // in manual holding state
      {
        INTAKE_SUBSYSTEM.pidEnable = true;
        
        if(wristPwr > 0)
        {
            INTAKE_SUBSYSTEM.targetPositionDeg = Math.min((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_FORWARD);
        }
        else
        {
            INTAKE_SUBSYSTEM.targetPositionDeg = Math.max((targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_REVERSE);
        }
        INTAKE_SUBSYSTEM.prevCurrentPosition = -INTAKE_SUBSYSTEM.prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values
      }
  }
  else //Manual power is OFF
  {
      if(manualMode)//if we are still in manual mode and want to hold intake in place
      {
          targetPower = 0.0;
          INTAKE_SUBSYSTEM.intakeManualHolding(targetPower);
      }
  }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
