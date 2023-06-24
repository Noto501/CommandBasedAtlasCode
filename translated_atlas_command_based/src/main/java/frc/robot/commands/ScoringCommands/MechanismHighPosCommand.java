// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzArmSubsystem;
import frc.robot.subsystems.CatzElevatorSubsystem;
import frc.robot.subsystems.CatzIntakeSubsytem;
import frc.robot.subsystems.CatzStateMachineSubsystem;

public class MechanismHighPosCommand extends CommandBase {

  private CatzElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private CatzArmSubsystem ARM_SUBSYSTEM;
  private CatzIntakeSubsytem INTAKE_SUBSYSTEM;
  private CatzStateMachineSubsystem STATE_MACHINE_SUBSYSTEM;

  private final double POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR = 73000.0;

  /** Creates a new MechanismHighPosCommand. */
  public MechanismHighPosCommand(CatzElevatorSubsystem ELEVATOR_SUBSYSTEM, CatzArmSubsystem ARM_SUBSYSTEM, CatzIntakeSubsytem INTAKE_SUBSYSTEM, CatzStateMachineSubsystem STATE_MACHINE_SUBSYSTEM) 
  {
    this.ELEVATOR_SUBSYSTEM = ELEVATOR_SUBSYSTEM;
    this.ARM_SUBSYSTEM = ARM_SUBSYSTEM;
    this.INTAKE_SUBSYSTEM = INTAKE_SUBSYSTEM;
    this.STATE_MACHINE_SUBSYSTEM = STATE_MACHINE_SUBSYSTEM;
    

    addRequirements(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ELEVATOR_SUBSYSTEM.elevatorSetToHighPos();    

    INTAKE_SUBSYSTEM.pidEnable = true;
    if(STATE_MACHINE_SUBSYSTEM.getSelectedGamePiece() == 1)
    {
    INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakeScoreCubePos();
    }
    else
    {
      INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakeScoreConeHighPos();
    }
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double elevatorPosition = ELEVATOR_SUBSYSTEM.getElevatorEncoder();

    if(false)//DriverStation.isAutonomousEnabled() && Robot.selectedGamePiece == Robot.GP_CONE) // TBD need to find where robot.selectedGamePiece is
    {
        if(elevatorPosition >= POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR && 
            INTAKE_SUBSYSTEM.isIntakeInPos())
        {
            ARM_SUBSYSTEM.armSetFullExtendPos();
        }
    }
    else
    {
        if(elevatorPosition >= POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR)
        {
            ARM_SUBSYSTEM.armSetFullExtendPos();
        }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
