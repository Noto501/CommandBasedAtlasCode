// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzArmSubsystem;
import frc.robot.subsystems.CatzElevatorSubsystem;
import frc.robot.subsystems.CatzIntakeSubsytem;
import frc.robot.subsystems.CatzStateMachine;


public class MechanismMidPosCommand extends CommandBase {
  private CatzElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private CatzIntakeSubsytem INTAKE_SUBSYSTEM;
  private CatzArmSubsystem ARM_SUBSYSTEM;
  private CatzStateMachine STATE_MACHINE_SUBSYSTEM;

  /** Creates a new MechanismMidPosCommand. */
  public MechanismMidPosCommand(CatzElevatorSubsystem ELEVATOR_SUBSYSTEM, 
                                CatzArmSubsystem ARM_SUBSYSTEM, 
                                CatzIntakeSubsytem INTAKE_SUBSYSTEM, 
                                CatzStateMachine STATE_MACHINE_SUBSYSTEM) 
  {
    this.ELEVATOR_SUBSYSTEM = ELEVATOR_SUBSYSTEM;
    this.INTAKE_SUBSYSTEM = INTAKE_SUBSYSTEM;
    this.ARM_SUBSYSTEM = ARM_SUBSYSTEM;
    this.STATE_MACHINE_SUBSYSTEM = STATE_MACHINE_SUBSYSTEM;

    
      //This command requires elevator, intake, arm subsystems
    addRequirements(ELEVATOR_SUBSYSTEM, INTAKE_SUBSYSTEM, ARM_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    ARM_SUBSYSTEM.armSetRetractPos();

    INTAKE_SUBSYSTEM.pidEnable = true;
    
    if(STATE_MACHINE_SUBSYSTEM.getSelectedGamePiece() == 1)
    {
    INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakeScoreCubePos();
    ELEVATOR_SUBSYSTEM.elevatorSetToMidPosCube();
    }
    else if(STATE_MACHINE_SUBSYSTEM.getSelectedGamePiece() == 2)
    {
      INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakeScoreConeLowPos();
      ELEVATOR_SUBSYSTEM.elevatorSetToMidPosCone();
    }
    else
    {
      
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
