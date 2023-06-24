// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzArmSubsystem;
import frc.robot.subsystems.CatzElevatorSubsystem;
import frc.robot.subsystems.CatzIntakeSubsytem;
import frc.robot.subsystems.CatzStateMachineSubsystem;

public class MechanismSinglePickupCommand extends CommandBase {

  private CatzElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private CatzArmSubsystem ARM_SUBSYSTEM;
  private CatzIntakeSubsytem INTAKE_SUBSYSTEM;
  private CatzStateMachineSubsystem STATE_MACHINE_SUBSYSTEM;

  /** Creates a new MechanismSinglePickupCommand. */
  public MechanismSinglePickupCommand(CatzElevatorSubsystem ELEVATOR_SUBSYSTEM, 
                                      CatzArmSubsystem ARM_SUBSYSTEM, 
                                      CatzIntakeSubsytem INTAKE_SUBSYSTEM, 
                                      CatzStateMachineSubsystem STATE_MACHINE_SUBSYSTEM) 
  {
    this.ELEVATOR_SUBSYSTEM = ELEVATOR_SUBSYSTEM;
    this.INTAKE_SUBSYSTEM = INTAKE_SUBSYSTEM;
    this.ARM_SUBSYSTEM = ARM_SUBSYSTEM;
    this.STATE_MACHINE_SUBSYSTEM = STATE_MACHINE_SUBSYSTEM;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_SUBSYSTEM, INTAKE_SUBSYSTEM, ARM_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ELEVATOR_SUBSYSTEM.elevatorSetToSinglePickup();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    ARM_SUBSYSTEM.armSetPickupPos();
    if(STATE_MACHINE_SUBSYSTEM.getSelectedGamePiece() == 1)
    {
    INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakeScoreCubePos();
    }
    else
    {
      INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakePickupSingleConePos();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ELEVATOR_SUBSYSTEM.getElevatorEncoder() == 27739;
  }
}
