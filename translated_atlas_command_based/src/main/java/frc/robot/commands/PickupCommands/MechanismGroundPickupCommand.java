// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.subsystems.CatzArmSubsystem;
import frc.robot.subsystems.CatzElevatorSubsystem;
import frc.robot.subsystems.CatzIntakeSubsytem;
import frc.robot.subsystems.CatzStateMachine;

public class MechanismGroundPickupCommand extends CommandBase {

  private CatzElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private CatzArmSubsystem ARM_SUBSYSTEM;
  private CatzIntakeSubsytem INTAKE_SUBSYSTEM;
  private CatzStateMachine STATE_MACHINE;

  private final double ARM_ENCODER_THRESHOLD = 35000.0;

  /** Creates a new MechanismGroundPickupCommand. */
  public MechanismGroundPickupCommand(CatzElevatorSubsystem ELEVATOR_SUBSYSTEM, 
                                      CatzArmSubsystem ARM_SUBSYSTEM,
                                      CatzIntakeSubsytem INTAKE_SUBSYSTEM, 
                                      CatzStateMachine STATE_MACHINE)
  {
    this.ELEVATOR_SUBSYSTEM = ELEVATOR_SUBSYSTEM;
    this.INTAKE_SUBSYSTEM = INTAKE_SUBSYSTEM;
    this.ARM_SUBSYSTEM = ARM_SUBSYSTEM;
    this.STATE_MACHINE = STATE_MACHINE;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_SUBSYSTEM, INTAKE_SUBSYSTEM, ARM_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Grndpckup cmd start");
    ARM_SUBSYSTEM.armSetPickupPos();

    if(STATE_MACHINE.getSelectedGamePiece() == CatzConstants.GP_CUBE) INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakePickupGroundCubePos();
    else                                                              INTAKE_SUBSYSTEM.targetPositionDeg = INTAKE_SUBSYSTEM.getIntakePickupGroundConePos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    System.out.println("Grndpckup cmd thread strt");
     
    if (ARM_SUBSYSTEM.getArmEncoder() <= ARM_ENCODER_THRESHOLD)
    { 
        ELEVATOR_SUBSYSTEM.elevatorSetToLowPos();
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Grndpckup cmd end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
