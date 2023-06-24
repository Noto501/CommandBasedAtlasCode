// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.CatzConstants.OperatorConstants;
import frc.robot.commands.ManualCommands.ManualArmCommand;
import frc.robot.commands.ManualCommands.ManualElevatorCommand;
import frc.robot.commands.ManualCommands.ManualIntakeCommand;
import frc.robot.commands.MiscCommands.DetermineStateCommand;
import frc.robot.commands.MiscCommands.DrivetrainWheelLockCommand;
import frc.robot.commands.MiscCommands.IntakeRollersCommand;
import frc.robot.commands.MiscCommands.MechanismStowPosCommand;
import frc.robot.commands.MiscCommands.NavxResetFieldOrientationCommand;
import frc.robot.commands.ManualCommands.DefaultDriveCommand;
import frc.robot.commands.PickupCommands.MechanismDoublePickupCommand;
import frc.robot.commands.PickupCommands.MechanismGroundPickupCommand;
import frc.robot.commands.PickupCommands.MechanismSinglePickupCommand;
import frc.robot.commands.ScoringCommands.MechanismHighPosCommand;
import frc.robot.commands.ScoringCommands.MechanismLowPosCommand;
import frc.robot.commands.ScoringCommands.MechanismMidPosCommand;
import frc.robot.subsystems.CatzArmSubsystem;
import frc.robot.subsystems.CatzDriveTrainSubsystem;
import frc.robot.subsystems.CatzElevatorSubsystem;
import frc.robot.subsystems.CatzIntakeSubsytem;
import frc.robot.subsystems.CatzStateMachineSubsystem;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.kauailabs.navx.frc.AHRS;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...



  //subsystems
  private final CatzDriveTrainSubsystem DRIVE_SUBSYSTEM;
  private final CatzIntakeSubsytem INTAKE_SUBSYSTEM;
  private final CatzElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private final CatzArmSubsystem ARM_SUBSYSTEM;
  private final CatzStateMachineSubsystem STATE_MACHINE_SUBSYSTEM;

  private final CatzConstants catzConstants;

  private CommandXboxController xboxDrv;
  private CommandXboxController xboxAux;

  public static AHRS              navX;
  //xbox commands

  //RobotContainer Constants
  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  private final int GP_NONE = 0;
  private final int GP_CUBE = 1;
  private final int GP_CONE = 2;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    catzConstants = new CatzConstants();

    DRIVE_SUBSYSTEM  = new CatzDriveTrainSubsystem();
    INTAKE_SUBSYSTEM = new CatzIntakeSubsytem();
    ELEVATOR_SUBSYSTEM = new CatzElevatorSubsystem();
    ARM_SUBSYSTEM = new CatzArmSubsystem();
    STATE_MACHINE_SUBSYSTEM = new CatzStateMachineSubsystem();


    xboxDrv = new CommandXboxController(XBOX_DRV_PORT); 
    xboxAux = new CommandXboxController(XBOX_AUX_PORT);

    navX = new AHRS();
    navX.reset();
  
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
    //-------------------Scoring Position Bindings------------------------------------
    xboxAux.y().onTrue(new MechanismHighPosCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM));
    xboxAux.b().onTrue(new MechanismMidPosCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM));
    xboxAux.a().onTrue(new MechanismLowPosCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM));

    //------------------Stowing Position Bindings-------------------------------------
    xboxAux.x().onTrue(new MechanismStowPosCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM));
    xboxDrv.rightStick().onTrue(new MechanismStowPosCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM));
  

    //---------------------Determining Game Piece Bindings------------------------------
    xboxAux.back().onTrue(new DetermineStateCommand(GP_NONE, STATE_MACHINE_SUBSYSTEM));
    xboxAux.povLeft().onTrue(new DetermineStateCommand(GP_CUBE, STATE_MACHINE_SUBSYSTEM));
    xboxAux.povRight().onTrue(new DetermineStateCommand(GP_CONE, STATE_MACHINE_SUBSYSTEM));
    
    //------------------------PickupPositon Bindings--------------------------
    xboxDrv.rightTrigger().onTrue(new MechanismDoublePickupCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM));
    xboxDrv.leftTrigger().onTrue(new MechanismSinglePickupCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM));
    xboxDrv.leftStick().onTrue(new MechanismGroundPickupCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM));
    xboxAux.start().onTrue(new MechanismGroundPickupCommand(ELEVATOR_SUBSYSTEM, ARM_SUBSYSTEM, INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM));


    //-------------------------Manual Arm Bindings--------------------------------------
    xboxAux.rightTrigger().onTrue(new ManualArmCommand(ARM_SUBSYSTEM, () -> xboxAux.getRightTriggerAxis()));
    xboxAux.leftTrigger().onTrue(new ManualArmCommand(ARM_SUBSYSTEM, () -> xboxAux.getLeftTriggerAxis()));
    

    //---------------------------Intake Roller Bindings-----------------------------
    xboxAux.rightBumper().onTrue(new IntakeRollersCommand(INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM, () -> xboxAux.rightBumper().getAsBoolean(), () -> xboxAux.leftBumper().getAsBoolean()));
    xboxAux.leftBumper().onTrue(new IntakeRollersCommand(INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM, () -> xboxAux.rightBumper().getAsBoolean(), () -> xboxAux.leftBumper().getAsBoolean()));
    xboxDrv.rightBumper().onTrue(new IntakeRollersCommand(INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM, () -> xboxAux.rightBumper().getAsBoolean(), () -> xboxAux.leftBumper().getAsBoolean()));
    xboxDrv.leftBumper().onTrue(new IntakeRollersCommand(INTAKE_SUBSYSTEM, STATE_MACHINE_SUBSYSTEM, () -> xboxAux.rightBumper().getAsBoolean(), () -> xboxAux.leftBumper().getAsBoolean()));

    //--------------------------Misc Bindings------------------------------------------------
    xboxDrv.start().onTrue(new NavxResetFieldOrientationCommand(navX));
    xboxDrv.a().onTrue(new DrivetrainWheelLockCommand(DRIVE_SUBSYSTEM));


  }

  private void defaultCommands() 
  {
    ELEVATOR_SUBSYSTEM.setDefaultCommand(new ManualElevatorCommand(ELEVATOR_SUBSYSTEM, () -> xboxAux.getLeftY(), () -> xboxAux.leftStick().getAsBoolean()));
    INTAKE_SUBSYSTEM.setDefaultCommand(new ManualIntakeCommand(INTAKE_SUBSYSTEM, () -> xboxAux.getRightY(), () -> xboxAux.rightStick().getAsBoolean()));

    DRIVE_SUBSYSTEM.setDefaultCommand(new DefaultDriveCommand(DRIVE_SUBSYSTEM,
                                                              () -> xboxDrv.getLeftX(),
                                                              () -> xboxDrv.getLeftY(), 
                                                              () -> xboxDrv.getRightX(), 
                                                              () -> navX.getAngle(), 
                                                              () -> xboxDrv.getRightTriggerAxis()));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(//TBD will need to get updates on Yuyhun trajectory planner before I proceed with auton
      
    );
  }
}
