package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class CatzDataLogger 
{

    public static final SendableChooser<Integer> chosenDataID = new SendableChooser<>();

    public static final int LOG_ID_NONE            = 0;
    public static final int LOG_ID_SWERVE_STEERING = 1;
    public static final int LOG_ID_SWERVE_DRIVING  = 2;
    public static final int LOG_ID_ARM             = 3;
    public static final int LOG_ID_INTAKE          = 4;
    public static final int LOG_ID_ELEVATOR        = 6;
    public static final int LOG_ID_DRV_STRAIGHT    = 8;
    public static final int LOG_ID_TURN_IN_PLACE   = 9;
    public static final int LOG_ID_BALANCE         = 10;

    long currentTime;

    public DataLog log;



    public CatzDataLogger()
    {
        log = DataLogManager.getLog();

    }
    

    public void dataCollectionShuffleboard()
    {
        chosenDataID.setDefaultOption("None",        LOG_ID_NONE);
        chosenDataID.addOption("Swerve Steering", LOG_ID_SWERVE_STEERING);
        chosenDataID.addOption("Swerve Driving", LOG_ID_SWERVE_DRIVING);
        chosenDataID.addOption("Arm", LOG_ID_ARM);
        chosenDataID.addOption("Intake", LOG_ID_INTAKE);
        chosenDataID.addOption("Elevator", LOG_ID_ELEVATOR);
        chosenDataID.addOption("Auton Drv Straight", LOG_ID_DRV_STRAIGHT);
        chosenDataID.addOption("Auton Turn In Place", LOG_ID_TURN_IN_PLACE);
        chosenDataID.addOption("Balance", LOG_ID_BALANCE);

        SmartDashboard.putData("Data Collection", chosenDataID);
    
    }
}
