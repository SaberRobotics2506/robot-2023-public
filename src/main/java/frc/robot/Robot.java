// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;


//import edu.wpi.first.wpilibj.DoubleSolenoid;


import edu.wpi.first.networktables.DoublePublisher;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String ArcadeMessage = "Arcade";
  private static final String TankMessage = "Tank";

  DigitalInput ultrasonicIn = new DigitalInput(Constants.ULTRASONIC_IN_ID);
  DigitalOutput ultrasonicOut = new DigitalOutput(Constants.ULTRASONIC_OUT_ID);
  Ultrasonic ultrasonic = new Ultrasonic(ultrasonicOut, ultrasonicIn);
  Timer haveUsedCharger = new Timer();

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> drive_chooser = new SendableChooser<>();
  WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.TOP_LEFT_ID);
  WPI_TalonFX frontRight = new WPI_TalonFX(Constants.TOP_RIGHT_ID);
  WPI_TalonFX backRight = new WPI_TalonFX(Constants.BOTTOM_RIGHT_ID);
  WPI_TalonFX backLeft = new WPI_TalonFX(Constants.BOTTOM_LEFT_ID);
  //WPI_TalonFX middleWheel = new WPI_TalonFX(Constants.H_MOTOR_ID);
  WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);
  MotorControllerGroup leftSide = new MotorControllerGroup(backLeft, frontLeft);
  MotorControllerGroup rightSide = new MotorControllerGroup(frontRight, backRight);
  DifferentialDrive difDrive = new DifferentialDrive(leftSide, rightSide);
  private final SendableChooser<String> auto_chooser = new SendableChooser<>();
  private static final String doNothing = "Do Nothing Auto";
  private static final String justScoreMid = "Just Score Mid";
  private static final String justScoreHigh = "Just Score High";
  private static final String justDrive = "Just Drive";
  //private static final String justBalance = "Just Balance";
  private static final String scoreMidAndDrive = "Score Mid and Drive";
  private static final String scoreHighAndDrive = "Score High and Drive";
  private static final String scoreMidAndBalance = "Score Mid and Balance";
  private static final String scoreHighAndBalance = "Score High and Balance";
  private static final String scoreMidAndExitAndBalance = "Score Mid and Exit and Balance";
  private static final String scoreHighAndExitAndBalance = "Score High and Exit and Balance";
  SupplyCurrentLimitConfiguration limitingMethod = new SupplyCurrentLimitConfiguration(true, Constants.ampLimit, 5, 0);
  public XboxController driveController = new XboxController(Constants.CONTROLLER_DRIVER_0);
  public XboxController operatorController = new XboxController(Constants.CONTROLLER_OPERATOR_1);
  
  double kpDriveOver = 0.0285;
  double kpDriveUp = 0.03;
  double ki = 0;
  double kd = 0.0025;
  

  // public PneumaticHub pnuematicHub = new PneumaticHub(1);


  DriveTrain drive = new DriveTrain(difDrive, driveController, frontLeft, backRight, frontRight, backLeft, pigeon);
  Rangefinder rangefinder = new Rangefinder(ultrasonic, drive);
  private String drive_choice = drive_chooser.getSelected();
  LED emLed = new LED(rangefinder, driveController, operatorController, drive);

  // creates both motors for intake
  WPI_TalonFX intakeLeft = new WPI_TalonFX(Constants.LEFT_MOTOR_ID);
  WPI_TalonFX intakeRight = new WPI_TalonFX(Constants.RIGHT_MOTOR_ID);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLAW_DEPLOYED_CHANNEL, Constants.CLAW_RETRACTED_CHANNEL);
  public boolean clawToggle = false;
  private Claw claw = new Claw(solenoid, operatorController);
  public boolean failSafe = false;

  WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(0);
  //DifferentialDrive difDrive = new DifferentialDrive(leftSide, rightSide);

  
//TODO: DONE change that .035 back to 0.03during competition
  String selectedAuto = "General Auto";
  Autonomous epicAuto;
  boolean chargingDoneTele = false;


  private Camera camera = new Camera(driveController);
  DoublePublisher driveXPub;
  DoublePublisher driveYPub;
  DoublePublisher distPub;
  DoublePublisher aprilTagPub;
  DoublePublisher logRampMultiplierPub;
  boolean finishAuto;
  


  // Arm an Elevator Variables
  WPI_TalonFX arm_motor_right = new WPI_TalonFX(Constants.ELEVATOR_RIGHT_ID);
  WPI_TalonFX arm_motor_left = new WPI_TalonFX(Constants.ELEVATOR_LEFT_ID);

  WPI_TalonFX arm_motor_out = new WPI_TalonFX(Constants.ARM_MOTOR_ID);

  public boolean toggle_manual = true;
  public int mode_tracker;

  // public boolean right_po = arm_motor_right.getSelectedSensorPosition()

  public Arm arm = new Arm(arm_motor_out);

  public Elevator elevator = new Elevator(arm_motor_right, arm_motor_left);

  PickupSystem pickupSystem = new PickupSystem(claw, arm, elevator, operatorController, mode_tracker);
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    emLed.blinkin(0);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("Encoder Value", frontLeft.getSelectedSensorPosition());
    drive.resetMotors();
    // sets forward direction for motors and set ramping
    frontLeft.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    
    //ramps
    
    drive.drivetrainTelemetryTable();
    pickupSystem.pickupSystemTelemetrySetup();
    

    //drive_chooser.setDefaultOption("FOHdrive" , FOHDriveMessage);
    drive_chooser.setDefaultOption("Arcade Drive" , ArcadeMessage);
    drive_chooser.addOption("Tank Drive" , TankMessage);
    //drive_chooser.addOption("Arcade drive" , ArcadeMessage);
    SmartDashboard.putData("Drive Train Choice", drive_chooser);

    auto_chooser.setDefaultOption("Nothing", doNothing);
    auto_chooser.addOption("Nothing", doNothing);
    auto_chooser.addOption("Just Score Mid", justScoreMid);
    auto_chooser.addOption("Just Score High", justScoreHigh);
    auto_chooser.addOption("Just Drive", justDrive);
    //auto_chooser.addOption("Just Balance", justBalance);
    auto_chooser.addOption("Score Mid and Drive", scoreMidAndDrive);
    auto_chooser.addOption("Score High and Drive", scoreHighAndDrive);
    auto_chooser.addOption("Score Mid and Balance", scoreMidAndBalance);
    auto_chooser.addOption("Score High and Balance", scoreHighAndBalance);
    auto_chooser.addOption("Score Mid and Exit and Balance", scoreMidAndExitAndBalance);
    auto_chooser.addOption("Score High and Exit and Balance", scoreHighAndExitAndBalance);
    SmartDashboard.putData("Auto choices", auto_chooser);
    SmartDashboard.putNumber("Yaw value", pigeon2.getYaw());
    pigeon2.reset();

    /*GenericEntry autoChoice = Shuffleboard.getTab("SmartDashboard")
    autoChoice.add("Auto Choice", false)
    .withWidget("Toggle Button")
    .getEntry();
    */
    
    //THIS TOO

    
    // Claw starts off closed
    claw.closeClaw();

    claw.clawTelemetryTable();
 
    // starts recording 
    camera.cameraStart();
    // creates claw telemetry
  

    arm.arm_setting_reset();
    elevator.elevator_setting_reset();

    // pnuematicHub.enableCompressorAnalog(Constants.AIR_COMPRESSOR_MIN_PRESSURE, Constants.AIR_COMPRESSOR_MAX_PRESSURE);

    //creates a config for the pigeon
    drive.pigeonConfig();
    // limits motor output to certain current
    drive.rateLimiter();
    elevator.rateLimiter();
    arm.rateLimiter();
    arm.arm_config();
    elevator.elevator_config();
    toggle_manual = true;
    
    // frontLeft.configSupplyCurrentLimit(limitingMethod);
    // frontRight.configSupplyCurrentLimit(limitingMethod);
    // backLeft.configSupplyCurrentLimit(limitingMethod);
    // backRight.configSupplyCurrentLimit(limitingMethod);
    // middleWheel.configSupplyCurrentLimit(limitingMethod);

    selectedAuto = auto_chooser.getSelected();
    //epicAuto = new Autonomous(pigeon2, drive, pickupSystem,elevator, arm, claw, selectedAuto, frontLeft, frontRight, backLeft, backRight, middleWheel, pid, emLed);
    }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pitch Value: ", pigeon.getPitch());
    
  }


  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro 
   * (secret message no one will see because nobody reads these comments)
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    DataLogManager.start();
    frontLeft.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    //middleWheel.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);

    //m_autoSelected = m_chooser.getSelected();
    //System.out.println("Auto selected: " + m_autoSelected);
    drive.commandDrive(0, 0, 0);
    drive.resetEncoders();
    selectedAuto = auto_chooser.getSelected();
    double kp = 0.0;
    if (selectedAuto.equals(scoreMidAndBalance) || selectedAuto.equals(scoreHighAndBalance)){
      kp = kpDriveUp;
    }else{
      kp = kpDriveOver;
    }
    PIDController pid = new PIDController(kp, ki, kd);
    epicAuto = new Autonomous(pigeon2, drive, pickupSystem, elevator, arm, claw, selectedAuto, frontLeft, frontRight, backLeft, backRight, pid, emLed);
    pigeon2.reset();
    epicAuto.autoTelemetrySetup();
    System.out.println(selectedAuto);
    pickupSystem.resetPosition();
    arm.arm_setting_reset();
    elevator.elevator_setting_reset();
    finishAuto= false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    emLed.changeColorsAuto(epicAuto.getStepCounter());
    drive.collectDrivetrainTelemetry();
    claw.collectClawTelemetry();
    epicAuto.collectAutoTelemetry();
    pickupSystem.collectPickupSystemTelemetry();    DriverStation.startDataLog(DataLogManager.getLog());
    SmartDashboard.putNumber("Encoder Value", frontLeft.getSelectedSensorPosition());
    arm.armPeriodic();
    elevator.elevatorPeriodic();
    pickupSystem.periodicDashboard();
    switch (selectedAuto){
      case doNothing:
        // Put default auto code here
        epicAuto.doNothingAutoRun();
        break;
        // Put custom auto code here
      case justScoreMid:
        epicAuto.justScoreMidAutoRun();
        break;
      case justScoreHigh:
        epicAuto.justScoreHighAutoRun();
        break;
      case justDrive:
        epicAuto.justDriveAutoRun();
        break;
      /*
       case justBalance:
        epicAuto.justBalanceAutoRun();
        break;
       */
      case scoreMidAndDrive:
        epicAuto.scoreMidAndDriveAutoRun();
        break;
      case scoreHighAndDrive:
        epicAuto.scoreHighAndDriveAutoRun();
        break;
      case scoreMidAndBalance:
        epicAuto.scoreMidAndBalanceAutoRun();
        break;
      case scoreHighAndBalance:
        epicAuto.scoreHighAndBalanceAutoRun();
        break;
      case scoreMidAndExitAndBalance:
        epicAuto.scoreMidAndExitAndBalanceAutoRun();
        break;
      case scoreHighAndExitAndBalance:
        epicAuto.scoreHighAndExitAndBalanceAutoRun();
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    DataLogManager.start();
    System.out.println("Drive selected: " + drive_choice);

    // limits motor output to certain current
    // drive.rateLimiter();
    toggle_manual = true;
    drive_choice = drive_chooser.getSelected();
    // frontLeft.configSupplyCurrentLimit(limitingMethod);
    // frontRight.configSupplyCurrentLimit(limitingMethod);
    // backLeft.configSupplyCurrentLimit(limitingMethod);
    // backRight.configSupplyCurrentLimit(limitingMethod);
    // middleWheel.configSupplyCurrentLimit(limitingMethod);

    rangefinder.enable();
    Rangefinder.setAutomaticMode();


    
  }

  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() 
  {
    emLed.changeColors();
    camera.cameraSwitcher();
    drive_choice = drive_chooser.getSelected();
    
    arm.armPeriodic();
    elevator.elevatorPeriodic();
    // SmartDashboard.putNumber("Pressure Value", pnuematicHub.getPressure(0));
    switch(drive_choice) {
      /*case FOHDriveMessage:
        drive.deadFOHdrive();
        break;*/
      case TankMessage:
        drive.tankDrive();
        break;
      case ArcadeMessage:
        drive.Arcade_Drive();
        break;
        

      }
    drive.checkSlowToggle();
    drive.collectDrivetrainTelemetry();
    pickupSystem.collectPickupSystemTelemetry();
    

    rangefinder.getDistanceInches();
    if (driveController.getAButton())
    {
      rangefinder.driveToDistance(Constants.ULTRASONIC_DRIVE_DESIRED_DISTANCE);
    }

    //CHARGING PLATFORM BUTTON
    /*
    if(!chargingDoneTele)
    {
      if(driveController.getXButtonPressed())
      {
        chargingDoneTele = epicAuto.balanceOnStation();
      }
    }
    if(chargingDoneTele)
    {
      haveUsedCharger.start();
      if(haveUsedCharger.get() > 3)
      {
        chargingDoneTele = false;
        haveUsedCharger.stop();
        haveUsedCharger.reset();
      }
    }
    */
    
    



    

    SmartDashboard.putNumber("controller Left X input", driveController.getLeftX());
    SmartDashboard.putNumber("controller Left y input", driveController.getLeftY());
    SmartDashboard.putNumber("controller Right X input", driveController.getRightX());
    SmartDashboard.putNumber("Yaw Value: ", pigeon.getYaw());    
   
    // Toggle for opening and closing the claw.
    //CLAW CONTROL HAS BEEN INTEGRATED INTO PICKUPSYSTEM //claw.clawToggled();
    
    
    // Starts to collect telemetry data for claw.
    claw.collectClawTelemetry();


    //Arm subsystems-
    // TODO: Optimize toggles.


    if (operatorController.getStartButtonPressed()) {
      toggle_manual = !toggle_manual;
    }

     if (toggle_manual == true) {
      // DataLogManager.log("Arm in Manual!");
      pickupSystem.manual_mode();
      SmartDashboard.putBoolean("Manual Mode", true);
     } else {
      // DataLogManager.log("Arm in Auto!");
      pickupSystem.controlPickupSystem();
      pickupSystem.periodicDrive();
      SmartDashboard.putBoolean("Manual Mode", false);
   }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}