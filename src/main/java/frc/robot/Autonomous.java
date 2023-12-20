package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Conversion from wheel revolutions to inches
//C = pi × d = 3.14 x 6 = 18.84 inches per wheel revolution
//For Falcon 500: 2048 ticks per motor rotation × 9.759 motor rotations per wheel revolution = 19986.432 ticks per wheel revolution
//19986.432 ticks = 18.84 inches
//1060.851 ticks = 1 inch

//For H-Drive Motor: 3.14 * 3.25 = 10.205 inches per wheel revolution
//2048 ticks per motor rotation * 5.4 gearbox ratio = 11059.2
//1083.7 ticks = 1 inch

public class Autonomous{
  WPI_Pigeon2 bird;
  DriveTrain roboMove;
  int startup = 0;
  String selectedAuto;
  double startingDegrees;
  int degreeWorker = 0;
  double autoStep = 0;
  PickupSystem PUsystem;
  Claw claw;
  Arm arm;
  Elevator elev;
  DoublePublisher stepCounterTelemetry;
  boolean isCurrentStepDone = false;
  Timer chargingTime = new Timer();
  Timer dropTimer = new Timer();
  Timer offsetTimer = new Timer();
  
  WPI_TalonFX frontLeft;
  WPI_TalonFX frontRight;
  WPI_TalonFX backLeft;
  WPI_TalonFX backRight;
  WPI_TalonFX middleWheel;
  int stepCounter;
  int finaleHelper;
  int scoreHelper;
  PIDController pid;
  LED led;

  public Autonomous(WPI_Pigeon2 bird, DriveTrain roboMove, PickupSystem PUsystem,Elevator elev, Arm arm, Claw claw, String selectedAuto, WPI_TalonFX frontLeft, WPI_TalonFX frontRight, WPI_TalonFX backLeft, WPI_TalonFX backRight, PIDController pid, LED led){
    this.bird = bird;
    this.roboMove = roboMove;
    this.selectedAuto = selectedAuto;
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
    this.PUsystem = PUsystem;
    this.arm = arm;
    this.claw = claw;
    this.middleWheel = middleWheel;
    this.elev = elev;
    this.pid = pid;
    this.led = led;
    stepCounter = 0;
    finaleHelper = 0;
    scoreHelper = 0;
  }
  public void autoTelemetrySetup()
  {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Autonomous");

    stepCounterTelemetry = table.getDoubleTopic("Step Counter").publish();
  }
  public void collectAutoTelemetry()
  {
    stepCounterTelemetry.set(stepCounter);
  }
  /*
   * AUTONOMOUS MODES
   */

  //1. Nothing
  public void doNothingAutoRun(){

  }
  
  //2. Just Score (Mid)
  public void justScoreMidAutoRun(){
    if(stepCounter<3){
      scoreCone(false);
    }
  }

  //3. Just Score (High)
  public void justScoreHighAutoRun(){
    if(stepCounter<3){
      scoreCone(true);
    }
  }

  //4. Just Drive
  public void justDriveAutoRun(){
      if(stepCounter == 0){
        goSet(-Constants.autoDistanceToMidField, Constants.autoDriveSpeed);
      }
    }
  //5. Just Balance
  public void justBalanceAutoRun(){
    if(stepCounter == 0){
      roboMove.commandDrive(0, -0.4, -0.4);
      if(bird.getYaw() <= Constants.lowThreshold || bird.getYaw() >= Constants.highThreshold){
        stepCounter++;
      }
    }else{
      balanceOnStation(false);
    }
  }

  //6.Score + Drive (Mid)
  public void scoreMidAndDriveAutoRun(){
    if(stepCounter<3){
      scoreCone(false);
    }else if(stepCounter==3){
      goSet(-Constants.autoDistanceToMidField, Constants.autoDriveSpeed);
    }
  }

  //7. Score + Drive (High)
  public void scoreHighAndDriveAutoRun(){
    if(stepCounter<3){
      scoreCone(true);
    }else if(stepCounter == 3){
      goSet(-Constants.autoDistanceToMidField, Constants.autoDriveSpeed);
    }
  }
  

  //8. Score + Balance (Mid)
  public void scoreMidAndBalanceAutoRun(){
    if(stepCounter<3){
      scoreCone(false);
    }else if(stepCounter==3){
      goSet(-70, 0.6);
    }else{
      balanceOnStation(false);
    }
  }

  //9. Score + Balance (High)
  public void scoreHighAndBalanceAutoRun(){
    if(stepCounter<3){
      scoreCone(true);
    }else if(stepCounter==3){
      goSet(-70, 0.6);
    }else{
      balanceOnStation(false);
    }
  }

  //10. Score + Exit + Balance (Mid)
  public void scoreMidAndExitAndBalanceAutoRun(){
    SmartDashboard.putNumber("Step On", stepCounter);
    roboMove.checkSlowToggle();
    if(stepCounter < 3){
      scoreCone(false);
    }else if(stepCounter == 3){
      goSet(-150, 0.56);
    }
    else if(stepCounter == 4){
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {

      }
      stepCounter++;
    }else if(stepCounter == 5){
      goSet(66, 0.6);
    }else{
      balanceOnStation(true);
    }
  }

  //11. Score + Exit + Balance (High)
  public void scoreHighAndExitAndBalanceAutoRun(){
    SmartDashboard.putNumber("Step On", stepCounter);
    if(stepCounter < 3){
      scoreCone(true);
    }else if(stepCounter == 3){
      goSet(-150, 0.56);
    }else if(stepCounter == 4)
    {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {

      }
      stepCounter++;
    }else if(stepCounter == 5){
      goSet(66, 0.6);
    }else{
      balanceOnStation(true);
    }
  }

  /*
   * METHODS
   */
  
  //Scores high goal
  public void scoreCone(boolean high){
    roboMove.difDrive.feed();
    SmartDashboard.putNumber("drop timer", dropTimer.get());
    SmartDashboard.putNumber("step on", stepCounter);
    double armPositionValue;
    double armElevatorPositionValue;
    if(high){
      armPositionValue = Constants.HIGH_CONE_ARM;
      armElevatorPositionValue = Constants.HIGH_CONE_ELEV;
    }else{
      armPositionValue = Constants.MID_CONE_ARM;
      armElevatorPositionValue = Constants.MID_CONE_ELEV;
    }
    if(stepCounter == 0){
      if(high){
        PUsystem.coneHighPosition();
      }else{
        PUsystem.coneMidPosition();
      }
      PUsystem.periodicDrive();
      if(arm.isInDestinationRange(1) && elev.isInDestinationRange(1)){
        stepCounter++;
        dropTimer.start();
      }
    }else if(stepCounter == 1 && dropTimer.get() > Constants.clawOpenWait){
      claw.openClaw();
      stepCounter++;
    }else if(stepCounter == 2 && dropTimer.get() > Constants.moveBackElevWait){
      PUsystem.transitPosition();
      PUsystem.periodicDrive();
      /*if(aroundEqual(elev.getPosition(),(Constants.TRANSIT_ELEVATOR + armElevatorPositionValue)/2)){
        claw.closeClaw();
      }*/
      if(arm.isInDestinationRange(1) && elev.isInDestinationRange(1)){
        stepCounter++;
        dropTimer.stop();
        dropTimer.reset();
      }
    }
  }

  //Balances on the charging station
  public void balanceOnStation(boolean forward){
    int speed;
    if(forward){
      speed = 1;
    }else{
      speed = -1;
    }
    double superPidValue = pid.calculate(bird.getPitch(), 0);

    SmartDashboard.putNumber("The Big Pid ", superPidValue);

    if(bird.getPitch() <= -Constants.lowThreshold && bird.getPitch() >= -Constants.highThreshold && startup == 0){
      //Drives forward at 70% speed
      roboMove.commandDrive(0, 0.5*speed, 0.5*speed);
      //|
      //|    If we are below our high threshold, it signifies that we need to drive forward
      //|
      //v
    }else if(bird.getPitch() < -Constants.highThreshold || bird.getPitch() > -Constants.lowThreshold){
      //Drives forward at 20% speed
      roboMove.commandDrive(0, superPidValue, superPidValue);
      //spin(180);
      //Registers that we have made it out of our "0 zone" which implies that it is on the ramp
      startup = 1;
      //|
      //|    If we are above our low threshold, it signifies that we have overshot
      //|
      //v
    }else if(bird.getPitch() <= -Constants.lowThreshold && bird.getPitch() >= -Constants.highThreshold && startup == 1){
      //This should only happen if we have made it to the ramp and are in our "0 zone" again
      //Sets the motors to 0
      roboMove.commandDrive(0, 0, 0);
      led.partyTime();
      //stepCounter += 1;
    }


    // int directionMultiplier = 1;
    // if(forward==false){
    //   directionMultiplier = -1;
    // }
    // //If the pitch is bascially 0 and we haven't gotten out of that zone yet
    // //If elses to find the right position
    // if(bird.getPitch() <= Constants.highThreshold && bird.getPitch() >= Constants.lowThreshold && startup == 0){
    //   //Drives forward at 50% speed
    //   roboMove.commandDrive(0, 0.5*directionMultiplier, 0.5*directionMultiplier);
    //   //If we are below our high threshold, it signifies that we need to drive forward
    // }else if(bird.getPitch() > Constants.highThreshold || bird.getPitch() < Constants.lowThreshold){
    //   roboMove.commandDrive(0, pid.calculate(bird.getPitch(), 0), pid.calculate(bird.getPitch(), 0));
    //   //Registers that we have made it out of our "0 zone" which implies that it is on the ramp
    //   startup = 1;
    //   //If we are above our low threshold, it signifies that we have overshot
    // }else if(bird.getPitch() <= Constants.highThreshold && bird.getPitch() >= Constants.lowThreshold && startup == 1){
    //   //This should only happen if we have made it to the ramp and are in our "0 zone" again
    //   //Sets the motors to 0
    //   roboMove.commandDrive(0, 0, 0);
    // }
  }
  public int getStepCounter()
  {
    return stepCounter;
  }
  //Moves a set distance
  public void goSet(double distanceInInches, double speed){
    double encoderValue = frontLeft.getSelectedSensorPosition();
    double distanceInTicks = distanceInInches * Constants.ticksPerInch;
    
    if(encoderValue < distanceInTicks && distanceInTicks > 0){
      roboMove.commandDrive(0, speed, speed);
    }else if(encoderValue > distanceInTicks && distanceInTicks < 0){
      roboMove.commandDrive(0, -speed, -speed);
    }else{
      roboMove.commandDrive(0, 0, 0);
      stepCounter++;
      frontLeft.setSelectedSensorPosition(0);
    }
  }

  //Checks encoder value
  public boolean checkEncoder(double encoderValue){
    if(frontLeft.get() >= encoderValue){
      return true;
    }else{
      return false;
    }
  }

  //Checks if it's about in the right position
  /*
   * public boolean aroundEqual(double num1, double num2)
  {
    if(num1 > num2-Constants.ABOUT_EQUAL_VAL && num1 < num2 +Constants.ABOUT_EQUAL_VAL)
    {
      return true;
    }
    return false;
  }
   */
  

//Timer (for later use)
/*
 private void doOffset(){
    double startingTime = 0.0;
    offSetTimer.start();
    if(offSetTimer.get() >= 0.5){
      offSetTimer.stop();
      offSetTimer.reset();
      stepCounter += 1;
    }
  }
 */
}


