package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import java.lang.Math;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveTrain {
    

    // define motors, controller, and variables
     XboxController controller;
    
     WPI_TalonFX frontLeft;
     WPI_TalonFX backRight;
     WPI_TalonFX backLeft;
     WPI_TalonFX frontRight;
     //WPI_TalonFX middleWheel;
     WPI_Pigeon2 pigeon;
     DifferentialDrive difDrive;

    DoublePublisher frontLeftTelemetry;
    DoublePublisher backLeftTelemetry;
    DoublePublisher frontRightTelemetry;
    DoublePublisher backRightTelemetry;
    //DoublePublisher middleWheelTelemetry;
    DoublePublisher pigeonPitchTelemetry;

    DoublePublisher frontLeftTelemetryPos;
    DoublePublisher backLeftTelemetryPos;
    DoublePublisher frontRightTelemetryPos;
    DoublePublisher backRightTelemetryPos;
    //DoublePublisher middleWheelTelemetryPos;
    


     double leftX;
     double leftY;
     double rightX;
     double rightY;
     boolean slowToggleFlag;
     double speedMultiplierOuter;
     double turnSpeedMult;
     // object for rate limiter
     SlewRateLimiter turnSlew = new SlewRateLimiter(7);
     SlewRateLimiter driveSlew = new SlewRateLimiter(2);
     SupplyCurrentLimitConfiguration limitingMethod = new SupplyCurrentLimitConfiguration(true, Constants.ampLimit, 5, 0);

    public DriveTrain(DifferentialDrive difDrive, XboxController controller, WPI_TalonFX frontLeft, WPI_TalonFX backRight, WPI_TalonFX frontRight, WPI_TalonFX backLeft, WPI_Pigeon2 pigeon) 
    {
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        //this.middleWheel = middleWheel;
        this.difDrive = difDrive;
        this.controller = controller;
        this.pigeon = pigeon;
        slowToggleFlag = false;
        speedMultiplierOuter = 1;
        turnSpeedMult = 1;
    }

    /*
     * ARCADE DRIVE
     * 
     * left thumbstick controls forward/backward and right thumbstick controls left/right
     * 
     * 
     * CONTROLS
     * forward - y button
     * backward - a button
     * turning - right thumbstick
     */
        





        // ARCADE DRIVE

        public void Arcade_Drive() 
        {
            double turn;
            boolean turnInPlace;
            // double arcadeTurnSlower;
              // if(slowToggleFlag)
              // {
              //     arcadeTurnSlower = Constants.slowModearcadeTurnDampener * speedMultiplierOuter;
              // }
              // else{
              //   arcadeTurnSlower = Constants.arcadeTurnDampener * speedMultiplierOuter;
              // }
              //   if(yDeadzone(controller.getLeftX()) && xDeadzone(controller.getRightX()))
              //   {
              //     arcadeTurnSlower = .5;
              //   }
            
            turnInPlace = !(Math.abs(controller.getLeftY()) > .1);
            turn = ((controller.getRightX() / Constants.arcadeTurnDampener) * -1) * turnSpeedMult;
            double throttle = -1 * controller.getLeftY() * speedMultiplierOuter;
            
            // throttle = driveSlew.calculate(controller.getLeftY() * -1 * speedMultiplierOuter);
            //double turnOut = controller.getRightX();
            //double turning =  turnSlew.calculate(turnOut * -1 *arcadeTurnSlower);
            //SmartDashboard.putNumber("throttle arcade", throttle);
            //SmartDashboard.putNumber("turning value", turning);
            difDrive.curvatureDrive(throttle, turn, turnInPlace);
        }

    public void coastMode()
    {
      frontLeft.setNeutralMode(NeutralMode.Coast);
      frontRight.setNeutralMode(NeutralMode.Coast);
      backLeft.setNeutralMode(NeutralMode.Coast);
      backRight.setNeutralMode(NeutralMode.Coast);
    }

    public void brakeMode()
    {
      frontLeft.setNeutralMode(NeutralMode.Brake);
      frontRight.setNeutralMode(NeutralMode.Brake);
      backLeft.setNeutralMode(NeutralMode.Brake);
      backRight.setNeutralMode(NeutralMode.Brake);
    }

    public void stopMotors()
    {
      frontLeft.stopMotor();
      frontRight.stopMotor();
      backLeft.stopMotor();
      backRight.stopMotor();
      //middleWheel.stopMotor();
    }

/*TANK DRIVE
 * 
 * one joystick conrtols the lefts two wheels. the other joystick controls the right side wheels. 
 */


    public void tankDrive(){
        difDrive.tankDrive((controller.getLeftY() * -1) * speedMultiplierOuter, (controller.getRightY() * -1
        ) * speedMultiplierOuter);

    }

    public void commandDrive(double xGo, double yLeftGo,double yRightGo)
    {
      difDrive.tankDrive(yLeftGo,yRightGo);
      //middleWheel.set(xGo);
    }


    public void pigeonConfig() {
      pigeon.configFactoryDefault();
      Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
      pigeonConfig.MountPoseYaw = -90;
      pigeon.configAllSettings(pigeonConfig);
    }

    public void unsquaredCommandDrive(double yLeft, double yRight)
    {
      difDrive.tankDrive(yLeft, yRight, false);
    }


    /*HDRIVE
     * 
     * left joystick moves the robot, forward, backwards, left, and right. 
     * the right joystick turns the robot left and right. 
     * 
     * Note: Turning is first priority for hdrive, if both forwards and turn joysticks are being used, the robot will turn and not go forwards. 
     * 
     */
    public void hDrive() {
        leftY =  -1 * controller.getLeftY();
        leftX =  controller.getLeftX();
        rightY = -1 * controller.getRightY();
        if(!xDeadzone(leftX))
        {
          leftX = 0;
        }
        difDrive.arcadeDrive(controller.getLeftY() * -1 * speedMultiplierOuter, controller.getRightX() * -1 * speedMultiplierOuter);
        // middleWheel.set(leftX * speedMultiplierH);
        

    }
    public void resetEncoders()
    {
      frontLeft.setSelectedSensorPosition(0);
      frontRight.setSelectedSensorPosition(0);
      backLeft.setSelectedSensorPosition(0);
      backRight.setSelectedSensorPosition(0);
    }
    // reset motors and sets ramp up time
        public void  resetMotors() {
        frontLeft.configFactoryDefault();
        frontRight.configFactoryDefault();
        backLeft.configFactoryDefault();
        backRight.configFactoryDefault();
        //middleWheel.configFactoryDefault();
        //drive.resetMotors();
      frontLeft.setInverted(TalonFXInvertType.Clockwise);
      backLeft.setInverted(TalonFXInvertType.Clockwise);
      frontRight.setInverted(TalonFXInvertType.CounterClockwise);
      backRight.setInverted(TalonFXInvertType.CounterClockwise);
      //middleWheel.setInverted(TalonFXInvertType.CounterClockwise);

      frontRight.setNeutralMode(NeutralMode.Brake);
      frontLeft.setNeutralMode(NeutralMode.Brake);
      backLeft.setNeutralMode(NeutralMode.Brake);
      backRight.setNeutralMode(NeutralMode.Brake);
      //middleWheel.setNeutralMode(NeutralMode.Brake);
      //ramps
      frontRight.configOpenloopRamp(Constants.outerMotorRamp);
      frontLeft.configOpenloopRamp(Constants.outerMotorRamp);
      backRight.configOpenloopRamp(Constants.outerMotorRamp);
      backLeft.configOpenloopRamp(Constants.outerMotorRamp);
      //middleWheel.configOpenloopRamp(Constants.outerMotorRamp);
        }

    

    /*
     * FIELD-ORIENTED DRIVE
     */
    //  public void deadFOHdrive()
    //  {
    //     if(controller.getStartButtonPressed())
    //     {
    //       resetHome();
    //     }
    //     double yVelocity  = controller.getLeftY();
    //     double xVelocity = controller.getLeftX();

    //     if(!xDeadzone(xVelocity) && yDeadzone(yVelocity))
    //     {
    //       FOHdrive(yVelocity, 0);
    //     }
    //     else if(xDeadzone(xVelocity) && !yDeadzone(yVelocity))
    //     {
    //       FOHdrive(0, xVelocity);
    //     }
    //     else if(!xDeadzone(xVelocity) && !yDeadzone(yVelocity))
    //     {
    //       FOHdrive(0, 0);
    //     }
    //     else{
    //       FOHdrive(yVelocity, xVelocity);
    //     }
    //  }
     public void resetHome(){
        pigeon.setYaw(0);
        
      }
      public boolean xDeadzone(double xPlace)
      {
          if(Math.abs(xPlace) > Constants.xDeadZone)
          {
              return true;
          }
          else
          {
              return false;
          }
  
      }
    public boolean yDeadzone(double yPlace)
    {
        if(Math.abs(yPlace) > Constants.yDeadZone)
        {
          return true;
        }
        else
        {
          return false;
        }
          
  
      }
      //   public void FOHdrive(double yVelocity, double xVelocity)
      //     {
      //       /*
      //       * This code is a simplified version of a few trigonometry steps
      //       *
      //       * STEP ONE -------------
      //       * First, we use Pythagoreans theorem to get the length of the hypotenuse (the distance from the origin of (0,0))
      //       * STEP TWO -------------
      //       * Then, since we know all of the side lengths, we can find the angle of the angle that the controller is looking
      //       // TODO: explain what it means for controller to look
      //       * compared to the x right axis
      //       * STEP THREE -----------
      //       * Once we know the angle and the length of the hypotenuse, then we can add the angle of offset (the yaw of the robot
      //       * from the home position) to the angle that the controller is at.
      //       * STEP FOUR ------------
      //       * Since we know where the new input point is located based on the newly made angle, we can then use the sine of the angle
      //       * multiplied by the hypotenuse and the cosine of the angle multiplied by the hypotenuse to find the Y and X inputs respectively
      //       */

      //       double appliedXVelocity = (((xVelocity)*(Math.cos(Math.toRadians(pigeon.getYaw())))) - ((yVelocity)*(Math.sin(Math.toRadians(pigeon.getYaw())))));
      //       double appliedYVelocity = (((xVelocity)*(Math.sin(Math.toRadians(pigeon.getYaw())))) + ((yVelocity)*(Math.cos(Math.toRadians(pigeon.getYaw())))));
      //       difDrive.feed();

      //             // This displays the rotated position of the controller on the smart dashboard
      //             SmartDashboard.putNumber("rotated X axis", appliedXVelocity);
      //             SmartDashboard.putNumber("rotated Y axis", appliedYVelocity);
                  
  
      //            /*
      //             * This takes the rotated X velocity that we calcuated by rotating the inputs and then applies it
      //             * as the speed for the H-Motor 
      //             */
      //             //TODO: change this to work with the new robot's different gear ratio
      //             middleWheel.set((appliedXVelocity * Constants.hMotorSpeed) *speedMultiplierH);
      //             /* This takes the rotated Y velocity that we calculated above and then implements it with a turning
      //                 * offset so that -100% of turning power is only 50% of the motor's power so that it can turn while
      //                 * still moving in a dominant direction allowing it to spin and arc while moving.
      //             */
      //                 frontLeft.set((-(Constants.outerMotorSpeed * appliedYVelocity- (Constants.turnOffset * controller.getRightX()))) * speedMultiplierOuter);
      //                 backLeft.set((-(Constants.outerMotorSpeed * appliedYVelocity- (Constants.turnOffset * controller.getRightX())))* speedMultiplierOuter);
      //                 frontRight.set((-(Constants.outerMotorSpeed * appliedYVelocity+ (Constants.turnOffset * controller.getRightX())))* speedMultiplierOuter);
      //                 backRight.set((-(Constants.outerMotorSpeed * appliedYVelocity+ (Constants.turnOffset * controller.getRightX())))* speedMultiplierOuter);
      //     }
      // // IDs
      // // top right - 4
      // // top left - 14
      // // bottom right - 3
      // // bottom left - 15
      // // h motor - 6
      //   /*
      //   * The same as FOHdrive, but for limelight!
      //   * this allows the user to input the y and x velocity (based off home) and the amount of degrees the robot is away
      //   * from the rotation it should be at for the limelight  
      //   */
        public void LimelightFOHdrive(double yVelocity, double xVelocity, double degToMove){

            double appliedXVelocity = ((xVelocity)*(Math.cos(Math.toRadians(pigeon.getYaw())))) - ((yVelocity)*(Math.sin(Math.toRadians(pigeon.getYaw()))));
            double appliedYVelocity = ((xVelocity)*(Math.sin(Math.toRadians(pigeon.getYaw())))) + ((yVelocity)*(Math.cos(Math.toRadians(pigeon.getYaw()))));
            double turningValue = degToMove / 180;
            //middleWheel.set(Constants.hPercentPower * appliedXVelocity);
            frontLeft.set(appliedYVelocity - (turningValue));
            frontRight.set(appliedYVelocity + (turningValue));
        }
        

        public void drivetrainTelemetryTable(){
          NetworkTableInstance inst = NetworkTableInstance.getDefault();
          NetworkTable table = inst.getTable("drivetrain");

          frontLeftTelemetry = table.getDoubleTopic("Front left motor Output").publish();
          frontRightTelemetry = table.getDoubleTopic("Front right motor Output").publish();
          backLeftTelemetry = table.getDoubleTopic("Back left motor Output").publish();
          backRightTelemetry = table.getDoubleTopic("Back right motor Output").publish();
          //middleWheelTelemetry = table.getDoubleTopic("Middle wheel Output").publish();
          
          frontLeftTelemetryPos = table.getDoubleTopic("Front left motor Position").publish();
          frontRightTelemetryPos = table.getDoubleTopic("Front right motor Position").publish();
          backLeftTelemetryPos = table.getDoubleTopic("Back left motor Position").publish();
          backRightTelemetryPos = table.getDoubleTopic("Back right motor Position").publish();
          //middleWheelTelemetryPos = table.getDoubleTopic("Middle wheel Position").publish();

          pigeonPitchTelemetry = table.getDoubleTopic("Pitch").publish();


        }
        public void checkSlowToggle()
        {
          if(controller.getYButtonPressed())
          {
            if(!slowToggleFlag)
            {
              slowToggleFlag = true;
              turnSpeedMult = Constants.slowTurnSpeedMult;
              speedMultiplierOuter = Constants.slowPercentSpeedOuter;
            }
            else{
              slowToggleFlag = false;
              turnSpeedMult = Constants.fastTurnSpeedMult;
              speedMultiplierOuter = Constants.fastPercentSpeedOuter;
            }
          }
          SmartDashboard.putBoolean("slow", slowToggleFlag);
          SmartDashboard.putNumber("speed mult H", turnSpeedMult);
        }
        public void collectDrivetrainTelemetry(){
          
        double frontleftValue = frontLeft.get();
        double frontrightValue = frontRight.get();
        double backleftValue = backLeft.get();
        double backrightValue = backRight.get();
        //double middlewheelValue = middleWheel.get();


          frontLeftTelemetry.set(frontleftValue);
          frontRightTelemetry.set(frontrightValue);
          backLeftTelemetry.set(backleftValue);
          backRightTelemetry.set(backrightValue);
          //middleWheelTelemetry.set(middlewheelValue);
          pigeonPitchTelemetry.set(pigeon.getPitch());

          frontLeftTelemetryPos.set(frontLeft.getSelectedSensorPosition());
          frontRightTelemetryPos.set(frontRight.getSelectedSensorPosition());
          backLeftTelemetryPos.set(backLeft.getSelectedSensorPosition());
          backRightTelemetryPos.set(backRight.getSelectedSensorPosition());
          //middleWheelTelemetryPos.set(middleWheel.getSelectedSensorPosition());

  
        }


    public void rateLimiter()
    {
      frontLeft.configSupplyCurrentLimit(limitingMethod);
      frontRight.configSupplyCurrentLimit(limitingMethod);
      backLeft.configSupplyCurrentLimit(limitingMethod);
      backRight.configSupplyCurrentLimit(limitingMethod);
      //middleWheel.configSupplyCurrentLimit(limitingMethod);
    }
    
}
