package frc.robot;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import java.lang.Math;
import edu.wpi.first.networktables.DoublePublisher;

public class Limelight 
{
    //Network tables are the messenger between limelight and roborio
    private NetworkTable table;

    private XboxController xboxDrive;
    private XboxController xboxAssist;
    private DriveTrain drivetrain;
    private WPI_Pigeon2 pigeon;

    private double x; //x coornates of target (-27 degrees to 27 degrees)
    private double y; //y coornates of target (-20.5 degrees to 20.5 degrees)
    private double area; //how much does the target take up the limelight`s view (0% of image to 100% of image)vv
    private boolean canSee; //checks if it sees target (tape/apriltag)
    private double aprilTagNumber; //reads aprilTag ID
    private int logarthmicRampCounter;
    
    NetworkTableInstance tele = NetworkTableInstance.getDefault();
    NetworkTable tableT = tele.getTable("telemetry");
    DoublePublisher driveXPub;
    DoublePublisher driveYPub;
    DoublePublisher distPub;
    DoublePublisher aprilTagPub;
    DoublePublisher logRampMultiplierPub;



    //constructor uses both drivers control (for now)
    public Limelight(XboxController xboxDrive, XboxController xboxAssist, DriveTrain drivetrain, WPI_Pigeon2 pigeon)
    {
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.xboxDrive = xboxDrive;
        this.xboxAssist = xboxAssist;
        this.drivetrain = drivetrain;
        this.pigeon = pigeon;
        this.logarthmicRampCounter = 0;
       
      
        //tele

        driveXPub = table.getDoubleTopic("drive x").publish();
        driveYPub = table.getDoubleTopic("drive y").publish();
        distPub = table.getDoubleTopic("drive dist").publish();
        aprilTagPub = table.getDoubleTopic("tag").publish();
        logRampMultiplierPub = table.getDoubleTopic("logrampmult").publish();
    }

    public double getLogarithmicRamping()
    {
      logarthmicRampCounter++;
      return Constants.limelightLogRampingMultiplier * (Math.log(logarthmicRampCounter)/Math.log(Constants.limelightLogRampingBase)) / Math.pow(logarthmicRampCounter, Constants.limelightLogRampingInversePower) + Constants.limelightLogRampingAddedConstant;
    }

    public void resetLogarithmicRampCounter()
    {
      logarthmicRampCounter = 0;
    }

    public double getTargetHeight()
    {
      return Constants.heightGoalInches;
      /* TODO: uncomment when using real field heights 
      if (this.getPipeline() != Constants.offPipeline)
      {
        if (this.getPipeline() == Constants.aprilPipeline)
        {
          if (aprilTagNumber == Constants.aprilTagsCompetitor[3])
          {
            return Constants.heightApriltagLoadingInches;
          }
          else if (aprilTagNumber != -1)
          {
            return Constants.heightApriltagGoalInches;
          }
        }
        else if(this.getPipeline() == Constants.lowTapePipeline)
        {
          return Constants.lowTapeHeightInches;
        }
        else if(this.getPipeline() == Constants.highTapePipeline)
        {
          return Constants.lowTapeHeightInches;
        }
      }
      return -1;
      */
    }

    public int getPipeline()
    {
      return (int) table.getEntry("pipeline").getDouble(0);
    }

    //This must be run in a periodic method in Robot.java for the limelight to sense!!
    public void locateTarget()
    {
        aprilTagNumber = table.getEntry("tid").getDouble(-1); //reads aprilTag ID
        x = table.getEntry("tx").getDouble(0); //x coordinates of target (-27 degrees to 27 degrees)
        y = table.getEntry("ty").getDouble(0); //y coordinates of target (-20.5 degrees to 20.5 degrees) (a2)
        area = table.getEntry("ta").getDouble(0); //how much does the target take up the limelight`s view (0% of image to 100% of image)
        canSee = x != 0; //checks if it sees target (tape/apriltag)
    }

    public int getAprilTagNumber()
    {
      return (int) aprilTagNumber;
    }

    public double getX()
    {
      return x;
    }

    public double getY()
    {
      return y;
    }

    //Also runs periodically for the pipeline to change
    public void checkPipelineSwitch()
    {
        if (xboxAssist.getStartButton())
        {
          if (xboxAssist.getPOV() == 0) //up on Dpad
          {
            this.setPipeline(Constants.highTapePipeline);
          }
          else if (xboxAssist.getPOV() == 90) //right on Dpad
          {
            this.setPipeline(Constants.aprilPipeline);
          }
          else if (xboxAssist.getPOV() == 180) //down on Dpad
          {
            this.setPipeline(Constants.lowTapePipeline);
          }
          else if (xboxAssist.getPOV() == 270) //left on Dpad (default)
          {
            this.setPipeline(Constants.offPipeline);
          }
        }
    }

    //A pipeline decides what settings the limelight uses.
    //Different pipelines are used for different things, like having one to sense apriltags and another to look at the highest visible tape
    public void setPipeline(int pipeline)
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }

    //calculates distance (d) from limelight to target (d = (h2-h1) / tan(a1+a2))
    public double calculateDistance() 
    {
        double goalHeight = getTargetHeight();
        return Math.abs((goalHeight - Constants.heightLimelightInches) / Math.tan(Math.toRadians(Constants.limelightMountAngleDegrees + y)));
    }

    //rotates to look directly at the apriltag, and drives in front of it
    //the h wheel is controlled by the x velocity and the others are controlled by y
    /*
    public void moveFromTagToPole(double direction)
    {
      
    }
    */
      /* MATH
       * drivex = cos(limelightx) center wheels
       * drivey = sin(limelightx) tank wheels
       * driveleft = limelightkPTurning * (targetYaw - yaw) / 2
       * 
       * left = driveY + driveleft
       * right = driveY - driveleft
       * center = driveX
       * 
       *  |  |
       *  |--|
       *  |  |
       * 
       */
    public void lineUpToAprilTag()
    {
      if (aprilTagNumber != -1)
      {
        double logRampMultiplier = getLogarithmicRamping();
        double dist = this.calculateDistance();
        SmartDashboard.putNumber("Distance", dist);
        double angleFromHomeToTarget = Math.toRadians(90 + x + pigeon.getYaw());
        double driveX = -1 * dist * Math.cos(angleFromHomeToTarget) * Constants.limelightDriveXDampener * logRampMultiplier;
        double driveY = ((dist * Math.sin(angleFromHomeToTarget)) - Constants.automaticDistanceFromAprilTagInches) * Constants.limelightDriveYDampener * logRampMultiplier;
        double degTurn = -pigeon.getYaw();
        SmartDashboard.putNumber("driveX", driveX);
        SmartDashboard.putNumber("driveY", driveY);
        driveXPub.set(driveX);
        driveYPub.set(driveY);
        distPub.set(dist);
        logRampMultiplierPub.set(logRampMultiplier);
        aprilTagPub.set(aprilTagNumber);

        if (Math.abs(dist - Constants.automaticDistanceFromAprilTagInches) > 2)
        {
          drivetrain.LimelightFOHdrive(driveY, driveX, degTurn);
        }
        else
        {
          drivetrain.LimelightFOHdrive(0, 0, 0);
        }

        //* This should log the details of the lineup method
       
      }

    }

    public void pushToDashboard() //pushes the values to SmartDashboard
    {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putBoolean("CanSeeTarget", canSee);
        SmartDashboard.putNumber("AprilTag ID", aprilTagNumber);
        SmartDashboard.putNumber("d", calculateDistance());
    }
}