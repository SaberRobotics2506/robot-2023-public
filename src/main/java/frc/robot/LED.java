package frc.robot;


import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  Spark blinkin;
  Rangefinder ultrasonic;
  XboxController driver;
  XboxController operator;
  DriveTrain drivetrain;
  DoublePublisher LEDTelemetry;

    public LED(Rangefinder ultrasonic,XboxController driver, XboxController operator, DriveTrain driveTrain)
    {
    this.ultrasonic = ultrasonic;
    this.driver = driver;
    this.operator = operator;
    this.drivetrain = driveTrain;
    }
  public void blinkin(int pwmPort) {
    blinkin = new Spark(pwmPort);
  }
  public void LEDTelemetrySetup()
  {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("LEDs");
    LEDTelemetry = table.getDoubleTopic("LED Value").publish();
  }
  public void LEDTelemetryCollection()
  {
    LEDTelemetry.set(blinkin.get());
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public void set(LEDVALUES val) {
    if ((val.getValue() >= -1.0) && (val.getValue() <= 1.0)) {
      blinkin.set(val.getValue());
    }
  }
  public void partyTime()
  {
    set(LEDVALUES.PARTY);
  }
  public void changeColorsAuto(int stepOn)
  {
    if(stepOn == 0)
    {
        set(LEDVALUES.RED);
    }
    else if(stepOn == 1)
    {
        set(LEDVALUES.ORANGE);
    }
    else if(stepOn == 2)
    {
        set(LEDVALUES.YELLOW);
    }
    else if(stepOn == 3)
    {
        set(LEDVALUES.GREEN);
    }
    else if(stepOn == 4)
    {
        set(LEDVALUES.BLUE);
    }
    else if(stepOn == 5)
    {
        set(LEDVALUES.VIOLET);
    }
    else if(stepOn == 6)
    {
        set(LEDVALUES.WHITE);
    }
    else if(stepOn == 7)
    {
        set(LEDVALUES.GREY);
    }
    else if(stepOn == 8)
    {
        set(LEDVALUES.HOTPINK);
    }
    else
    {
        set(LEDVALUES.ULTRAGOING);
    }
  }
  public void changeColors()
  {
    if(Timer.getMatchTime() > 30 || Timer.getMatchTime() < 25)
    {
        if(driver.getAButton())
        {
            if(ultrasonic.isAtDestination())
            {
                set(LEDVALUES.RAINBOW);
            }
            else if(!ultrasonic.isAtDestination())
            {
                set(LEDVALUES.ULTRAGOING);
            }
        }
        else if(drivetrain.slowToggleFlag)
            {
                set(LEDVALUES.SLOWMODE);
            }
        else if(operator.getLeftTriggerAxis() >= .1)
        {
            set(LEDVALUES.CONECOLOR);
        }
        else if(operator.getRightTriggerAxis() >= .1)
        {
            set(LEDVALUES.CUBECOLOR);
        }
        else
        {
           set(LEDVALUES.FASTMODE);
        }
    }
    else
    {
        set(LEDVALUES.GAMEENDINGRED);
    }
        
      }
    }

enum LEDVALUES{

    FASTMODE (.75),
    LARSON(-.35), //Sky blue
    SLOWMODE(.61),
    RAINBOW(-.99),
    BUSY(.15),
    ULTRAGOING(-.09),
    CONECOLOR(.65),
    CUBECOLOR(.91),
    PARTY(-.97),
    GAMEENDINGBLUE(-.95),
    GAMEENDINGRED(-.93),
    RED(.61),
    ORANGE(.65),
    YELLOW(.69),
    GREEN(.77),
    BLUE(.87),
    VIOLET(.91),
    WHITE(.93),
    GREY(.95),
    HOTPINK(.57);

    double value;
    double color1;
    double color2;
    LEDVALUES(double value, double color1, double color2)
    {
        this.value = value;
        this.color1 = color1;
        this.color2 = color2;
    }
    LEDVALUES(double value, double color1)
    {
        this.value = value;
        this.color1 = color1;
    }
    LEDVALUES(double value)
    {
        this.value = value;
    }
    public double getValue()
    {
        return value;
    }
    public double getColor1()
    {
        return color1;
    }
    public double getColor2()
    {
        return color2;
    }
}