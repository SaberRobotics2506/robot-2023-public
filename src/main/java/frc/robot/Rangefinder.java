package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Rangefinder {
    private Ultrasonic ultrasonic;
    private DriveTrain drive;
    private MedianFilter ultrasonicFilter;
    private PIDController pid;
    private double distance;

    public Rangefinder(Ultrasonic ultrasonic, DriveTrain drive)
    {
        this.ultrasonic = ultrasonic;
        this.drive = drive;
        this.pid = new PIDController(Constants.ULTRASONIC_KP,Constants.ULTRASONIC_KI, Constants.ULTRASONIC_KD);
        pid.setTolerance(Constants.ULTRASONIC_DRIVE_ALLOWED_ERROR_INCHES);

        ultrasonicFilter = new MedianFilter(5);
    }

    public void enable()
    {
        ultrasonic.setEnabled(true);
    }

    public static void setAutomaticMode()
    {
        Ultrasonic.setAutomaticMode(true);
    }

    public double getDistanceInches()
    {
        double distance = ultrasonicFilter.calculate(ultrasonic.getRangeInches());
        SmartDashboard.putNumber("ultra distance", distance);
        this.distance = distance;
        return distance;
    }

    public void driveToDistance(double targetDistance)
    {
        double currentDistance = getDistanceInches();
        if (currentDistance > Constants.ULTRASONIC_MAX_START_DIST + Constants.ULTRASONIC_DRIVE_DESIRED_DISTANCE)
        {
            return;
        }
        pid.setSetpoint(targetDistance);
        double driveOutput = clamp(Constants.ULTRASONIC_MIN_OUTPUT, Constants.ULTRASONIC_MAX_OUTPUT, pid.calculate(getDistanceInches()));
        SmartDashboard.putNumber("ultra drive", driveOutput);
        if (currentDistance > targetDistance + Constants.ULTRASONIC_DRIVE_ALLOWED_ERROR_INCHES)
        {
            drive.unsquaredCommandDrive(driveOutput, driveOutput);
        }
        else if (currentDistance < targetDistance - Constants.ULTRASONIC_DRIVE_ALLOWED_ERROR_INCHES)
        {
            drive.unsquaredCommandDrive(-driveOutput, -driveOutput);
        }
        else 
        {
            drive.stopMotors();
        }
    }


    private double clamp(double min, double max, double value)
    {
        return Math.max(min, Math.min(max, value));
    }

    public boolean isInRange()
    {
        return (Constants.ULTRASONIC_TOTAL_MAX_RANGE > getDistanceInches());
    }

    public boolean isAtDestination()
    {
        return (getDistanceInches() > Constants.ULTRASONIC_DRIVE_DESIRED_DISTANCE - Constants.ULTRASONIC_DRIVE_ALLOWED_ERROR_INCHES && getDistanceInches() < Constants.ULTRASONIC_DRIVE_DESIRED_DISTANCE + Constants.ULTRASONIC_DRIVE_ALLOWED_ERROR_INCHES);
    }

    DoublePublisher distanceTelemetry;
    public void rangefinderTelemetrySetup()
    {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("ultrasonic");

        distanceTelemetry = table.getDoubleTopic("ultrasonic distance").publish();
    }

    public void collectRangefinderTelemetry()
    {
        distanceTelemetry.set(distance);
    }
}

