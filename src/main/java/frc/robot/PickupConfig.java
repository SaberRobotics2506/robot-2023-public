package frc.robot;

public class PickupConfig
{
    double outputArm;
    double outputElevator;
    double positionArm;
    double positionElevator;

    //Normal
    public PickupConfig(double outputArm, double outputElevator, double positionArm, double positionElevator)
    {
        this.outputArm = outputArm;
        this.outputElevator = outputElevator;
        this.positionArm = positionArm;
        this.positionElevator = positionElevator;
    }

    public double getOutputArm()
    {
        return outputArm;
    }

    public double getOutputElevator()
    {
        return outputElevator;
    }

    public double getPositionArm()
    {
        return positionArm;
    }

    public double getPositionElevator()
    {
        return positionElevator;
    }
}
