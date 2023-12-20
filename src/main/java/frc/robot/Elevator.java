package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    WPI_TalonFX arm_motor_right; //port 1 (5)
    WPI_TalonFX arm_motor_left; // port 2 (6)
    ErrorCode Lsens;
    ErrorCode Rsens;
    DigitalInput elevatorLimitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ELEVATOR_ID);
    DigitalInput irBreakbeam = new DigitalInput(Constants.breakbeamID);
    Arm arm;
    private double destinationTicks = 0;
    private double elevatorPosition = 0;
    double periodicPercentOutput = Constants.elevatorExtendOutput;
    double positionOffset = 0;

    //rate limiter
    SupplyCurrentLimitConfiguration limitingMethod = new SupplyCurrentLimitConfiguration(true, Constants.ampLimit, 5, 0);



    
    // These numbers are used for the configuration of the motors
    
    public double getPosition()
    {
        return elevatorPosition;
    }

    public Elevator( WPI_TalonFX arm_motor_right, WPI_TalonFX arm_motor_left){
        this.arm_motor_right = arm_motor_right;
        this.arm_motor_left = arm_motor_left;
    }


    public void setDestination(double destinationTicks)
    {
        this.destinationTicks = destinationTicks;
    }

    public void setPeriodicPercentOutput(double newPeriodicPercentOutput)
    {
        periodicPercentOutput = newPeriodicPercentOutput;
    }

    public void periodicDriveToValue()
    {
        checkEncoderReset(-1);
        if (elevatorPosition > destinationTicks + Constants.ELEVATOR_ACCEPTABLE_ERROR)
        {
            if (avoidOverextension(-periodicPercentOutput))
            {
                arm_motor_right.set(ControlMode.MotionMagic, destinationTicks);
            }
        }
        else if (elevatorPosition < destinationTicks - Constants.ELEVATOR_ACCEPTABLE_ERROR)
        {
            if (avoidOverextension(periodicPercentOutput))
            {
                arm_motor_right.set(ControlMode.MotionMagic, destinationTicks);
            }
        }
        else 
        {
            arm_motor_right.stopMotor();
        }
    }

    // Configures the motors
    public void elevator_config(){
        arm_motor_right.configFactoryDefault();
        arm_motor_right.configNominalOutputForward(0, Constants.kTimeoutMs);
        arm_motor_right.configNominalOutputReverse(0, Constants.kTimeoutMs);
        arm_motor_right.setInverted(true);
        arm_motor_right.configOpenloopRamp(Constants.RAMP_ARM_ELEVATOR);
        arm_motor_right.setNeutralMode(NeutralMode.Brake);
        arm_motor_right.configPeakOutputForward(Constants.PEAK_OUTPUT_ELEV_FORWARD);
        arm_motor_right.configPeakOutputReverse(Constants.PEAK_OUTPUT_ELEV_REVERSE);

        arm_motor_left.configFactoryDefault();
        arm_motor_left.configNominalOutputForward(0, Constants.kTimeoutMs);
        arm_motor_left.configNominalOutputReverse(0, Constants.kTimeoutMs);
        arm_motor_left.follow(arm_motor_right);
        arm_motor_left.setInverted(InvertType.OpposeMaster);
        arm_motor_left.configOpenloopRamp(Constants.RAMP_ARM_ELEVATOR);
        arm_motor_left.configPeakOutputForward(Constants.PEAK_OUTPUT_ELEV_FORWARD);
        arm_motor_left.configPeakOutputReverse(Constants.PEAK_OUTPUT_ELEV_REVERSE);
        arm_motor_left.configMotionSCurveStrength(0);
        
        arm_motor_right.config_kP(Constants.ELEVATOR_PID_SLOT, Constants.ELEV_PIDF[0]);
        arm_motor_right.config_kI(Constants.ELEVATOR_PID_SLOT, Constants.ELEV_PIDF[1]);
        arm_motor_right.config_kD(Constants.ELEVATOR_PID_SLOT, Constants.ELEV_PIDF[2]);
        arm_motor_right.config_kF(Constants.ELEVATOR_PID_SLOT, Constants.ELEV_PIDF[3]);
        arm_motor_right.configMotionCruiseVelocity(Constants.MAX_CRUISE_VELOCITY_ELEV);
        arm_motor_right.configMotionAcceleration(Constants.MAX_CRUISE_ACCEL_ELEV);
        arm_motor_right.configMotionSCurveStrength(0);

        arm_motor_left.setNeutralMode(NeutralMode.Brake);
    }
    // resets elevator to inside the frame perimeter
    public void elevator_reset_limitswitch(){
        if (!elevatorLimitSwitch.get()){
            arm_motor_right.set(Constants.armRetractOutput);
        } else {
            arm_motor_right.stopMotor();
            positionOffset = 0;
            elevator_setting_reset();
        }
    }
/* 
    public void elevator_reset_breakbeam()
    {
        if (irBreakbeam.get() && getVelocity() < 0)
        {
            arm_motor_right.setSelectedSensorPosition(0);
            positionOffset = Constants.TRANSIT_ELEVATOR;
        }
    }
*/
public boolean isInDestinationRange(double evaluationPercent)
{
    if (elevatorPosition > destinationTicks * evaluationPercent - Constants.ELEVATOR_ACCEPTABLE_ERROR && elevatorPosition < evaluationPercent * destinationTicks + Constants.ELEVATOR_ACCEPTABLE_ERROR)
    {
        return true;
    }
    return false;
}
    public void elevator_manual_mode_up(double percentOutput){
        if (avoidOverextension(-percentOutput))
        {
            arm_motor_right.set(ControlMode.PercentOutput, -percentOutput); 
        }
    }
    public void elevator_manual_mode_down(double percentOutput){
        if (avoidOverextension(-percentOutput))
        {
            arm_motor_right.set(ControlMode.PercentOutput, -percentOutput); 
        }
    }
    // stops elevator motors
    public void elevator_stop_motor(){
        arm_motor_right.stopMotor();
    }

    // extends elevator to maximum position
    public void elevator_up_max()
    {
        if (elevatorPosition < Constants.MAX_ELEVATOR)
        {
            arm_motor_right.set(Constants.elevatorExtendOutput);
        } else 
        {
            arm_motor_right.stopMotor();
        }
    }
    // sets the current elevator position to zero
    public void elevator_setting_reset(){
        Lsens = arm_motor_right.setSelectedSensorPosition(0);
    }
    // gets the position of the arm
    public double get_pos(){
        double pos = arm_motor_right.getSelectedSensorPosition();
        return pos;
    }

    public boolean avoidOverextension(double output)
    {
        if (output > 0 && elevatorPosition <= Constants.MAX_ELEVATOR)
        {
            return true;
        } else if (output < 0 && !elevatorLimitSwitch.get())
        {
            return true;
        }else if (output < 0 && elevatorLimitSwitch.get()) {
            elevator_setting_reset();
            return false;
        } else{
            arm_motor_right.stopMotor();
            return false;  
        }

    }

    public void checkEncoderReset(double output)
    {
        if (output < 0 && elevatorLimitSwitch.get()) {
            elevator_setting_reset();
        }
    }
    
    public void elevatorPeriodic()
    {
        checkEncoderReset(-periodicPercentOutput);
        elevatorPosition = arm_motor_right.getSelectedSensorPosition() + positionOffset;
        SmartDashboard.putNumber("elevator encoder", elevatorPosition);
        SmartDashboard.putBoolean("limitswitch elevator", elevatorLimitSwitch.get());
        SmartDashboard.putNumber("elev error", arm_motor_right.getClosedLoopError());

    }

    public boolean getLimitSwitch()
    {
        return elevatorLimitSwitch.get();
    }

    public double getPercentOutput()
    {
        return arm_motor_right.get();
    }

    public void rateLimiter()
    {
        arm_motor_left.configSupplyCurrentLimit(limitingMethod);
        arm_motor_right.configSupplyCurrentLimit(limitingMethod);
    }
}

