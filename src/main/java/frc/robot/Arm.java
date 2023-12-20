package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Arm {
    WPI_TalonFX arm_motor_out;
    public ErrorCode Osens;
    private DigitalInput armLimitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ARM_ID);
    private double armEncoderValue;
    private double destinationTicks = 0;
    private double periodicPercentOutput = Constants.armExtendOutput;
    SupplyCurrentLimitConfiguration limitingMethod = new SupplyCurrentLimitConfiguration(true, Constants.ampLimit, 5, 0);

    
    public Arm(WPI_TalonFX arm_motor_out){
        this.arm_motor_out = arm_motor_out;
    }
   
    // Configures the motors
    public void arm_config(){
        arm_motor_out.configFactoryDefault();
        arm_motor_out.configNominalOutputForward(0, Constants.kTimeoutMs);
        arm_motor_out.configNominalOutputReverse(0, Constants.kTimeoutMs);
        arm_motor_out.setInverted(true);
        arm_motor_out.configOpenloopRamp(Constants.RAMP_ARM_ELEVATOR);
        arm_motor_out.configPeakOutputForward(Constants.PEAK_OUTPUT_ARM_FORWARD);
        arm_motor_out.configPeakOutputReverse(Constants.PEAK_OUTPUT_ARM_REVERSE);
        arm_motor_out.setNeutralMode(NeutralMode.Brake);
        arm_motor_out.config_kP(Constants.ARM_PID_SLOT, Constants.ARM_PIDF[0]);
        arm_motor_out.config_kI(Constants.ARM_PID_SLOT, Constants.ARM_PIDF[1]);
        arm_motor_out.config_kD(Constants.ARM_PID_SLOT, Constants.ARM_PIDF[2]);
        arm_motor_out.config_kF(Constants.ARM_PID_SLOT, Constants.ARM_PIDF[3]);
        arm_motor_out.configMotionCruiseVelocity(Constants.MAX_CRUISE_VELOCITY_ARM);
        arm_motor_out.configMotionAcceleration(Constants.MAX_CRUISE_ACCEL_ARM);
        arm_motor_out.configMotionSCurveStrength(0);
    }

    public void setPeriodicPercentOutput(double newPeriodicPercentOutput)
    {
        periodicPercentOutput = newPeriodicPercentOutput;
    }

    public void periodicDriveToValue()
    {
        avoidOverextension(-1);
        if (armEncoderValue > destinationTicks + Constants.ARM_ACCEPTABLE_ERROR)
        {
            if (avoidOverextension(-periodicPercentOutput))
            {
                arm_motor_out.set(ControlMode.MotionMagic, destinationTicks);
            }
        }
        else if (armEncoderValue < destinationTicks - Constants.ARM_ACCEPTABLE_ERROR)
        {
            if (avoidOverextension(periodicPercentOutput))
            {
                arm_motor_out.set(ControlMode.MotionMagic, destinationTicks);
            }
        }
        else 
        {
            arm_motor_out.stopMotor();
        }
    }

    public double getPosition()
    {
        return armEncoderValue;
    }
    // sets the destination of the arm when moving
    public void setDestination(double destinationTicks)
    {
        this.destinationTicks = destinationTicks;
    }
    public void arm_out_max()
    {
        if (armEncoderValue < Constants.MAX_ARM)
        {
            arm_motor_out.set(Constants.armExtendOutput);
        } else 
        {
            arm_motor_out.stopMotor();
        }
    }

    public boolean isInDestinationRange(double evaluationPercent)
    {
        if (armEncoderValue > destinationTicks * evaluationPercent - Constants.ARM_ACCEPTABLE_ERROR && armEncoderValue < destinationTicks * evaluationPercent + Constants.ARM_ACCEPTABLE_ERROR)
        {
            return true;
        }
        return false;
    }

    public void arm_reset_position(){
        
    if (!armLimitSwitch.get()){
            // move the arm down 
            arm_motor_out.set(Constants.armRetractOutput); //change this to a speed
        } else {
            // stop the motor and reset the value to 0
            arm_motor_out.stopMotor();
            arm_setting_reset();
            setDestination(0);
        }
    }

    public void arm_manual_mode_up(double percentOutput){
        if (avoidOverextension(-percentOutput))
        {
            arm_motor_out.set(ControlMode.PercentOutput, -percentOutput);
        }
        
    }
    public void arm_manual_mode_down(double percentOutput){
        if (avoidOverextension(-percentOutput))
        {
            arm_motor_out.set(ControlMode.PercentOutput, -percentOutput);
        }
    }

    public boolean avoidOverextension(double output)
    {
        if (output > 0 && armEncoderValue <= Constants.MAX_ARM)
        {
            SmartDashboard.putBoolean("arm avoid", true);
            return true;
        } else if (output < 0 && !armLimitSwitch.get()) {
            SmartDashboard.putBoolean("arm avoid", true);
            return true;
        } else if (output < 0 && armLimitSwitch.get()) {
            arm_setting_reset();
            return false;
        } else {
            arm_motor_out.stopMotor();
            SmartDashboard.putBoolean("arm avoid", false);
            return false;
        }
        
    }

    public void arm_stop_motor(){
        arm_motor_out.stopMotor();
    }

    // sets the current position of the arm to zero
    public void arm_setting_reset(){
        Osens = arm_motor_out.setSelectedSensorPosition(0);
    }
    
    public void armPeriodic()
    {
        SmartDashboard.putBoolean("limitswitch arm", armLimitSwitch.get());
        armEncoderValue = arm_motor_out.getSelectedSensorPosition();
        SmartDashboard.putNumber("error arm", arm_motor_out.getClosedLoopError());
        SmartDashboard.putNumber("arm encoder", armEncoderValue);
    }

    public boolean getLimitSwitch()
    {
        return armLimitSwitch.get();
    }

    public double getPercentOutput()
    {
        return arm_motor_out.get();
    }

    public void rateLimiter()
    {
        arm_motor_out.configSupplyCurrentLimit(limitingMethod);
    }
}
