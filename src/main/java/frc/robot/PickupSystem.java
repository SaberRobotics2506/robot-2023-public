package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PickupSystem {
    Arm arm;
    Elevator elevator;
    Claw claw;
    XboxController operatorController;
    boolean isRetracted = false;
    boolean retractedFlag = false;
    String pickupMode = "Manual";
    String destinationPositionName;
    String usedConfigType = "step";
    Debouncer aDebouncer = new Debouncer(Constants.OPERATOR_DEBOUNCE_TIME, DebounceType.kFalling);
    Debouncer bDebouncer = new Debouncer(Constants.OPERATOR_DEBOUNCE_TIME, DebounceType.kFalling);
    Debouncer yDebouncer = new Debouncer(Constants.OPERATOR_DEBOUNCE_TIME, DebounceType.kFalling);
    Debouncer dpadDebouncer = new Debouncer(Constants.OPERATOR_DEBOUNCE_TIME, DebounceType.kFalling);

    public PickupSystem(Claw claw, Arm arm, Elevator elevator, XboxController operatorController, int mode_tracker) {
        this.claw = claw;
        this.arm = arm;
        this.elevator = elevator;
        this.operatorController = operatorController;
    }


    public void periodicDrive()
    {
      arm.periodicDriveToValue();
      elevator.periodicDriveToValue();
    }

    public void periodicDashboard()
    {
      SmartDashboard.putNumber("Arm Encoder", arm.getPosition());
      SmartDashboard.putNumber("Elevator Encoder", elevator.getPosition());
      SmartDashboard.putBoolean("Arm Limit Switch", arm.getLimitSwitch());
      SmartDashboard.putBoolean("Elevator Limit Switch", elevator.getLimitSwitch());
    }

    // public void loadBasicPickupConfig(PickupConfig config)
    // {
    //   usedConfigType = "basic";
    //   arm.setDestination(config.getPositionArm());
    //   elevator.setDestination(config.getPositionElevator());
    //   arm.setPeriodicPercentOutput(config.getOutputArm());
    //   elevator.setPeriodicPercentOutput(config.getOutputElevator());
    // }

    public void loadStepBasedPickupConfig(int elevatorPosition, int armPosition)
    {
      usedConfigType = "step";
      elevator.setDestination(elevatorPosition);
      if (elevator.isInDestinationRange(1))
      {
        arm.setDestination(armPosition);
      }
    }

    public void loadStepBasedReturnConfig(int elevatorPosition, int armPosition)
    {
      arm.setDestination(armPosition);
      if (arm.isInDestinationRange(1))
      {
        elevator.setDestination(elevatorPosition);
      }
    }

    // public void loadPositionConfig(PickupConfig config)
    // {
    //   arm.setDestination(config.getPositionArm());
    //   elevator.setDestination(config.getPositionElevator());
    // }

    public void transitPosition()
    {
      // arm.arm_transit();
      // elevator.elevator_transit();
      // if (usedConfigType.equals("basic"))
      // {
      //   loadPositionConfig(Constants.TRANSIT);
      // }
      // else
      if (usedConfigType.equals("step"))
      {
        loadStepBasedReturnConfig(Constants.TRANSIT_ELEVATOR, Constants.TRANSIT_ARM);
      }
      destinationPositionName = "transit";
      SmartDashboard.putString("Pickup System Position", "TRANSIT");
    }

    public void hybridPosition()
    {
      loadStepBasedPickupConfig(Constants.HYBRID_ELEV, Constants.HYBRID_ARM);
      destinationPositionName = "hybrid";
      SmartDashboard.putString("Pickup System Position", "HYBRID");
    }

    public void coneMidPosition()
    {
      loadStepBasedPickupConfig(Constants.MID_CONE_ELEV, Constants.MID_CONE_ARM);
      destinationPositionName = "coneMid";
      SmartDashboard.putString("Pickup System Position", "CONE MID");
    }

    public void coneHighPosition()
    {
      loadStepBasedPickupConfig(Constants.HIGH_CONE_ELEV, Constants.HIGH_CONE_ARM);
      destinationPositionName = "coneHigh";
      SmartDashboard.putString("Pickup System Position", "CONE HIGH");
    }

    public void coneStation()
    {
      loadStepBasedPickupConfig(Constants.STATION_CONE_ELEV, Constants.STATION_CONE_ARM);
      destinationPositionName = "coneStation";
      SmartDashboard.putString("Pickup System Position", "CONE STATION");
    }

    public void cubeMidPosition()
    {
      loadStepBasedPickupConfig(Constants.MID_CUBE_ELEV, Constants.MID_CUBE_ARM);
      destinationPositionName = "cubeMid";
      SmartDashboard.putString("Pickup System Position", "CUBE MID");
    }

    public void cubeHighPosition()
    {
      loadStepBasedPickupConfig(Constants.HIGH_CUBE_ELEV, Constants.HIGH_CUBE_ARM);
      destinationPositionName = "cubeHigh";
      SmartDashboard.putString("Pickup System Position", "CUBE HIGH");
    }

    public void cubeStation()
    {
      loadStepBasedPickupConfig(Constants.STATION_CUBE_ELEV, Constants.STATION_CUBE_ARM);
      destinationPositionName = "cubeStation";
      SmartDashboard.putString("Pickup System Position", "CUBE STATION");
    }

    public void resetPosition()
    {
      arm.arm_reset_position();
      elevator.elevator_reset_limitswitch();
      claw.closeClaw();
      destinationPositionName = "reset";
      SmartDashboard.putString("Pickup System Position", "RESET");
    }

    // Manual Mode Controls
    public void manual_mode() {
      // gets Y-axis Joystick input and converts to arm and elevator movement
      if (operatorController.getLeftY() > Constants.ELEVATOR_MANUAL_MINIMUM_OUTPUT) {
          elevator.elevator_manual_mode_down(operatorController.getLeftY() * Constants.MANUAL_ELEVATOR_DAMPENER);
      }else if (operatorController.getLeftY() < -Constants.ELEVATOR_MANUAL_MINIMUM_OUTPUT) {
          elevator.elevator_manual_mode_up(operatorController.getLeftY() * Constants.MANUAL_ELEVATOR_DAMPENER);
      }else if (operatorController.getRightY() < -Constants.ARM_MANUAL_MINIMUM_OUTPUT) {
          arm.arm_manual_mode_down(operatorController.getRightY() * Constants.MANUAL_ARM_DAMPENER);
      }else if (operatorController.getRightY() > Constants.ARM_MANUAL_MINIMUM_OUTPUT) {
          arm.arm_manual_mode_up(operatorController.getRightY() * Constants.MANUAL_ARM_DAMPENER); 
      }else if (operatorController.getLeftStickButton()){
        elevator.elevator_reset_limitswitch();
      } else if (operatorController.getRightStickButton()) {
        arm.arm_reset_position();
      } else {
          elevator.elevator_stop_motor();
          arm.arm_stop_motor();
      }
      //bumpers control wrist
      if (operatorController.getLeftBumper())
      {
        claw.openClaw();
      }
      else if (operatorController.getRightBumper())
      {
        claw.closeClaw();
      }
      
      if (operatorController.getAButtonPressed())
      {
        if (claw.isOpen())
        {
          claw.closeClaw();
        }
        else
        {
          claw.openClaw();
        }
      }
    }

    public void controlPickupSystem()
    {
      // This section controls all the claw positions.
      // Left and right triggers are cone and cube control.
      // B and Y are medium and high presets.
      // The double substation position is D-PAD down.

      // If there is no combination of trigger and position button,
      // the arm and elevator will return to transit position.
      if (operatorController.getLeftBumper())
      {
        claw.openClaw();
      }
      else if (operatorController.getRightBumper())
      {
        claw.closeClaw();
      }

      if (operatorController.getLeftTriggerAxis() > Constants.TRIGGER_LEEWAY)
      {
        if (aDebouncer.calculate(operatorController.getAButton()))
        {
          hybridPosition();
        } 
        else if (bDebouncer.calculate(operatorController.getBButton()))
        {
          coneMidPosition();
        }
        else if (yDebouncer.calculate(operatorController.getYButton()))
        {
          coneHighPosition();
        }
        else if (dpadDebouncer.calculate(operatorController.getPOV() == Constants.DPAD_DOWN))
        {
          coneStation();
        }
        else 
        {
            transitPosition();
        }
      }
      else if (operatorController.getRightTriggerAxis() > Constants.TRIGGER_LEEWAY)
      {
        if (aDebouncer.calculate(operatorController.getAButton()))
        {
          hybridPosition();
        } 
        else if (bDebouncer.calculate(operatorController.getBButton()))
        {
          cubeMidPosition();
        }
        else if (yDebouncer.calculate(operatorController.getYButton()))
        {
          cubeHighPosition();
        }
        else if (dpadDebouncer.calculate(operatorController.getPOV() == Constants.DPAD_DOWN))
        {
          cubeStation();
        }
        else 
        {
            transitPosition();
        }
      }
      else 
      {
          transitPosition();
      }
    }

    DoublePublisher armEncoderTelemetry;
    DoublePublisher elevatorEncoderTelemetry;
    DoublePublisher armPercentOutputTelemetry;
    DoublePublisher elevatorPercentOutputTelemetry;
    BooleanPublisher armLimitSwitchTelemetry;
    BooleanPublisher elevatorLimitSwitchTelemetry;
    
    public void pickupSystemTelemetrySetup()
    {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();
      NetworkTable table = instance.getTable("pickup");

      armEncoderTelemetry = table.getDoubleTopic("arm encoder value").publish();
      elevatorEncoderTelemetry = table.getDoubleTopic("elevator encoder value").publish();
      armLimitSwitchTelemetry = table.getBooleanTopic("arm limit switch").publish();
      elevatorLimitSwitchTelemetry = table.getBooleanTopic("elevator limit switch").publish();
      armPercentOutputTelemetry = table.getDoubleTopic("arm percent output").publish();
      elevatorPercentOutputTelemetry = table.getDoubleTopic("elevator percent output").publish();
    }

    public void collectPickupSystemTelemetry()
    {
      armEncoderTelemetry.set(arm.getPosition());
      elevatorEncoderTelemetry.set(elevator.getPosition());
      armLimitSwitchTelemetry.set(arm.getLimitSwitch());
      elevatorLimitSwitchTelemetry.set(elevator.getLimitSwitch());
      armPercentOutputTelemetry.set(arm.getPercentOutput());
      elevatorPercentOutputTelemetry.set(elevator.getPercentOutput());
    }
}
