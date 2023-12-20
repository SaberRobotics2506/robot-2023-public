package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;


public class Claw {
    DoubleSolenoid solenoid;
    boolean isSolenoidForward = false;
    XboxController controller;
    private boolean clawToggle = false;
    private boolean toggleClawTimer = false;
    private boolean failSafe = false;
    BooleanPublisher clawTelemetryButton;
    DoublePublisher pneumaticPressureTelemetry;
    Timer timer = new Timer();
    
    public Claw(DoubleSolenoid solenoid, XboxController controller) {
        this.solenoid = solenoid;
        this.controller = controller;
    }


    // This method sets the value of the solenoid to close it.
    public void closeClaw() {
        solenoid.set(Value.kReverse);
    }

    // This method sets the value of the solenoid to open it.
    public void openClaw() {
        solenoid.set(Value.kForward);
    }
    // Used to tell the SmartDashboard (Shuffleboard) if the claw is open or closed.
    public boolean isOpen() {
        return solenoid.get() == Value.kForward;
    }

    // This method starts the timer for failsafe and it rests failsafe after 2 seconds 
    // so an accidental press does not mess up the timing of the failsafe.
    public void failsafeTimer(){
        double clawTimer = timer.get();
        if(failSafe == false && toggleClawTimer == true){
            if(clawTimer >= 2){
                failSafe = true;
                toggleClawTimer = false;
                timer.stop();
                timer.reset();
            }else{
                timer.stop();
                timer.reset();
                failSafe = false;
                toggleClawTimer = false;
            }
       } 
        
       }

    // This method is a toggle to open and close claw. 
    // It has a failsafe that activates when claw is trying to be opened.
    // 'A' has to pressed once to close the claw but twice to open it.
    public void clawToggled() {
        boolean aButton = controller.getAButtonPressed();
        if (aButton == true) {
            if (!clawToggle) {
                closeClaw();
                failSafe = true;
                SmartDashboard.putBoolean("Claw open", isOpen());
                clawToggle = true;
            } else if (failSafe == true) {
                timer.start();
                toggleClawTimer = true;
                failSafe = false;
                failsafeTimer();
            } else if (clawToggle == true && failSafe == false && toggleClawTimer == false){
                openClaw();
                clawToggle = false;
                SmartDashboard.putBoolean("Claw open", isOpen());
                failSafe = false;
            }
        }

    }
    public void clawTelemetryTable() {
    
        // Creates the Network table for claw          
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Claw");
    
        pneumaticPressureTelemetry = table.getDoubleTopic("Pneumatic Pressure (psi): ").publish();
        clawTelemetryButton = table.getBooleanTopic("Claw open and close button").publish();
        
        }
        // Starts collecting telemetry data for claw
        public void collectClawTelemetry() {
            boolean aButton = controller.getAButtonPressed();
            
            boolean solenoidValue = solenoid.get() != null;
            clawTelemetryButton.set(solenoidValue);
        }

}


