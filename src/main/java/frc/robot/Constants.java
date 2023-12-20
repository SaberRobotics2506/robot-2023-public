package frc.robot;

public class Constants {
    
    public static final int TOP_RIGHT_ID = 3;
    public static final int TOP_LEFT_ID = 8;
    public static final int BOTTOM_RIGHT_ID = 2;
    public static final int BOTTOM_LEFT_ID = 7;
    //public static final int H_MOTOR_ID = 1;
    //Thresholds should be -3 and 3 respectively
    public static final double lowThreshold = -1.5;
    public static final double highThreshold = 1.5;
    
    
    
    
       
    public static final double turnOffset = 0.3375;
    public static final double hPercentPower = 0.5;
    public static final double outerMotorRamp = .51;
    public static final double hMotorRamp = .8;
    public static final double outerMotorSpeed = 0.61;
    //public static final double hMotorSpeed = 1;

    public static final double yDeadZone = 0.2;
    public static final double xDeadZone = 0.2;
    
    final static double REVERSE_SPEED_PICK = -0.5;
    final static double SPEED_PICK = 0.5;
    final static int LEFT_MOTOR_ID = 4;
    final static int RIGHT_MOTOR_ID = 9;
    final static int INTAKE_DEPLOYED_CHANNEL = 4;
    final static int INTAKE_RETRACTED_CHANNEL = 5;
    final static int CONTROLLER_DRIVER_0 = 0;
    final static int CONTROLLER_OPERATOR_1 = 1;
    //final static int HDRIVE_SPEED_0 = 0;
    
    final static int CLAW_DEPLOYED_CHANNEL = 1;
    final static int CLAW_RETRACTED_CHANNEL = 0;

    public static final double scoreToObjectDistance = 224;
    //public static final double rampToObjectDistance = 85.13;
    //public static final double ticksPerInch = 1060.851;
    
    public static final double rampToObjectDistance = 30;
    public static final double autoDistanceToMidField = 170;
    public static final double ticksPerInch = 1060.851;
    //public static final double hTicksPerInch = 1083.7;
    public static final double autoDriveSpeed = 0.6;
    public static final double autoBalanceDriveSpeed = 0.45;

    public static final double scoreToObjectInTicks = scoreToObjectDistance * ticksPerInch;
    public static final double rampToObjectInTicks = rampToObjectDistance * ticksPerInch;

    public static final int WRIST_DEPLOYED_CHANNEL = 2;
    public static final int WRIST_RETRACTED_CHANNEL = 3;

    public static final double DEAD_ZONE_FOR_RIGHT_STICK = 0.05;

    public static final double spinSpeed = 0.4;
    public static final double spinSlowSpeed = 0.2;
    public static final double spinReverseSpeed = 0.05;
   
    public static final double ampLimit = 40;

    public static final double fastTurnSpeedMult = 1;
    public static final double fastPercentSpeedOuter = .9;
    public static final double slowTurnSpeedMult = .7;
    public static final double slowPercentSpeedOuter = .3;
    public static final double arcadeTurnDampener = 3;
    public static final double slowModearcadeTurnDampener = .8;
    public static final double turningSpeedSuper = 1.2;

    //Limelight mount constants
    final public static double limelightMountAngleDegrees = 1.899; //from vertical (a1)
    final public static double heightLimelightInches = 30; //from ground (h1)
    final public static double aprilTagAngleFromHomeDegrees = 0;
    final public static double aprilTagAngleFromOppositeHomeDegrees = 180 - aprilTagAngleFromHomeDegrees; //it is perpendicular to the surface of the apriltag on the ground plane
    final public static double automaticDistanceFromAprilTagInches = 49;

    //Limelight drive and log ramping constants
    final public static double limelightLogRampingMultiplier = .7;
    final public static double limelightLogRampingBase = 2;
    final public static double limelightLogRampingInversePower = 0.6;
    final public static double limelightLogRampingAddedConstant = -.04;
    final public static double limelightDriveYDampener = .02250;
    final public static double limelightDriveXDampener = .005;
    final public static double limelightDriveTurnDampener = .1;

    //Limelight field element constants
    final public static int[] aprilTagsBlueAlliance = {8, 7, 6, 5};
    final public static int[] aprilTagsRedAlliance = {1, 2, 3, 4};
    final public static int[] aprilTagsAllied = aprilTagsBlueAlliance;
    final public static int[] aprilTagsCompetitor = aprilTagsRedAlliance;

    final public static double heightApriltagGoalInches = 27.375; //height of apriltags from ground (inches)
    final public static double heightApriltagLoadingInches = 23.375; //height of apriltags from ground (inches)
    final public static double lowTapeHeightInches = 24.125; //height of second row cone poles reflective tape (inches)
    final public static double highTapeHeightInches = 45.875; //height of third row cone poles reflective tape (inches)

    //Limelight pipeline constants
    final public static int offPipeline = 0; //turns off LEDs (default)
    final public static int aprilPipeline = 1; //tracks apriltags
    final public static int lowTapePipeline = 2; //tracks reflective tape (close row cone poles)
    final public static int highTapePipeline = 3; //tracks reflective tape (far cone poles)

    //Limelight in-school constants (testing)
    //double heightGoalInches = 12.65; //robot cart
    //double heightGoalInches = 35.0394; //blue wall window
    //double heightGoalInches = 44; //white wall window
    final public static double heightGoalInches = 25.125;

    //TODO: change or remove
    final public static int ULTRASONIC_IN_ID = 5;
    final public static int ULTRASONIC_OUT_ID = 4;
    final public static double ULTRASONIC_DRIVE_OUTPUT = .38;
    final public static double ULTRASONIC_DRIVE_DESIRED_DISTANCE = 31.5;
    final public static double ULTRASONIC_DRIVE_ALLOWED_ERROR_INCHES = 1;
    final public static double ULTRASONIC_MAX_START_DIST = 80;
    final public static double ULTRASONIC_MIN_OUTPUT = .13;
    final public static double ULTRASONIC_MAX_OUTPUT = 0.3;
    final public static double ULTRASONIC_TOTAL_MAX_RANGE = ULTRASONIC_MAX_START_DIST + ULTRASONIC_DRIVE_DESIRED_DISTANCE;

    final public static double ULTRASONIC_KP = 0.0375;
    final public static double ULTRASONIC_KI = 0;
    final public static double ULTRASONIC_KD = 0;
    //Limelight control constants

    
    //Arm and Elevator Constants
    //Defining the port values for arm motors
    final static int ELEVATOR_RIGHT_ID = 5;
    final static int ELEVATOR_LEFT_ID = 6;

    final static int ARM_MOTOR_ID = 18;

    final static int kPIDLoopIdx = 0;

	final static int TOP_LIMIT_CHANNEL = 0;
	final static int BOTTOM_LIMIT_CHANNEL = 0;



    final static int SENSOR_UNITS_PER_100_MS = 25000;

    final static int SENSOR_UNITS_PER__100_MS_PER_SEC = 50000;

    final static int AIR_COMPRESSOR_MIN_PRESSURE = 90;
    final static int AIR_COMPRESSOR_MAX_PRESSURE = 100;

	//POV Angles for controller
	final static int DPAD_UP = 0;
	final static int DPAD_LEFT = 270;
	final static int DPAD_DOWN = 180;
	final static int DPAD_RIGHT = 90;

	// Change these values when we get actual robot
	final static int kTimeoutMs = 30;
    final static int kSlotIdx = 0;
	final static double armRetractOutput = -0.2;
    final static double armExtendOutput = 0.2;
    final static double elevatorRetractOutput = -0.4;
    final static double elevatorExtendOutput = 0.4;
	final static  double ARM_FULL_DOWN = 0; 
    final static double ABOUT_EQUAL_VAL = 3000;
    final static double clawOpenWait = .1; //in seconds and 
    final static double moveBackElevWait = .3; //the delay between this and claw open is this-

    // Automatic Cone presets for arm
    //TODO: must test for actual encoder values, or do math or something
    final static int LIMIT_SWITCH_ARM_ID = 2;
    final static int LIMIT_SWITCH_ELEVATOR_ID = 1;
    final static int breakbeamID = 3;

    final static int STATION_CONE_ARM = 121000;
    final static int STATION_CONE_ELEV = 68500;

    final static int STATION_CUBE_ARM = 120000;
    final static int STATION_CUBE_ELEV = 68500;

    final static int TRANSIT_ARM = 0;
    final static int TRANSIT_ELEVATOR = 0;

	final static int HIGH_CONE_ELEV = 96500;
	final static int HIGH_CONE_ARM = 131000; 

	final static int MID_CONE_ELEV = 61000;
	final static int MID_CONE_ARM = 72000;

    final static int MAX_ARM = 149000;
    final static int MAX_ELEVATOR = 97000;

	final static int HIGH_CUBE_ELEV = 97000;
	final static int HIGH_CUBE_ARM = 121000;

	final static int MID_CUBE_ELEV = 41000;
	final static int MID_CUBE_ARM = 121000;

    final static int HYBRID_ELEV = 0;
    final static int HYBRID_ARM = 100000;

    //arm and elevator pid
    final static int ARM_PID_SLOT = 0;
    final static int ELEVATOR_PID_SLOT = 0;
    final static double[] ARM_PIDF = {0.16, 0, 0.0, 0};
    //kd used to be 0.0843
    final static double[] ELEV_PIDF = {0.06, 0, 0, 0};

    final static double PEAK_OUTPUT_ARM_FORWARD = 1;
    final static double PEAK_OUTPUT_ARM_REVERSE = -1;
    final static double PEAK_OUTPUT_ELEV_FORWARD = 1;
    final static double PEAK_OUTPUT_ELEV_REVERSE = -1;

    final static double ELEVATOR_ACCEPTABLE_ERROR = 1000;
    final static double ARM_ACCEPTABLE_ERROR = 1000;

    final static double ARM_MANUAL_MINIMUM_OUTPUT = .2;
    final static double ELEVATOR_MANUAL_MINIMUM_OUTPUT = .2;
    final static double TRIGGER_LEEWAY = .2;

    final static double MANUAL_ARM_DAMPENER = 0.5;
    final static double MANUAL_ELEVATOR_DAMPENER = 0.4;
	final static double ARM_RESET = 0;

    final static double OPERATOR_DEBOUNCE_TIME = .2;

    final static double RAMP_ARM_ELEVATOR = 0.30001;

    final static double MAX_CRUISE_VELOCITY_ELEV = 15000;
    final static double MAX_CRUISE_ACCEL_ELEV = 200000;
    final static double MAX_CRUISE_VELOCITY_ARM = 20000;
    final static double MAX_CRUISE_ACCEL_ARM = 200000;
}