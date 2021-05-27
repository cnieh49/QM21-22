package org.firstinspires.ftc.teamcode.simpleBotCode;


public final class simpleBotConstants {
    public static double GEAR_RATIO_MULTIPLIER = 1-(0.388268398268398);
    //Speeds:
    public static double FLYWHEEL_SPEED = .75;  //Volkswagen Mode = .7
    public static double FLYWHEEL_POWERSHOT_SPEED = .636;
    public static double FLYWHEEL_LONGSHOT_SPEED = .688;
    public static double FLYWHEEL_LONGPOWERSHOT_SPEED = .635;
    //.95 = 28.45 ft @ 31 degrees
    //.53 = 9.4 ft @ 31 degrees
    //.65 = 13.12ft @ 31 degrees
    //.73 = 16.60 ft @ 31 degrees
    //.71 = 16.21 @ 31 degrees
    public static final double INTAKE_SPEED = 1;
    public static final double DEFAULT_ACCELERATION_INCREMENT = 0.02;

    //ROTATIONS
    public static final double SHOOTER_DEFAULT_ROTATION = -3.6; //The angle that the robot needs to be at for the shooter to shoots straight (recorded at FLYWHEEL_SPEED = .77)

    //Controls:
    public static final float DRIVE_STICK_THRESHOLD = .0f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;
    public static final float TRIGGER_THRESHOLD = .4f;
    public static final int BUTTON_DELAY = 250; //Delay after any toggle button is pressed before button is checked again. idk

    //Motor:
    public static final int LIFTER_MOTOR_UP = -50;
    public static final int LIFTER_MOTOR_MID = (int) (100 * 4.2);
    public static final int LIFTER_MOTOR_DOWN = 1470;

    //Wheel Diameter = 100m
    //100m * pi = 314.16mm = circumference = 1 wheel rotation
    //767.2 Ticks per 314.16mm distance traveled
    //2 ft (1 tile distance) = 609.6mm
    //609.6/314.16 = 1.940412528647823
    //1.940412528647823 * 767.2 ticks = 1488.684491978609626 ticks per tile?

    public static final double TICKS_PER_ROTATION = 767.2;
    public static final double ENCODER_DRIVE_ONE_TILE = 1.940412528647823 * TICKS_PER_ROTATION; //Evaluates to 1488.684491978609626 right now


    //Servo:
    public static final double SHOOTER_OUT = .27;
    public static final double SHOOTER_IN = .09; //Position when servo is just inside of the hopper

    public static final double WOBBLE_ARMED = 1;
    public static final double WOBBLE_OPEN = .51;
    public static final double WOBBLE_CLOSED = .30;


//    public static final double LIFTER_MAXIMUM = .58;
//    public static final double LIFTER_UP = .61;
//    public static final double LIFTER_MID = .79;
//    public static final double LIFTER_DOWN = 1;
//    public static final double LIFTER_MINIMUM = .99;

    //Color Sensor:
    //Setting the correct WHITE_ALPHA_THRESHOLD value is key to stopping correctly. This should be set half way between the light and dark values.
    public static final double WHITE_ALPHA_THRESHOLD = 170; //

    //Distance Sensor:
    //These are in inches:
    public static final double SIDE_TO_CENTER_DISTANCE = 7.44; //inches
    public static final double FRONT_TO_CENTER_DISTANCE = 10.17;
    public static final double CENTER_TO_TOWER_DISTANCE = FRONT_TO_CENTER_DISTANCE + 61;
    public static final double SIDE_WALL_TO_TOWER_DISTANCE = 34.5;
    //in mm

    public static final double WOBBLE_MININUM_DISTANCE = 13; // in mm
    public static final double WOBBLE_MOVE_DOWN_DELAY = 1500; //in ms

    public static final int HOPPER_ZERO_RINGS = 130;
    public static final int HOPPER_ONE_RING = 121;
    public static final int HOPPER_TWO_RINGS = 100;
    public static final int HOPPER_THREE_RINGS = 86;
    public static final int[] ringDistanceArray = {HOPPER_ZERO_RINGS, HOPPER_ONE_RING, HOPPER_TWO_RINGS, HOPPER_THREE_RINGS};

    public static final double WOBBLE_2M_THRESHOLD = 325; //in mm


}
