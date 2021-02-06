package org.firstinspires.ftc.teamcode.simpleBotCode;


public final class simpleBotConstants {

    //Speeds:
    public static double FLYWHEEL_SPEED = .77;  //Volkswagen Mode = .7
    //.95 = 28.45 ft @ 31 degrees
    //.53 = 9.4 ft @ 31 degrees
    //.65 = 13.12ft @ 31 degrees
    //.73 = 16.60 ft @ 31 degrees
    //.71 = 16.21 @ 31 degrees
    public static final double INTAKE_SPEED = 1;
    public static final double DEFAULT_ACCELERATION_INCREMENT = 0.01;

    //ROTATIONS
    public static final double SHOOTER_DEFAULT_ROTATION = -8.6; //The angle that the robot needs to be at for the shooter to shoots straight (recorded at FLYWHEEL_SPEED = .77)

    //Controls:
    public static final float DRIVE_STICK_THRESHOLD = .0f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;
    public static final float TRIGGER_THRESHOLD = .5f;
    public static final int BUTTON_DELAY = 250; //Delay after any toggle button is pressed before button is checked again. idk

    //Motor:
    public static final int LIFTER_MOTOR_UP = 10;
    public static final int LIFTER_MOTOR_DOWN = 370;


    //Servo:
    public static final double SHOOTER_OUT = .41;
    public static final double SHOOTER_IN = .29;
    public static final double LIFTER_MAXIMUM = .58;
    public static final double LIFTER_UP = .61; //.60 before
    public static final double LIFTER_MID = .79;
    public static final double LIFTER_DOWN = 1; //.88 before
    public static final double LIFTER_MINIMUM = .99;

    //Color Sensor:
    //Setting the correct WHITE_ALPHA_THRESHOLD value is key to stopping correctly. This should be set half way between the light and dark values.
    public static final double WHITE_ALPHA_THRESHOLD = 170; //TODO: Update Color Values on real field

}
