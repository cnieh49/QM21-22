package org.firstinspires.ftc.teamcode.simpleBotCode;


public final class simpleBotConstants {

    //Speeds:
    public static final double FLYWHEEL_SPEED = .9;
    //.95 = 28.45 ft @ 31 degrees
    //.53 = 9.4 ft @ 31 degrees
    //.65 = 13.12ft @ 31 degrees
    //.73 = 16.60 ft @ 31 degrees
    //.71 = 16.21 @ 31 degrees

    public static final double INTAKE_SPEED = 1;

    //Sticks:
    public static final float DRIVE_STICK_THRESHOLD = .05f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;
    public static final float TRIGGER_THRESHOLD = .5f;

    //Servo:
    public static final double SHOOTER_OUT = .69;
    public static final double SHOOTER_IN = .55;
    public static final double LIFTER_MAXIMUM = .58;
    public static final double LIFTER_UP = .60;
    public static final double LIFTER_DOWN = .88;
    public static final double LIFTER_MINIMUM = .99;

    //Color Sensor:
    //Setting the correct WHITE_ALPHA_THRESHOLD value is key to stopping correctly. This should be set half way between the light and dark values.
    public static final double WHITE_ALPHA_THRESHOLD = 170; //TODO: Update Color Values on real field

}
