package org.firstinspires.ftc.teamcode.simpleBotCode.autos;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.simpleBotCode.HardwareSimpleBot;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.ENCODER_DRIVE_ONE_TILE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_POWERSHOT_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SHOOTER_DEFAULT_ROTATION;

//Import Constants:
//WARNING!!!!!!!!!!: Before initializing any program, make sure the wobble goal lifter is not facing downwards, if it is the servo will try to do a 360 to get to the right position and it can break itself. There is not position it should be at but make sure its just not facing downwards.
@Autonomous(name = "Remote Auto", group = "!Primary")
public class remoteAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes. (already done)
     */

    private static final String VUFORIA_KEY =
            "AVEjJ2v/////AAABmdKQOJQzM0o6okKDJrLI2rxkN0GIoH6MQXf5ccupZBuAc9sWIow1gQZPILscyRQsZK9U3XSsREHPJ5AwLABu7qK6kv3ttm4u0xPZNFL9Z6xQeE4J2qNBCthVP/GxLJhgogNv8pJS9TP7IRQj+95TzG1ifxKd2CSIR6RwMr7rNdAZ8wZ1q0HDjG62OXXu5zyWLgbxZ0bEwR0tVyRXEEem6w/V6s3H1TOF8w2s0vCyGzFpJBk1Fuh5l5Rorrv3TCY1Y+E5QIt8PokZJH2NxpBzrSgRF9JldVTwRpNz43UY9HppZF2/PuHAvdF1x8hjgSWL4OgVbA7E/hrFtXSg532zmFaEIzblCHrIwoyOLhLTjvEg";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    private final HardwareSimpleBot rb = new HardwareSimpleBot();
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    //Setup variables:
    private final boolean flywheelOn = false;
    private final boolean shooterOut = false;
    private final boolean intakeOn = false;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    public static ColorSensor groundColorSensor;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        rb.init(hardwareMap, this); //runs init stuff in HardwareSimpleBot.java
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /*
          Activate TensorFlow Object Detection before we wait for the start command.
          Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(3.25, 1.78);
        }

        telemetry.addData("Status", "Initializing Ground Color Sensor...");
        telemetry.update();
        // get a reference to the color sensor.
        groundColorSensor = hardwareMap.get(ColorSensor.class, "groundcolorsensor");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float[] hsvValues = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float[] values = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        telemetry.addData("Status", "Ground Color Sensor Initialized");
        telemetry.update();

        telemetry.addData("Mode", "calibrating IMU...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !rb.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status: ", rb.imu.getCalibrationStatus().toString());

        //composeTelemetry();

        rb.moveShooter(false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart(); //Everything up to here is initialization
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        //Create global variables here... //TODO: Someone who is better at coding tell me if this is actually where these should go...
        int numberOfRingsDetected = 0; //Stores number of rings detected by webcam at start of auto
        int angleFacingForward; //Stores heading at beginning of match so we can reference it later
        angles = rb.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleFacingForward = (int) angles.firstAngle;
        telemetry.addData("Angle Captured=", angleFacingForward);
        telemetry.update();

        Thread.sleep(2000); //Wait 1000ms for camera to detect ring after pressing Start (2000 for testing bc idk) TODO: LOWER THIS IF WE NEED MORE TIME FOR AUTO
        telemetry.addData(">", "One second has passsed... Counting Rings...");
        //Get Number of Rings from Camera (which is already on)

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }

                telemetry.update();

                if (updatedRecognitions.isEmpty()) {
                    numberOfRingsDetected = 0;
                } else if (updatedRecognitions.get(0).getLabel().equals("Single")) {
                    numberOfRingsDetected = 1;
                } else if (updatedRecognitions.get(0).getLabel().equals("Quad")) {
                    numberOfRingsDetected = 4;
                } else {
                    telemetry.addData("ERROR:", "Couldn't find any rings :( defaulting to 0?");
                    numberOfRingsDetected = 0;
                }

            } else {
                numberOfRingsDetected = 0;
                telemetry.addData("ERROR:", "Couldn't get new data because updatedRecognitions = null :(");
                telemetry.update();
            }
        }

        if (tfod != null) { //Shutdown to free up system resources
            telemetry.addData(">", "tfod Shutting Down...");
            telemetry.update();

            tfod.shutdown();

            telemetry.addData(">", "tfod Shutdown");
            telemetry.update();
        }

        //Driving Starts here:
        if (numberOfRingsDetected == 0) {
            //0 Ring Code, Go to A (closest)
            telemetry.addData(">", "Starting A Code...");
            telemetry.update();

            rb.driveForwardByEncoderAndIMU(3360, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone

            Thread.sleep(300);
            rb.setLifterMotor(false, 1);
            Thread.sleep(650);

            rb.driveForwardByEncoderAndIMU(-336, rb.FL, .5, .06, DEFAULT_ACCELERATION_INCREMENT); //Reverse to get wobble goal out of lifter and to shooting spot on line
            rb.setLifterMotor(true, -1);
            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED);

            rb.strafeRightByEncoderAndIMU(1300, rb.FL, .8, .06);
            rb.driveStop();

            Thread.sleep(80);
            rb.driveForwardByEncoderAndIMU(648, rb.FL, .8, .06, DEFAULT_ACCELERATION_INCREMENT);
            Thread.sleep(100);

            //rb.rotate(-0.5, .3);
            //Thread.sleep(350);

            rb.moveShooter(true); //Shot 1
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(3, .3);
            Thread.sleep(350);

            Thread.sleep(800);
            rb.moveShooter(true); //Shot 2
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(3, .3);
            Thread.sleep(350);

            Thread.sleep(800);
            rb.moveShooter(true); //Shot 3
            Thread.sleep(200);
            rb.moveShooter(false);
            Thread.sleep(200);
            rb.moveShooter(true); //Shot 4 just to make sure
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(-6, .5);
            rb.driveForwardByEncoderAndIMU(691, rb.FL, .8, .06, DEFAULT_ACCELERATION_INCREMENT); //Drive up to park on white line


        } else if (numberOfRingsDetected == 1) {
            //1 Ring Code, Go to B (middle)
            telemetry.addData(">", "Starting B Code...");
            telemetry.update();
            rb.driveForwardByEncoderAndIMU(4352, rb.FL, .75, .06, DEFAULT_ACCELERATION_INCREMENT); //2000 before?

            //rb.autoDriveSouthWestWithEncoderAndIMU(2104, rb.FL, .8, .06);

            //Old Slower Strafe Code:
//            Thread.sleep(750);
            rb.strafeRightByEncoderAndIMU((int) (1 * ENCODER_DRIVE_ONE_TILE), rb.FL, .4, .05);

            //Thread.sleep(500);
            //rb.driveForwardByEncoderAndIMU(864, rb.FL, .4, .06, DEFAULT_ACCELERATION_INCREMENT);

            Thread.sleep(1000);
            rb.setLifterMotor(false, 1);
            Thread.sleep(650);

            rb.driveForwardByEncoderAndIMU(-192, rb.FL, .5, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED); //replace this with real function lol
            rb.setLifterMotor(true, -1);
            rb.driveForwardByEncoderAndIMU(-626, rb.FL, .8, .06, DEFAULT_ACCELERATION_INCREMENT);
            Thread.sleep(150);
            rb.strafeRightByEncoderAndIMU(-173, rb.FL, .3, .05);

            Thread.sleep(100);

            //rb.rotate(-8.3, .2);
            //Thread.sleep(500);

            rb.moveShooter(true); //Shot 1
            Thread.sleep(200);
            rb.moveShooter(false);
            rb.rotate(3, .3);
            Thread.sleep(800);
            rb.moveShooter(true); //Shot 2
            Thread.sleep(200);
            rb.moveShooter(false);
            rb.rotate(3, .3);
            Thread.sleep(800);
            rb.moveShooter(true); //Shot 3
            Thread.sleep(200);
            rb.moveShooter(false);
            Thread.sleep(200);
            rb.moveShooter(true); //Shot 4 just to make sure
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(-7, .2);
            rb.runIntake(true, false);
            rb.driveForwardByEncoderAndIMU(-1808, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT); //Drive up to park on white line
            Thread.sleep(200);
            rb.flywheel.setPower(FLYWHEEL_SPEED);
            rb.driveForwardByEncoderAndIMU(1808, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.rotate(SHOOTER_DEFAULT_ROTATION, .2);
            rb.moveShooter(true); //top goal shot
            rb.rotate((-SHOOTER_DEFAULT_ROTATION) + 1, .2);
            rb.driveForwardByEncoderAndIMU(808, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT);

        } else if (numberOfRingsDetected == 4) {
            //4 Ring Code, Go to C (furthest)
            telemetry.addData(">", "Starting C Code...");
            telemetry.update();

            rb.driveForwardByEncoderAndIMU((int) (4.4 * ENCODER_DRIVE_ONE_TILE), rb.FL, 1, .08, DEFAULT_ACCELERATION_INCREMENT); //Drive to A Zone

            Thread.sleep(1000);
            rb.setLifterMotor(false, 1);
            Thread.sleep(650);
            Thread.sleep(750); //Wait a little bit for Servo to drop

            rb.driveForwardByEncoderAndIMU(-336, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT); //Reverse to get wobble goal out of lifter and to shooting spot on line
            rb.setLifterMotor(true, -1);
            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED);
            rb.autoDriveSouthWestWithEncoderAndIMU(2848, rb.FL, .9, .05);
            telemetry.addData(">", "Done with south west");
            telemetry.update();
            rb.driveForwardByEncoderAndIMU(-1095, rb.FL, 1, .05, DEFAULT_ACCELERATION_INCREMENT);
            telemetry.addData(">", "Done with approach behind line");
            telemetry.update();

            Thread.sleep(200);


            rb.moveShooter(true); //Shot 1
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(3, .3);
            Thread.sleep(350);

            Thread.sleep(800);
            rb.moveShooter(true); //Shot 2
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(2.5, .3);
            Thread.sleep(350);

            Thread.sleep(800);
            rb.moveShooter(true); //Shot 3
            Thread.sleep(200);
            rb.moveShooter(false);
            Thread.sleep(200);
            rb.moveShooter(true); //Shot 4 just to make sure
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(-6, .5); //rotate back to 0
            rb.runIntake(true, false);

            rb.driveForwardByEncoderAndIMU(-800, rb.FL, 1, .06, 0.1);
            rb.driveForwardByEncoderAndIMU(-800, rb.FL, 1, .08, 0.1);

            Thread.sleep(1000);

            rb.driveForwardByEncoderAndIMU(1600, rb.FL, .5, .08, DEFAULT_ACCELERATION_INCREMENT); //drive back up to line
            rb.rotate(SHOOTER_DEFAULT_ROTATION, .5);

            //Start Rapid Firing into high goal:
            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED);
            Thread.sleep(200);


            rb.moveShooter(true); //Shot 1
            Thread.sleep(200);
            rb.moveShooter(false);

            Thread.sleep(800);
            rb.moveShooter(true); //Shot 2
            Thread.sleep(200);
            rb.moveShooter(false);

            Thread.sleep(800);
            rb.moveShooter(true); //Shot 3
            Thread.sleep(200);
            rb.moveShooter(false);
            Thread.sleep(200);
            rb.moveShooter(true); //Shot 4 just to make sure
            Thread.sleep(200);
            rb.moveShooter(false);

            /*
            //Shot 1:
            rb.moveShooter(true); //Shoot
            Thread.sleep(8); //8ms = time for ring to leave shooter
            rb.flywheel.setPower(.98); //Increase speed as soon as ring is not in contact with flywheel to increase time back to normal speed
            Thread.sleep(117);
            rb.flywheel.setPower(FLYWHEEL_SPEED); //Return to normal speed
            Thread.sleep(67); //Wait a tiny bit before going back (originally 200 but this value is subtracted from prior Thread.sleep statements)
            rb.moveShooter(false);
            Thread.sleep(150); //Wait for flywheel to get back to 100 percent speed
            //Shot 2:
            rb.moveShooter(true); //Shoot
            Thread.sleep(8); //8ms = time for ring to leave shooter
            rb.flywheel.setPower(.98); //Increase speed as soon as ring is not in contact with flywheel to increase time back to normal speed
            Thread.sleep(117);
            rb.flywheel.setPower(FLYWHEEL_SPEED); //Return to normal speed
            Thread.sleep(67); //Wait a tiny bit before going back (originally 200 but this value is subtracted from prior Thread.sleep statements)
            rb.moveShooter(false);
            Thread.sleep(150); //Wait for flywheel to get back to 100 percent speed
            //Shot 3:
            rb.moveShooter(true); //Shoot
            Thread.sleep(8); //8ms = time for ring to leave shooter
            rb.flywheel.setPower(.98); //Increase speed as soon as ring is not in contact with flywheel to increase time back to normal speed
            Thread.sleep(117);
            rb.flywheel.setPower(FLYWHEEL_SPEED); //Return to normal speed
            Thread.sleep(67); //Wait a tiny bit before going back (originally 200 but this value is subtracted from prior Thread.sleep statements)
            rb.moveShooter(false);
            Thread.sleep(150); //Wait for flywheel to get back to 100 percent speed
            //Shot 4: (Just in case a ring doesn't fire)
            rb.moveShooter(true); //Shoot
            Thread.sleep(8); //8ms = time for ring to leave shooter
            rb.flywheel.setPower(.98); //Increase speed as soon as ring is not in contact with flywheel to increase time back to normal speed
            Thread.sleep(117);
            rb.flywheel.setPower(FLYWHEEL_SPEED); //Return to normal speed
            Thread.sleep(67); //Wait a tiny bit before going back (originally 200 but this value is subtracted from prior Thread.sleep statements)
            rb.moveShooter(false);
            Thread.sleep(150); //Wait for flywheel to get back to 100 percent speed
            */

            rb.driveForwardByEncoderAndIMU(1000, rb.FL, 1, 0.06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive up to park line


        } else {
            telemetry.addData("ERROR", "if you are seeing this error, you shouldn't be...");
            telemetry.update();
        }


        rb.driveStop();


    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /**
     * Logs IMU data to telemetry, TODO: Throttle or disable for competition
     */
    void composeTelemetry() throws InterruptedException {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = rb.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//               gravity = rb.imu.getGravity(); dont need gravity?
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return rb.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return rb.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        Thread.sleep(3000); //throttle display
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}