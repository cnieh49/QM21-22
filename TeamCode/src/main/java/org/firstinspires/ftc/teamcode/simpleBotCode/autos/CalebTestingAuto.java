//DO NOT USE EVER

package org.firstinspires.ftc.teamcode.simpleBotCode.autos;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.simpleBotCode.HardwareSimpleBot;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.ENCODER_DRIVE_ONE_TILE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_LONGPOWERSHOT_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_LONGSHOT_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_POWERSHOT_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SHOOTER_DEFAULT_ROTATION;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_2M_THRESHOLD;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_ARMED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_CLOSED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_MININUM_DISTANCE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_OPEN;

//Import Constants:
//WARNING!!!!!!!!!!: Before initializing any program, make sure the wobble goal lifter is not facing downwards, if it is the servo will try to do a 360 to get to the right position and it can break itself. There is not position it should be at but make sure its just not facing downwards.
@Autonomous(name = "CalebTestingAuto", group = "!Primary")
public class CalebTestingAuto extends LinearOpMode {

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


    // State used for updating telemetry
    Orientation angles;
//    Acceleration gravity;


    //public static ColorSensor groundColorSensor;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry.addData(">", "REMEMBER TO CHECK WOBBLE MOTOR AND SERVO POSITIONS!");
        telemetry.addData("Status", "Initializing...");
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
            tfod.setZoom(3, 1.78);
        }


        telemetry.addData("Mode", "Calibrating IMU...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !rb.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status: ", rb.imu.getCalibrationStatus().toString());

        //composeTelemetry();

        rb.moveShooter(false);

        telemetry.addData("Mode", "Resetting Encoders...");
        telemetry.update();
        //Reset Encoders
        rb.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(150);
        rb.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Mode", "Done Resetting Encoders...");
        telemetry.update();


        rb.wobbleServo.setPosition(WOBBLE_ARMED);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart(); //Everything up to here is initialization
        runtime.reset();
        rb.wobbleServo.setPosition(WOBBLE_CLOSED);
        // run until the end of the match (driver presses STOP)
        //Create global variables here
        int numberOfRingsDetected = 0; //Stores number of rings detected by webcam at start of auto
        int angleFacingForward; //Stores heading at beginning of match so we can reference it later
        angles = rb.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleFacingForward = (int) angles.firstAngle;
        telemetry.addData("Angle Captured=", angleFacingForward);
        telemetry.update();

        Thread.sleep(150); //Wait 1000ms for camera to detect ring after pressing Start (2000 for testing bc idk) TODO: LOWER THIS IF WE NEED MORE TIME FOR AUTO
        telemetry.addData(">", "One second has passsed... Counting Rings...");
        //Get Number of Rings from Camera (which is already on)

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.

                /*if (updatedRecognitions.isEmpty()) {
                    numberOfRingsDetected = 0;
                }
                else {*/
                //essentially it's either going to recognize something or its going to skip over
                //this part and the number of rings will stay at 0
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("label length", i), recognition.getLabel().length());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if(recognition.getLabel().length() == 4){ //when there are 4 rings
                        numberOfRingsDetected = 4;
                    }
                    else if(recognition.getLabel().length() == 6){ //when there is 1 ring - will print out "Single"
                        numberOfRingsDetected = 1;
                    }
                    else {
                        telemetry.addData("ERROR:", "Couldn't find any rings or something else went wrong :( defaulting to 1");
                        numberOfRingsDetected = 1;
                    }
                }
                telemetry.addData("HEY!", "Real question tho... does it work??");
                telemetry.update();
                //}


                /*
                if (updatedRecognitions.isEmpty()) {
                    numberOfRingsDetected = 0;
                } else if (updatedRecognitions.size() == 4) {
                    numberOfRingsDetected = 4;
                } else if (updatedRecognitions.size() == 1) {
                    numberOfRingsDetected = 1;
                } else {
                    telemetry.addData("ERROR:", "Couldn't find any rings :( defaulting to 1");
                    numberOfRingsDetected = 1;
                }
                telemetry.addData("Number of Rings!", updatedRecognitions); //updatedRecognitions.size()
                telemetry.update();
                */

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
            Thread.sleep(250);

            telemetry.addData(">", "tfod Shutdown");
            telemetry.update();
        }

        //Driving Starts here:
        if (numberOfRingsDetected == 0) {
            //0 Ring Code, Go to A (closest)
            telemetry.addData(">", "Starting A Code...");
            telemetry.update();

            rb.driveForwardByEncoderAndIMU(3600, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT * 2); //Drive to A Zone

            rb.setLifterMotor(false, 0.75);
            Thread.sleep(400);
            rb.wobbleServo.setPosition(WOBBLE_OPEN);

            rb.driveForwardByEncoderAndIMU(-436, rb.FL, .7, .06, DEFAULT_ACCELERATION_INCREMENT); //Reverse to get wobble goal out of lifter and to shooting spot on line
            rb.setLifterMotor(true, -1);
            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED);

            rb.strafeRightByEncoderAndIMU(1300, rb.FL, .8, .06);
            rb.driveStop();

            Thread.sleep(80);
            rb.driveForwardByEncoderAndIMU(728, rb.FL, .6, .06, DEFAULT_ACCELERATION_INCREMENT);
            Thread.sleep(100);

            //rb.rotate(1, .3);
            //Thread.sleep(350);

            rb.moveShooter(true); //Shot 1
            Thread.sleep(200);
            rb.moveShooter(false);

            rb.rotate(3, .3);
            Thread.sleep(350);

            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED - 0.025);
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
            if (rb.getNumberOfRingsInHopper() != 0) {
                rb.moveShooter(true); //Shot 4 just to make sure
                Thread.sleep(200);
                rb.moveShooter(false);
            }

            rb.flywheel.setPower(0);
            //rb.driveForwardByEncoderAndIMU(-1008, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT); //Drive up to park on white line
            //Thread.sleep(300);
            rb.rotate(173, .5);
            rb.setLifterMotor(false, -1);
            rb.driveForwardByEncoderAndIMU(1488, rb.FL, 1, .04, DEFAULT_ACCELERATION_INCREMENT);
            while (rb.wobble2mRangeSensor.getDistance(DistanceUnit.MM) > WOBBLE_2M_THRESHOLD && opModeIsActive()) { //TODO: Add a failsafe to this while loop based off of time
                rb.strafe(-.3, -.3);
            }
            telemetry.addData("Update:", "WOBBLE DETCTED");
            telemetry.update();
            rb.strafeRightByEncoderAndIMU(-60, rb.FL, .3, .05); //TODO: Uncomment this and add a tiny value to account for additional distance needed to travel after sensing wobble goal with 2m sensor
            rb.driveForwardByEncoderAndIMU(650, rb.FL, 0.4, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.wobbleServo.setPosition(WOBBLE_CLOSED);
            Thread.sleep(200);
            rb.setLifterMotor(true, -1);
            rb.rotate(-180, .4);
            //rb.flywheel.setPower(FLYWHEEL_SPEED);
            rb.strafeRightByEncoderAndIMU((int) ((-ENCODER_DRIVE_ONE_TILE)), rb.FL, 1, .05);
            rb.driveForwardByEncoderAndIMU(2008, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT);
            //rb.driveForwardByEncoderAndIMU(1408, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT);
            //rb.rotate(SHOOTER_DEFAULT_ROTATION, .2);
            //rb.moveShooter(true); //top goal shot
            //Thread.sleep(200);
            //rb.moveShooter(true); //second just in case
            //Thread.sleep(200);
            //rb.moveShooter(false);
            //rb.rotate((-SHOOTER_DEFAULT_ROTATION) + 1, .2);
            //rb.driveForwardByEncoderAndIMU(1008, rb.FL, 0.5, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.setLifterMotor(false, -1);
            Thread.sleep(400);
            rb.wobbleServo.setPosition(WOBBLE_OPEN);
            rb.driveForwardByEncoderAndIMU(-408, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.strafeRightByEncoderAndIMU((int) (ENCODER_DRIVE_ONE_TILE / 3), rb.FL, 1, .05);
            rb.driveForwardByEncoderAndIMU(1108, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT);
            //rb.driveForwardByEncoderAndIMU(691, rb.FL, .8, .06, DEFAULT_ACCELERATION_INCREMENT); //Drive up to park on white line


        } else if (numberOfRingsDetected == 1) {
            //1 Ring Code, Go to B (middle)
            telemetry.addData(">", "Starting B Code...");
            telemetry.update();
            rb.driveForwardByEncoderAndIMU(4352 + 600, rb.FL, 1, .06, DEFAULT_ACCELERATION_INCREMENT); //2000 before?

            //rb.autoDriveSouthWestWithEncoderAndIMU(2104, rb.FL, .8, .06);

            //Old Slower Strafe Code:
//            Thread.sleep(750);
            rb.strafeRightByEncoderAndIMU((int) (1 * ENCODER_DRIVE_ONE_TILE), rb.FL, .75, .05); //power was .4 before

            //Thread.sleep(500);
            //rb.driveForwardByEncoderAndIMU(864, rb.FL, .4, .06, DEFAULT_ACCELERATION_INCREMENT);

            Thread.sleep(200);
            rb.setLifterMotor(false, 1);
            Thread.sleep(340);
            rb.wobbleServo.setPosition(WOBBLE_OPEN);

            rb.driveForwardByEncoderAndIMU(-192, rb.FL, .6, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED); //replace this with real function lol
            //rb.setLifterMotor(true, -1);
            rb.driveForwardByEncoderAndIMU(-626 - 300, rb.FL, .8, .06, DEFAULT_ACCELERATION_INCREMENT);
            Thread.sleep(150);
            rb.strafeRightByEncoderAndIMU(-173, rb.FL, .3, .05);
            rb.setLifterMotor(true, 1);

            Thread.sleep(100);

            //rb.rotate(-8.3, .2);
            //Thread.sleep(500);

            rb.moveShooter(true); //Shot 1
            Thread.sleep(200);
            rb.moveShooter(false);
            rb.rotate(3, .3);
            rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED - 0.025);
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

            if (rb.getNumberOfRingsInHopper() != 0) {
                rb.moveShooter(true); //Shot 4 just to make sure
                Thread.sleep(200);
                rb.moveShooter(false);
            }

            rb.setLifterMotor(false, 1);

            //For top goal shot
            rb.rotate(-7, .8);

            rb.runIntake(true, false);
            rb.strafeRightByEncoderAndIMU(-100, rb.FL, .7, .04);
            rb.driveForwardByEncoderAndIMU(-1358, rb.FL, 0.75, .06, DEFAULT_ACCELERATION_INCREMENT); //Drive up to park on white line
            Thread.sleep(300);
            rb.strafeRightByEncoderAndIMU(100, rb.FL, .7, .04);
            rb.rotate(180, .6);
            rb.driveForwardByEncoderAndIMU(600, rb.FL, 0.5, .06, DEFAULT_ACCELERATION_INCREMENT);

            while (rb.wobble2mRangeSensor.getDistance(DistanceUnit.MM) > WOBBLE_2M_THRESHOLD && opModeIsActive()) { //TODO: Add a failsafe to this while loop based off of time
                rb.strafe(-.3, -.3);
            }
            telemetry.addData("Update:", "WOBBLE DETCTED");
            telemetry.update();
            rb.strafeRightByEncoderAndIMU(-100, rb.FL, .3, .05); //TODO: Uncomment this and add a tiny value to account for additional distance needed to travel after sensing wobble goal with 2m sensor
            rb.driveForwardByEncoderAndIMU(450, rb.FL, 0.4, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.wobbleServo.setPosition(WOBBLE_CLOSED);
            Thread.sleep(200);
            rb.setLifterMotor(true, -1);
            /*
            rb.strafeRightByEncoderAndIMU(-120, rb.FL, .4, .05);
            rb.driveForwardByEncoderAndIMU(1058, rb.FL, 0.5, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.wobbleServo.setPosition(WOBBLE_CLOSED);
            rb.driveStop();
            Thread.sleep(250);
            rb.setLifterMotor(true, -.75);
            rb.rotate(-180, .5);

             */
            rb.flywheel.setPower(FLYWHEEL_SPEED);
            rb.runIntake(false, false);
            //rb.moveShooter(false); //top goal shot
            rb.strafeRightByEncoderAndIMU((int) (-ENCODER_DRIVE_ONE_TILE / 8), rb.FL, .4, .05);
            rb.driveForwardByEncoderAndIMU(808, rb.FL, 0.7, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.driveForwardByEncoderAndIMU(1380, rb.FL, 0.7, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.rotate(SHOOTER_DEFAULT_ROTATION+1.5, .6);
            Thread.sleep(150);
            rb.moveShooter(true); //top goal shot
            Thread.sleep(200);
            rb.moveShooter(false);
            if (rb.getNumberOfRingsInHopper() != 0) {
                rb.moveShooter(true); //second just in case
                Thread.sleep(200);
                rb.moveShooter(false); //second just in case
            }

            rb.rotate((-SHOOTER_DEFAULT_ROTATION) - 1.5, .5);
            rb.driveForwardByEncoderAndIMU(1008, rb.FL, 0.5, .06, DEFAULT_ACCELERATION_INCREMENT);
            rb.wobbleServo.setPosition(WOBBLE_OPEN);
            rb.setLifterMotor(false, -1);
            rb.driveForwardByEncoderAndIMU(-408, rb.FL, 0.5, .06, DEFAULT_ACCELERATION_INCREMENT);

        }
        else if (numberOfRingsDetected == 4) {
            //4 Ring Code, Go to C (furthest)
            telemetry.addData(">", "Starting C Code...");
            telemetry.update();

            rb.flywheel.setPower(FLYWHEEL_LONGPOWERSHOT_SPEED);
            Thread.sleep(800);
            int numshots = 0;
            while(rb.getNumberOfRingsInHopper() >= 1 && numshots < 4){
                int x = rb.getNumberOfRingsInHopper();
                rb.moveShooter(true);
                Thread.sleep(200);
                rb.moveShooter(false);
                Thread.sleep(400);
                if(rb.getNumberOfRingsInHopper() > x){
                    rb.moveShooter(true);
                    Thread.sleep(200);
                    rb.moveShooter(false);
                }
                if(rb.getNumberOfRingsInHopper() >= 1) {
                    rb.strafeRightByEncoderAndIMU((int) (-ENCODER_DRIVE_ONE_TILE * 0.25), rb.FL, 0.8, .05);
                }
                numshots++;
            }
            rb.driveForwardByEncoderAndIMU((int) (4.4 * ENCODER_DRIVE_ONE_TILE) + 450, rb.FL, 1, .08, DEFAULT_ACCELERATION_INCREMENT); //Drive to A Zone

            Thread.sleep(200);
            rb.setLifterMotor(false, 0.7);
            Thread.sleep(500);
            rb.wobbleServo.setPosition(WOBBLE_OPEN);
            Thread.sleep(200);
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