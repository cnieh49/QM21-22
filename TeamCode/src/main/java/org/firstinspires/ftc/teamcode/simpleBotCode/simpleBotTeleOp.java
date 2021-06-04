package org.firstinspires.ftc.teamcode.simpleBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BLOCKER_LEFT_DOWN;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BLOCKER_LEFT_START;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BLOCKER_LEFT_UP;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BLOCKER_RIGHT_DOWN;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BLOCKER_RIGHT_START;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BLOCKER_RIGHT_UP;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BUTTON_DELAY;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.CENTER_TO_TOWER_DISTANCE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_POWERSHOT_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_MOTOR_MID;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SHOOTER_DEFAULT_ROTATION;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SIDE_TO_CENTER_DISTANCE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_ARMED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_CLOSED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_MININUM_DISTANCE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_MOVE_DOWN_DELAY;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.WOBBLE_OPEN;


@TeleOp(name = "!QM TeleOP", group = "!Primary")
public class simpleBotTeleOp extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance(); //Comment this out when not using dashboard
    private final HardwareSimpleBot rb = new HardwareSimpleBot();
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    //Setup variables:
    private boolean flywheelOn = false;
    private final boolean shooterOut = false;
    private boolean intakeOn = false;
    private boolean intakeIsEjecting = false;
    private int lifterPosition = 0; // 0= down, 1 = mid, 2 = up Default is true because needs to start up to stay in 18in
    private boolean powershotSpeedActive = false;
    private boolean intakeAutoStopped = false;
    private double timeSinceActivatingWobbleDown = 0;
    private double timeSincePossiblyDetecting3Rings = 0;
    private double slowModeMultiplier = 1;
    private double driveReverseModeMultiplier = 1;


    // State used for updating telemetry
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData(">", "REMEMBER TO CHECK WOBBLE MOTOR AND SERVO POSITIONS!");
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();
        rb.init(hardwareMap, this); //runs init stuff in HardwareSimpleBot.java
        //rb.lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Hardware Map Initialized");
        telemetry.update();

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        //IMPORTANT: The gyro will not calibrate unless the robot is not moving, make sure the robot is still during initialization.
        /*while (!isStopRequested() && !rb.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }*/

        telemetry.addData("imu calib status: ", rb.imu.getCalibrationStatus().toString());

        composeTelemetry();

        telemetry.addData("Status", "Initialized, Ready to Start. Make sure lifter is in back position");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        //rb.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Set Servo Start positions
        rb.wobbleServo.setPosition(WOBBLE_ARMED);
        rb.leftBlocker.setPosition(BLOCKER_LEFT_START);
        rb.rightBlocker.setPosition(BLOCKER_RIGHT_START);

        waitForStart(); //Everything up to here is initialization
        runtime.reset();
        rb.wobbleServo.setPosition(WOBBLE_OPEN);
        telemetry.setAutoClear(true);

        rb.runIntake(true, false); //Start with intake running TODO: Turn this on for real comp
        // run this until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive(); //Drive robot with sticks
            shooter(); // Triggers servo that pushes rings into flywheel
            flywheel(); // Turns flywheel on and off
            intake();//Turns intake on and off
            intakeEject(); //Runs intake in reverse for emergencies

            lifter();//Moves lifter up and down
            //lifterAutoClose();
            powershotSpeed();
            //captureAngle(); //TESTING ONLY: Captures angle
            rotateToAngle(); //TESTING ONLY (for now): Rotates
//            rapidRotateLeft(); //TESTING ONLY: Rotates to the left at max speed
//            rapidRotateRight(); //TESTING ONLY: Rotates to the right at max speed
            //volkswagenMode();
            //alignToGoal();
            slowMode();
            driveReverseMode();
            powershotEndgame();
            AligntoTower();
            ringBlocker();

            //telemetry.addData("Side Distance in Inches:", String.valueOf(rb.sensorRangeSide.getDistance(DistanceUnit.INCH)));
            telemetry.addData("RINGS IN HOPPER:", rb.getNumberOfRingsInHopper());
            telemetry.update(); //for imu display


            /* CONTROLS (for xbox 360 controller)
             * Driver: (Start + A)
             * Left Stick - Movement
             * Right Stick - Rotation
             * Left Bumper - Intake
             * Right Bumper - Flywheel on/off
             * Right Trigger - Shoot servo (only works if flywheel is on)
             * Left Trigger - TESTING ONLY: Aligns robot to goal
             * X - Wobble Goal Lifter
             * Y - Endgame powershot automated function (doesn't really work right now)
             * Right Stick Button - Rotate -8.6 degrees to shoot straight
             * Left Stick Button - Intake Eject for Emergencies
             * D-Pad Down - Slows down flywheel for powershots
             * D-Pad Left - Rapid Rotate Left
             * D-Pad Right - Rapid Rotate Right
             * D-pad Left + Share Button - Volkswagen Mode (do not use)
             * Gunner: (Start + B)
             *
             * */
        }
    }


    /**
     * Main mecanum movement function
     */
    private void drive() {
        //Front of robot is  intake side rn
        //Init variables
        double frontLeftPower;
        double frontRightPower;
        double rearLeftPower;
        double rearRightPower;


        double leftY = gamepad1.left_stick_y * driveReverseModeMultiplier;
        double leftX = gamepad1.left_stick_x * driveReverseModeMultiplier;
        double rightX = gamepad1.right_stick_x * driveReverseModeMultiplier;

//        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
//        blinkinLedDriver.setPattern(pattern);
        //DRIVE_STICK_THRESHOLD = deadzone
        if (rightX < -DRIVE_STICK_THRESHOLD || rightX > DRIVE_STICK_THRESHOLD || leftY < -DRIVE_STICK_THRESHOLD || leftY > DRIVE_STICK_THRESHOLD || leftX < -DRIVE_STICK_THRESHOLD || leftX > DRIVE_STICK_THRESHOLD) {
            //Get stick values and apply modifiers:
            double drive = (-gamepad1.left_stick_y * 1.10) * slowModeMultiplier;
            double turn = (gamepad1.right_stick_x * 1.25) * slowModeMultiplier;
            double strafe = (gamepad1.left_stick_x) * slowModeMultiplier;

            //Calculate each individual motor speed using the stick values:
            //range.clip calculates a value between min and max, change those values to reduce overall speed
            frontLeftPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            frontRightPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            rearLeftPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rearRightPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

            rb.drive(-frontRightPower, -frontLeftPower, -rearRightPower, -rearLeftPower); //Uses each of the motor values calculated above

//            telemetry.addData("Front-right motor", "%5.2f", frontRightPower);
//            telemetry.addData("Back-right motor", "%5.2f", rearRightPower);
//            telemetry.addData("Front-left motor", "%5.2f", frontLeftPower);
//            telemetry.addData("Back-left motor", "%5.2f", rearLeftPower);
//            telemetry.update();
        } else {
            rb.driveStop(); //Stop robot if no stick value (delete this if u want to drift lol)
        }

    }

    /**
     * Triggers servo that pushes rings into flywheel
     */
    //TODO: Setup teleop for 2 driver control w/ gamepad2
    private void shooter() throws InterruptedException {

        if (gamepad1.right_trigger > TRIGGER_THRESHOLD && !flywheelOn) {
            telemetry.addData("WARNING:", "flywheel is not running");
            telemetry.update();

        }

        if (gamepad1.right_trigger > .2) {

            rb.moveShooter(true); //Shoot
            if (powershotSpeedActive = false) {
                Thread.sleep(8); //8ms = time for ring to leave shooter
                rb.flywheel.setPower(1); //Increase speed as soon as ring is not in contact with flywheel to increase time back to normal speed
                Thread.sleep(100);
                rb.flywheel.setPower(FLYWHEEL_SPEED); //Return to normal speed
            } else {
                Thread.sleep(100 + 8);
            }
            Thread.sleep(67); //Wait a tiny bit before going back (originally 200 but this value is subtracted from prior Thread.sleep statements)
            rb.moveShooter(false);
            Thread.sleep(150); //Wait for flywheel to get back to 100 percent speed

        }

    }

    /**
     * Turns flywheel on and off
     */
    private void flywheel() throws InterruptedException {

        if ((gamepad1.right_bumper || gamepad2.right_bumper) && !flywheelOn) {
            telemetry.addData(">", "Flywheel ON");
            telemetry.update();
            rb.runFlywheel(true);
            flywheelOn = true;
            Thread.sleep(BUTTON_DELAY); //TODO: Make better code than this
        } else if ((gamepad1.right_bumper || gamepad2.right_bumper) && flywheelOn) {
            telemetry.addData(">", "Flywheel OFF");
            telemetry.update();
            rb.runFlywheel(false);
            flywheelOn = false;
            Thread.sleep(BUTTON_DELAY);
        }


    }

    /**
     * Turns intake on and off
     */
    private void intake() throws InterruptedException {

        if ((gamepad1.left_bumper || gamepad2.left_bumper) && !intakeOn) {
            telemetry.addData(">", "Intake ON");
            telemetry.update();
            rb.runIntake(true, false);
            intakeOn = true;
            System.out.println("iASO");
            //intakeAutoShutOff();

            Thread.sleep(BUTTON_DELAY);

        } else if ((gamepad1.left_bumper || gamepad2.left_bumper) && intakeOn) {
            telemetry.addData(">", "Intake OFF");
            telemetry.update();
            rb.runIntake(false, false);
            intakeOn = false;
            Thread.sleep(BUTTON_DELAY);
        }

        //Intake Auto Stop Code:
        if (intakeAutoStopped) {
            if (rb.getNumberOfRingsInHopper() < 3) {
                rb.runIntake(true, false);
                intakeAutoStopped = false;
                System.out.println("Intake Auto Resumed");
            }
        }

        if (rb.getNumberOfRingsInHopper() >= 3 && intakeAutoStopped == false && timeSincePossiblyDetecting3Rings == 0) {
            timeSincePossiblyDetecting3Rings = runtime.milliseconds();
        }

        if (rb.getNumberOfRingsInHopper() >= 3 && timeSincePossiblyDetecting3Rings < runtime.milliseconds() - 600 && timeSincePossiblyDetecting3Rings != 0) {
            if (rb.getNumberOfRingsInHopper() >= 3) {
                rb.runIntake(false, false);
                intakeAutoStopped = true;
                timeSincePossiblyDetecting3Rings = 0;
                System.out.println("Intake Auto Stopped");
            } else {
                timeSincePossiblyDetecting3Rings = 0;
            }
        }


    }


//    private void intakeAutoShutOff() throws InterruptedException {
//        if (rb.getNumberOfRingsInHopper() >= 3) {
//            Thread.sleep(1000);
//            System.out.println("stop1");
//            if (rb.getNumberOfRingsInHopper() >= 3) { //if there are...
//                rb.runIntake(false, false); //stop intake
//                intakeOn = false; //update variable
//                System.out.println("stop2");
//
//                while ((rb.getNumberOfRingsInHopper() <= 3) && opModeIsActive()) { //stay stuck in here until there aren't 3 rings
//                    System.out.println("waiting for less than 3...");
//
//                    if (rb.getNumberOfRingsInHopper() < 3) {
//                        System.out.println("less than 3 detected");
//                        rb.runIntake(true, false); //when we are no longer stuck in loop, turn intake back on
//                        intakeOn = true; // update variable and resume loop
//                    }
//                }
//            }
//        }
//    }


    /**
     * Turns intake on and off
     */

    private void intakeEject() throws InterruptedException {

        if (gamepad1.dpad_left) {
            telemetry.addData(">", "Intake EJECT");
            telemetry.update();
            rb.runIntake(true, true);
            intakeOn = true;
            intakeIsEjecting = true;
        }
//        } else if ((gamepad1.left_stick_button || gamepad2.left_stick_button) && intakeIsEjecting || gamepad1.left_bumper && intakeIsEjecting) {
//            telemetry.addData(">", "Intake OFF");
//            telemetry.update();
//            rb.runIntake(false, false);
//            intakeOn = false;
//            Thread.sleep(BUTTON_DELAY);
//        }
    }

    private void lifter() throws InterruptedException {

        if (gamepad1.x && lifterPosition == 2) {

            telemetry.addData(">", "Lifter DOWN");
            telemetry.update();
            rb.setLifterMotor(false, 1);
            lifterPosition = 0;
            rb.wobbleServo.setPosition(WOBBLE_OPEN);
            timeSinceActivatingWobbleDown = runtime.milliseconds();
            Thread.sleep(BUTTON_DELAY);

        } else if (gamepad1.x) {

            telemetry.addData(">", "Lifter UP");
            telemetry.update();
            rb.wobbleServo.setPosition(WOBBLE_CLOSED);
            rb.setLifterMotor(true, -1);
            lifterPosition = 2;
            Thread.sleep(BUTTON_DELAY);
        }

        if (gamepad1.b && lifterPosition == 2) {
            rb.lifterMotor.setPower(.75);
            rb.lifterMotor.setTargetPosition(LIFTER_MOTOR_MID);
            rb.wobbleServo.setPosition(WOBBLE_OPEN);
            lifterPosition = 1;
        } else if (gamepad1.b && lifterPosition == 0) {
            rb.lifterMotor.setPower(-1);
            rb.lifterMotor.setTargetPosition(LIFTER_MOTOR_MID);
            rb.wobbleServo.setPosition(WOBBLE_OPEN);
            lifterPosition = 1;
        }

        if (lifterPosition == 0 && (rb.wobbleRangeSensor.getDistance(DistanceUnit.MM) < WOBBLE_MININUM_DISTANCE) && (timeSinceActivatingWobbleDown + WOBBLE_MOVE_DOWN_DELAY < runtime.milliseconds())) {
            telemetry.addData(">", "Auto Closing: Lifter UP");
            telemetry.update();
            rb.setLifterMotor(true, -1);
            lifterPosition = 2;
            rb.wobbleServo.setPosition(WOBBLE_CLOSED);
            Thread.sleep(BUTTON_DELAY);
        }


    }

//    private void lifterAutoClose() {
//        if (lifterPosition == 0) {
//            while (lifterPosition == 0) {
//                if (rb.wobbleRangeSensor.getDistance(DistanceUnit.MM) > WOBBLE_MININUM_DISTANCE) {
//                    telemetry.addData(">", "Lifter UP");
//                    telemetry.update();
//                    rb.setLifterMotor(true, -1);
//                    rb.wobbleServo.setPosition(WOBBLE_CLOSED);
//                    lifterPosition = 2;
//
//                }
//            }
//        }
//    }
//
//    private void captureAngle() {
//        if (gamepad1.dpad_down) {
//            rb.driveForwardByEncoderAndIMU(200, rb.FL, .25, .10, DEFAULT_ACCELERATION_INCREMENT);
//        }
//    }

    private void rotateToAngle() throws InterruptedException {
        if (gamepad1.right_stick_button) {
            telemetry.addData("log:", "Driving to angle...");
            telemetry.update();
            rb.rotate(SHOOTER_DEFAULT_ROTATION, .8); //-8.6
            telemetry.addData("log:", "Done driving to angle!");
            telemetry.update();
        }
    }

    private void powershotSpeed() {
        if (gamepad1.dpad_down) {
            if (!powershotSpeedActive) {
                rb.flywheel.setPower(FLYWHEEL_POWERSHOT_SPEED);
                powershotSpeedActive = true;
            } else {
                rb.flywheel.setPower(FLYWHEEL_SPEED);
                powershotSpeedActive = false;
            }

            while (gamepad1.dpad_down && opModeIsActive()) { //wait until button is released to restart listener

            }
        }
    }

    private void rapidRotateLeft() throws InterruptedException {
        if (gamepad1.dpad_left) {
            while (gamepad1.dpad_left && opModeIsActive()) {
                rb.turn(-1);
            }
            Thread.sleep(BUTTON_DELAY);
        }
    }

    private void rapidRotateRight() throws InterruptedException {
        if (gamepad1.dpad_right) {
            while (gamepad1.dpad_right && opModeIsActive()) {
                rb.turn(1);
            }
            Thread.sleep(BUTTON_DELAY);


        }
    }


    private void volkswagenMode() throws InterruptedException {
        if (gamepad1.dpad_left && gamepad1.share && FLYWHEEL_SPEED != .7) {
            FLYWHEEL_SPEED = .7;
            telemetry.addData(">", "..1");
            telemetry.update();
            Thread.sleep(BUTTON_DELAY);
        } else if (gamepad1.dpad_left && gamepad1.share && FLYWHEEL_SPEED == .7) {
            FLYWHEEL_SPEED = .77;
            telemetry.addData(">", "..0");
            telemetry.update();
            Thread.sleep(BUTTON_DELAY);
        }
    }

    private void alignToGoal() throws InterruptedException {
        if (gamepad1.a) {
            telemetry.addData("STATUS:", "Rotating...");
            telemetry.update();
            //These are the variables we use for the trig equations:
            double readingFromSideSensor = rb.sideRangeSensor.getDistance(DistanceUnit.INCH);
            double sideLength = SIDE_TO_CENTER_DISTANCE + readingFromSideSensor;
            //probably look at this, does this still work?
            System.out.println("reading from side sensor = " + readingFromSideSensor);
            //- angle values go to the right and + go to the left

            double frontLength = CENTER_TO_TOWER_DISTANCE;

            double angleToRotate = -(Math.toDegrees(Math.atan(sideLength / frontLength)) - SHOOTER_DEFAULT_ROTATION);
            telemetry.addData("Angle Rotating To = ", angleToRotate);
            System.out.println("Angle Rotating to = " + angleToRotate);
            telemetry.update();
            rb.rotate(-angleToRotate, .8);

            telemetry.addData("STATUS:", "Done Rotating!!");
            telemetry.update();

            //1. Calculate distance from center of shooter to side wall
            //2. Assume approximate constant distance to front wall (or we can use sensors later)
            //3. do arctan (side wall distance / constant to front wall)
            //4. using angle from arctan, make rotation positive or negative
            //5. Subtract shooter offset angle
            //6. Rotate to angle and set LED status lights
        }
    }

    private void slowMode() {
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            slowModeMultiplier = .25;
        } else {
            slowModeMultiplier = 1;
        }
    }

    private void driveReverseMode() throws InterruptedException {
        if (gamepad1.left_stick_button && driveReverseModeMultiplier == 1) {
            driveReverseModeMultiplier = -1;
            while (gamepad1.left_stick_button &&opModeIsActive()){
                Thread.sleep(2);
            }
        } else if (gamepad1.left_stick_button && driveReverseModeMultiplier == -1){
            driveReverseModeMultiplier = -1;
            while (gamepad1.left_stick_button &&opModeIsActive()){
                Thread.sleep(2);
            }
        }
    }

    /*enum PowershotState{
        MOVING_SHOOTER_FORWARD,
        MOVING_SHOOTER_BACK,
        STRAFING,
    }*/
    private void powershotEndgame() throws InterruptedException {
        //PowershotState powershotState = PowershotState.MOVING_SHOOTER_FORWARD;
        if (gamepad1.dpad_up){
            telemetry.addData("STATUS:", "Endgame Powershots...");
            telemetry.update();
            double speedthing = FLYWHEEL_POWERSHOT_SPEED;
            rb.flywheel.setPower(speedthing);
            double powerstarttime = runtime.time();
            while(runtime.time() < powerstarttime + 0.45){

            }

            for (int x = rb.getNumberOfRingsInHopper() + 1; x > 0; x--) {
                /*res
                switch (powershotState){
                    case MOVING_SHOOTER_FORWARD:
                        rb.moveShooter(true); //Shot 4 just to make sure
                        if (r)
                    case MOVING_SHOOTER_BACK:
                    case STRAFING:
                }*/

                rb.moveShooter(true);
                double strafestarttime = runtime.time();
                while(runtime.time() < strafestarttime + 0.2) {
                }
                rb.moveShooter(false);
                if(x != 1) {
                    strafestarttime = runtime.time();
                    //rb.strafeRightByEncoderAndIMU(345, rb.FL, .4, .06);
                    rb.rotate(2.45, .3);
                    speedthing = speedthing - 0.035;
                    rb.flywheel.setPower(speedthing);
                    while(runtime.time() < strafestarttime + 0.45){

                    }
                }
            }
            /* When the code for the getNumberOfRingsInHopper is fixed
            while(rb.getNumberOfRingsInHopper() >= 1) {
                Thread.sleep(200);
                rb.moveShooter(true); //Shot 4 just to make sure
                Thread.sleep(200);
                rb.moveShooter(false);
                rb.strafeRightByEncoderAndIMU(350, rb.FL, .4, .06);
            }
             */
        }
    }
    private void AligntoTower() throws InterruptedException {
        /*if(gamepad1.a){
            rb.rotate(-30, .7);
            rb.rotate(30, .7);
        }*/
        if(gamepad1.y){
            if((rb.angleRangeSensor1.getDistance(DistanceUnit.MM) < 1000) && (rb.angleRangeSensor2.getDistance(DistanceUnit.MM) < 1000)) {
                telemetry.addData("STATUS:", "aligning to goal");
                telemetry.update();
                double lengthbetween = 77.9; //done in mm not inches -
                // this is the length between the two distance sensors
                //TODO: fix this value once the sensors are installed
                boolean rotatenegative = true;
                int rotatedirection = -1;
                double readingFromSideSensor1 = rb.angleRangeSensor1.getDistance(DistanceUnit.MM);
                double readingFromSideSensor2 = rb.angleRangeSensor2.getDistance(DistanceUnit.MM);
                //TODO: change SideSensor names accordingly once they are installed
                // sidesensor1 should be the one that is closer to the tower
                double distancedifference = java.lang.Math.abs(readingFromSideSensor1 - readingFromSideSensor2);
                //TODO: add a failsafe to make sure we are reading from the image
                if (distancedifference < 250) { //fix value here
                    if (readingFromSideSensor1 < readingFromSideSensor2) {
                        rotatenegative = false;
                        rotatedirection = 1;
                    }

                    //simpler code, maybe not as accurate? - NEED TO CHOSE ONE OR THE OTHER
                    /*
                    if (rotatenegative) {
                        while (readingFromSideSensor1 > readingFromSideSensor2) {
                            rb.rotate(-3, .7);
                        }
                    } else {
                        while (readingFromSideSensor1 < readingFromSideSensor2) {
                            rb.rotate(3, .7);
                        }
                    }
                    */
                    //other code, i think will be more accurate?
                    double degreestorotate = 180 * (Math.atan2(distancedifference, lengthbetween)) / Math.PI;
                    rb.rotate(rotatedirection * (degreestorotate / 1.05), .7);

                    //back to code that is needed for everything
                    /*
                    double timeBeforeWhile = runtime.milliseconds();
                    while ((rb.angleRangeSensor1.getDistance(DistanceUnit.MM) > 340) && (runtime.milliseconds() < timeBeforeWhile + 2000)) {
                        rb.strafe(-.6, -.6);
                    }
                    timeBeforeWhile = runtime.milliseconds();
                    while ((rb.angleRangeSensor2.getDistance(DistanceUnit.MM) > 20) && (runtime.milliseconds() < timeBeforeWhile + 2000)) {
                        rb.strafe(.6, .6);
                    }
                    */
                }
            }
        }
    }
    private void AligntoTower2() throws InterruptedException {
        if(gamepad1.a){
            if((rb.angleRangeSensor1.getDistance(DistanceUnit.MM) < 1000) && (rb.angleRangeSensor2.getDistance(DistanceUnit.MM) < 1000)) {
                telemetry.addData("STATUS:", "aligning to goal");
                telemetry.update();
                double lengthbetween = 77.9; //done in mm not inches -
                // this is the length between the two distance sensors
                //TODO: fix this value once the sensors are installed
                boolean rotatenegative = true;
                int rotatedirection = -1;
                double readingFromSideSensor1 = rb.angleRangeSensor1.getDistance(DistanceUnit.MM);
                double readingFromSideSensor2 = rb.angleRangeSensor2.getDistance(DistanceUnit.MM);
                //TODO: change SideSensor names accordingly once they are installed
                // sidesensor1 should be the one that is closer to the tower
                double distancedifference = java.lang.Math.abs(readingFromSideSensor1 - readingFromSideSensor2);
                //TODO: add a failsafe to make sure we are reading from the image
                if (distancedifference < 250) { //fix value here
                    if (readingFromSideSensor1 < readingFromSideSensor2) {
                        rotatenegative = false;
                        rotatedirection = 1;
                    }

                    //simpler code, maybe not as accurate? - NEED TO CHOSE ONE OR THE OTHER
                    /*
                    if (rotatenegative) {
                        while (readingFromSideSensor1 > readingFromSideSensor2) {
                            rb.rotate(-3, .7);
                        }
                    } else {
                        while (readingFromSideSensor1 < readingFromSideSensor2) {
                            rb.rotate(3, .7);
                        }
                    }*/
                    //other code, i think will be more accurate?
                    double degreestorotate = 180 * (Math.atan2(distancedifference, lengthbetween)) / Math.PI;
                    rb.rotate(rotatedirection * (degreestorotate / 1.05), .7);

                    //back to code that is needed for everything
                    double timeBeforeWhile = runtime.milliseconds();
                    while ((rb.angleRangeSensor1.getDistance(DistanceUnit.MM) > 340) && (runtime.milliseconds() < timeBeforeWhile + 2000)) {
                        rb.strafe(-.6, -.6);
                    }
                    timeBeforeWhile = runtime.milliseconds();
                    while ((rb.angleRangeSensor2.getDistance(DistanceUnit.MM) > 20) && (runtime.milliseconds() < timeBeforeWhile + 2000)) {
                        rb.strafe(.6, .6);
                    }
                }
            }
        }
    }



    private void ringBlocker() throws InterruptedException {

        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            rb.leftBlocker.setPosition(BLOCKER_LEFT_DOWN);
            rb.rightBlocker.setPosition(BLOCKER_RIGHT_DOWN);
        }

        else {
            rb.leftBlocker.setPosition(BLOCKER_LEFT_UP);
            rb.rightBlocker.setPosition(BLOCKER_RIGHT_UP);
        }



    }

    /**
     * Logs IMU data to telemetry
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

    private void delay(double delayTime) {

        double startTime = getRuntime();
        while ((getRuntime() < startTime + delayTime) && opModeIsActive()) {
            //wait for delay
        }
    }

}