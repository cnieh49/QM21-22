package org.firstinspires.ftc.teamcode.simpleBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.BUTTON_DELAY;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.CENTER_TO_TOWER_DISTANCE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_POWERSHOT_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_MOTOR_MID;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SHOOTER_DEFAULT_ROTATION;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SIDE_TO_CENTER_DISTANCE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SIDE_WALL_TO_TOWER_DISTANCE;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.TRIGGER_THRESHOLD;


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
    private boolean lifterUp = true; //Default is true because needs to start up to stay in 18in
    private boolean powershotSpeedActive = false;



    // State used for updating telemetry
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initializing");
        telemetry.update();


        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();
        rb.init(hardwareMap, this); //runs init stuff in HardwareSimpleBot.java
        rb.lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Hardware Map Initialized");
        telemetry.update();

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        //IMPORTANT: The gyro will not calibrate unless the robot is not moving, make sure the robot is still during initialization.
        while (!isStopRequested() && !rb.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status: ", rb.imu.getCalibrationStatus().toString());

        composeTelemetry();

        telemetry.addData("Status", "Initialized, Ready to Start");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        rb.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart(); //Everything up to here is initialization
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive(); //Drive robot with sticks
            shooter(); // Triggers servo that pushes rings into flywheel
            flywheel(); // Turns flywheel on and off
            intake();//Turns intake on and off
            intakeEject(); //Runs intake in reverse for emergencies
            lifter();//Moves lifter up and down
            powershotSpeed();
            //captureAngle(); //TESTING ONLY: Captures angle
            rotateToAngle(); //TESTING ONLY (for now): Rotates
            rapidRotateLeft(); //TESTING ONLY: Rotates to the left at max speed
            rapidRotateRight(); //TESTING ONLY: Rotates to the right at max speed
            volkswagenMode();
            alignToGoal();

            telemetry.addData("Side Distance in Inches:", String.valueOf(rb.sensorRangeSide.getDistance(DistanceUnit.INCH)));
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
             * Right Stick Button - Rotate -8.6 degrees to shoot straight
             * Left Stick Button - Intake Eject for Emergencies
             * D-Pad Down - Slows down flywheel for powershots
             * D-Pad Left - Rapid Rotate Left
             * D-Pad Right - Rapid Rotate Right
             * D-pad Left + Share Button - Volkswagen Mode (do not use)
             * Gunner: (Start + B) //TODO: Add gunner controls
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


        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

//        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
//        blinkinLedDriver.setPattern(pattern);
        //DRIVE_STICK_THRESHOLD = deadzone
        if (rightX < -DRIVE_STICK_THRESHOLD || rightX > DRIVE_STICK_THRESHOLD || leftY < -DRIVE_STICK_THRESHOLD || leftY > DRIVE_STICK_THRESHOLD || leftX < -DRIVE_STICK_THRESHOLD || leftX > DRIVE_STICK_THRESHOLD) {
            //Get stick values and apply modifiers:
            double drive = -gamepad1.left_stick_y * 1.10;
            double turn = gamepad1.right_stick_x * 1.25;
            double strafe = gamepad1.left_stick_x;

            //Calculate each individual motor speed using the stick values:
            //range.clip calculates a value between min and max, change those values to reduce overall speed
            frontLeftPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            frontRightPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            rearLeftPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rearRightPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

            rb.drive(-frontRightPower, -frontLeftPower, -rearRightPower, -rearLeftPower); //Uses each of the motor values calculated above

            telemetry.addData("Front-right motor", "%5.2f", frontRightPower);
            telemetry.addData("Back-right motor", "%5.2f", rearRightPower);
            telemetry.addData("Front-left motor", "%5.2f", frontLeftPower);
            telemetry.addData("Back-left motor", "%5.2f", rearLeftPower);
            telemetry.update();
        } else {
            rb.driveStop(); //Stop robot if no stick value (delete this if u want to drift lol)
        }

    }

    /**
     * Triggers servo that pushes rings into flywheel
     */
    //TODO: Setup teleop for 2 driver control w/ gamepad2
    //TODO: Configure delay variable in simpleBotConstants.java
    private void shooter() throws InterruptedException {

        if (gamepad1.right_trigger > TRIGGER_THRESHOLD && !flywheelOn) { //TODO: Figure out why trigger gamepad1.right_trigger > .5f isnt working
            telemetry.addData("WARNING:", "flywheel is not running");
            telemetry.update();

        }

        if (gamepad1.right_trigger > .2) {

            rb.moveShooter(true); //Shoot
            //TODO:Make screen red to indicate wait
            if (powershotSpeedActive = false) {
                Thread.sleep(8); //8ms = time for ring to leave shooter
                rb.flywheel.setPower(1); //Increase speed as soon as ring is not in contact with flywheel to increase time back to normal speed
                Thread.sleep(117);
                rb.flywheel.setPower(FLYWHEEL_SPEED); //Return to normal speed
            } else {
                Thread.sleep(117 + 8);
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

        if (gamepad1.right_bumper && !flywheelOn) {
            telemetry.addData(">", "Flywheel ON");
            telemetry.update();
            rb.runFlywheel(true);
            flywheelOn = true;
            Thread.sleep(BUTTON_DELAY); //TODO: Make better code than this
        } else if (gamepad1.right_bumper && flywheelOn) {
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

        if (gamepad1.left_bumper && !intakeOn) {
            telemetry.addData(">", "Intake ON");
            telemetry.update();
            rb.runIntake(true, false);
            intakeOn = true;
            Thread.sleep(BUTTON_DELAY);

        } else if (gamepad1.left_bumper && intakeOn) {
            telemetry.addData(">", "Intake OFF");
            telemetry.update();
            rb.runIntake(false, false);
            intakeOn = false;
            Thread.sleep(BUTTON_DELAY);
        }
    }

    /**
     * Turns intake on and off
     */
    private void intakeEject() throws InterruptedException {

        if (gamepad1.left_stick_button && !intakeOn) {
            telemetry.addData(">", "Intake EJECT");
            telemetry.update();
            rb.runIntake(true, true);
            intakeOn = true;
            intakeIsEjecting = true;
            Thread.sleep(BUTTON_DELAY);

        } else if (gamepad1.left_stick_button && intakeIsEjecting || gamepad1.left_bumper && intakeIsEjecting) {
            telemetry.addData(">", "Intake OFF");
            telemetry.update();
            rb.runIntake(false, false);
            intakeOn = false;
            Thread.sleep(BUTTON_DELAY);
        }
    }

    private void lifter() throws InterruptedException {
        if (gamepad1.x && lifterUp) {

            telemetry.addData(">", "Lifter DOWN");
            telemetry.update();
            rb.setLifterMotor(false, 1);
            lifterUp = false;
            Thread.sleep(BUTTON_DELAY);

        } else if (gamepad1.x) {

            telemetry.addData(">", "Lifter UP");
            telemetry.update();
            rb.setLifterMotor(true, -1);
            lifterUp = true;
            Thread.sleep(BUTTON_DELAY);

        }

        if (gamepad1.b && lifterUp) {
            rb.lifterMotor.setPower(.75);
            rb.lifterMotor.setTargetPosition(LIFTER_MOTOR_MID);
            lifterUp = false;
        } else if (gamepad1.b && !lifterUp) {
            rb.lifterMotor.setPower(-1);
            rb.lifterMotor.setTargetPosition(LIFTER_MOTOR_MID);
            lifterUp = false;
        }


    }

    private void captureAngle() {
        if (gamepad1.dpad_down) {
            rb.driveForwardByEncoderAndIMU(200, rb.FL, .25, .10, DEFAULT_ACCELERATION_INCREMENT);
        }
    }

    private void rotateToAngle() throws InterruptedException {
        if (gamepad1.right_stick_button) {
            telemetry.addData("log:", "Driving to angle...");
            telemetry.update();
            rb.rotate(SHOOTER_DEFAULT_ROTATION, .4); //-8.6
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

    private void rapidRotateLeft() {
        if (gamepad1.dpad_left) {
            rb.turn(-1);

        }
    }

    private void rapidRotateRight() {
        if (gamepad1.dpad_right) {

            rb.turn(1);


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
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            telemetry.addData("STATUS:", "Rotating...");
            telemetry.update();
            double readingFromSideSensor = rb.sensorRangeSide.getDistance(DistanceUnit.INCH);
            double sideLength = SIDE_WALL_TO_TOWER_DISTANCE - (SIDE_TO_CENTER_DISTANCE + readingFromSideSensor);
            //- angle values go to the right and + go to the left

            double frontLength = CENTER_TO_TOWER_DISTANCE;

            double angleToRotate = -(Math.toDegrees(Math.atan(sideLength / frontLength)) + SHOOTER_DEFAULT_ROTATION);

            rb.rotate(-angleToRotate, .8);

            telemetry.addData("STATUS:", "Done Rotating!!");
            telemetry.update();
            //TODO: Change led color + maybe auto shoot if it works well?

            //1. Calculate distance from center of shooter to side wall
            //2. Assume approximate constant distance to front wall (or we can use sensors later)
            //3. do arctan (side wall distance / constant to front wall)
            //4. using angle from arctan, make rotation positive or negative
            //5. Subtract shooter offset angle
            //6. Rotate to angle and set LED status lights
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

}