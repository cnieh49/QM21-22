package org.firstinspires.ftc.teamcode.simpleBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_DOWN;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_MID;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.SHOOTER_DEFAULT_ROTATION;


@TeleOp(name = "!QM TeleOP", group = "Sensor")
public class simpleBotTeleOp extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance(); //Comment this out when not using dashboard
    private final HardwareSimpleBot rb = new HardwareSimpleBot();
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    //Setup variables:
    private boolean flywheelOn = false;
    private final boolean shooterOut = false;
    private boolean intakeOn = false;
    private boolean lifterUp = true; //Default is true becausae needs to start up to stay in 18in

    // State used for updating telemetry
    Orientation angles;
    //Acceleration gravity;

//    RevBlinkinLedDriver blinkinLedDriver;
//    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();
        rb.init(hardwareMap, this); //runs init stuff in HardwareSimpleBot.java
        telemetry.addData("Status", "Hardware Map Initialized");
        telemetry.update();

//        telemetry.addData("Status", "Initializing Servo Positions...");
//        telemetry.update();
//        rb.setLifter(true); //Activate Lifter Up for init
//        telemetry.addData("Status", "Servo Positions Initialized");
//        telemetry.update();

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !rb.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status: ", rb.imu.getCalibrationStatus().toString());

        composeTelemetry();


        telemetry.addData("Status", "Initialized, Ready to Start");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart(); //Everything up to here is initialization
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive(); //Drive robot with sticks
            shooter(); // Triggers servo that pushes rings into flywheel
            flywheel(); // Turns flywheel on and off
            intake();//Turns intake on and off
            lifter();//Moves lifter up and down
            captureAngle(); //TESTING ONLY: Captures angle
            rotateToAngle(); //TESTING ONLY (for now): Rotates to captured angle
            volkswagenMode();


            telemetry.update(); //for imu display

            //  Show the elapsed game time and wheel power.
//            telemetry.addD    ata("Status", "Run Time: " + runtime.toString());
//            telemetry.update();

            /* CONTROLS: //TODO: Update controls
             * Driver: (Start + A)
             * Left Stick - Movement
             * Right Stick - Rotation
             *
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

        if (gamepad1.a) { //TODO: Figure out why trigger gamepad1.right_trigger > .5f isnt working
            telemetry.addData(">", "Shooter Out!");
            telemetry.update();

            rb.moveShooter(true); //Shoot
            //Make screen red to indicate wait
            Thread.sleep(8); //8ms = time for ring to leave shooter
            rb.flywheel.setPower(.92);
            Thread.sleep(117);
            rb.flywheel.setPower(FLYWHEEL_SPEED);

            Thread.sleep(67); //Wait a tiny bit before going back (originally 200)
            rb.moveShooter(false);
            Thread.sleep(150); //Wait for flywheel to get back to 100 percent speed

        } else if (gamepad1.a && !flywheelOn) {
            //TODO: Make it so that is impossible to shoot if the flywheel is not on
            telemetry.addData("WARNING:", "flywheel is not running");
            telemetry.update();

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
            Thread.sleep(250);
        } else if (gamepad1.right_bumper && flywheelOn) {
            telemetry.addData(">", "Flywheel OFF");
            telemetry.update();
            rb.runFlywheel(false);
            flywheelOn = false;
            Thread.sleep(250);
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
            Thread.sleep(250);
        } else if (gamepad1.left_bumper && intakeOn) {
            telemetry.addData(">", "Intake OFF");
            telemetry.update();
            rb.runIntake(false, false);
            intakeOn = false;
            Thread.sleep(250);
        }
    }

    private void lifter() throws InterruptedException {
        if (gamepad1.x && lifterUp) {
            telemetry.addData(">", "Lifter DOWN");
            telemetry.update();
            rb.lifter.setPosition(LIFTER_MID);
            Thread.sleep(1000);
            rb.lifter.setPosition(LIFTER_DOWN);
            lifterUp = false;
            Thread.sleep(250);
        } else if (gamepad1.x) {
            telemetry.addData(">", "Lifter UP");
            telemetry.update();
            rb.setLifter(true);
            lifterUp = true;
            Thread.sleep(250);
        }
    }


    private void captureAngle() throws InterruptedException {
        if (gamepad1.dpad_down) {
            rb.driveForwardByEncoderAndIMU(200, rb.FL, .25, .10);
        }
    }

    private void rotateToAngle() throws InterruptedException {
        if (gamepad1.right_stick_button) {
            telemetry.addData("log:", "Driving to angle...");
            telemetry.update();
            rb.rotate(SHOOTER_DEFAULT_ROTATION, .25); //-8.6
            telemetry.addData("log:", "Done driving to angle!");
            telemetry.update();

        }
    }


    private void volkswagenMode() {
        if (gamepad1.dpad_left && gamepad1.share && FLYWHEEL_SPEED != .7) {
            FLYWHEEL_SPEED = .7;
            telemetry.addData(">", "..1");
            telemetry.update();
        } else if (gamepad1.dpad_left && gamepad1.share && FLYWHEEL_SPEED == .7) {
            FLYWHEEL_SPEED = .77;
            telemetry.addData(">", "..0");
            telemetry.update();
        }
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


    //Old Functions: Keeping for reference

//    private void intake(){
//        boolean rightb = gamepad2.right_bumper;
//        boolean leftb = gamepad2.left_bumper;
//
//        if (rightb && gamepad2.right_stick_button) {
//            rb.intakeIn(-1);
//        } else if (leftb) {
//            rb.intakeIn();
//        }
//        else if (rightb) {
//            rb.intakeOut();
//        }
//        else{
//            rb.intakeStop();
//        }
//
//    }
//
//    private void platform(){
//        boolean dpadUp = gamepad2.dpad_up;
//        boolean dpadDown = gamepad2.dpad_down;
////        if(rightTrigger>.15){
////            rb.setPlatformUp(false);
////        }else if(leftTrigger>.15){
////            rb.setPlatformUp(true);
////        }
//
//        if(dpadUp){
//            rb.setPlatformUp(true);
//        }else if(dpadDown){
//            rb.setPlatformUp(false);
//        }
//    }
//
//    private void tapeMeasure(){
////        double leftx = gamepad2.left_stick_y;
////
////        if(leftx > .1){
////            rb.tapeIn();
////        } else if(leftx < -.1){
////            rb.tapeOut();
////        }else{
////            rb.tapeStop();
////        }
//    }
//

}