 package org.firstinspires.ftc.teamcode.simpleBotCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.DEFAULT_ACCELERATION_INCREMENT;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.GEAR_RATIO_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_MOTOR_DOWN;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.LIFTER_MOTOR_UP;
import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.ringDistanceArray;

public class HardwareSimpleBot {
    /* Public OpMode members. */
    //motors
    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;
    public DcMotor flywheel = null;
    public DcMotor intake = null;
    public DcMotor lifterMotor = null;

    public DistanceSensor wobbleRangeSensor = null;
    public DistanceSensor hopperRangeSensor = null;
    public DistanceSensor sideRangeSensor = null;
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sideRangeSensor;
    public DistanceSensor wobble2mRangeSensor = null;
    Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) wobble2mRangeSensor;


    public ColorSensor wobbleColorSensor = null;
    //servos
    //public Servo lifter = null;
    public Servo shooter = null;
    public Servo wobbleServo = null;
    public Servo leftBlocker = null;
    //imu:
    public BNO055IMU imu;

    Orientation angles;
    Orientation lastAngles = new Orientation();
    public double globalAngle, power = .30, correction;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private final ElapsedTime period = new ElapsedTime();
    LinearOpMode opMode;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {

        this.opMode = opMode;
        //IMU Setup:
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FR = hwMap.get(DcMotor.class, "FR");
        FL = hwMap.get(DcMotor.class, "FL");
        BR = hwMap.get(DcMotor.class, "BR");
        BL = hwMap.get(DcMotor.class, "BL");
        flywheel = hwMap.get(DcMotor.class, "flywheel");
        intake = hwMap.get(DcMotor.class, "intake");
        lifterMotor = hwMap.get(DcMotor.class, "lifterMotor");

        // Define and Initialize Servos
        shooter = hwMap.get(Servo.class, "shooter");
        wobbleServo = hwMap.get(Servo.class, "wobble_servo");
        leftBlocker = hwMap.get(Servo.class, "left_blocker");

        //Define and Initialize BNO055IMU
        imu = hwMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);

        sideRangeSensor = hwMap.get(DistanceSensor.class, "side_range_sensor");

        wobbleRangeSensor = hwMap.get(DistanceSensor.class, "wobble_color_sensor");

        hopperRangeSensor = hwMap.get(DistanceSensor.class, "hopper_range_sensor");

        wobbleColorSensor = hwMap.get(ColorSensor.class, "wobble_color_sensor");

        wobble2mRangeSensor = hwMap.get(DistanceSensor.class, "wobble_2m_distance_sensor");


        // Define and Initialize LED's
        // blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");

        // Set motor directions
        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lifterMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set rb behavior when power is zero (BRAKE = brake, FLOAT = no brake)
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power for initialization
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        flywheel.setPower(0);
        intake.setPower(0);


        // Set all motor encoder options.
        lifterMotor.setTargetPosition(0);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }//End init code

    //Driving Functions

    /**
     * Stops all motors and applies zero power behavior from config (either BRAKE or FLOAT)
     */
    public void driveStop() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    /**
     * Sets all wheels to same drive speed.
     *
     * @param speed -1 to 1
     */
    public void drive(double speed) {
        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }

    /**
     * Sets wheels on left and right sides of robot to different speeds
     *
     * @param leftPower  Left Motors Speed -1 to 1
     * @param rightPower Right Motors Speed -1 to 1
     */
    public void drive(double leftPower, double rightPower) {
        FR.setPower(rightPower);
        FL.setPower(leftPower);
        BR.setPower(rightPower);
        BL.setPower(leftPower);
    }

    /**
     * Sets each drive motor to individual speeds, used by main mecanum driving function in simpleBotTeleOp.java
     *
     * @param frontrightPower Speed -1 to 1
     * @param frontleftPower  Speed -1 to 1
     * @param backrightPower  Speed -1 to 1
     * @param backleftPower   Speed -1 to 1
     */
    public void drive(double frontrightPower, double frontleftPower, double backrightPower, double backleftPower) {
        FR.setPower(frontrightPower);
        FL.setPower(frontleftPower);
        BR.setPower(backrightPower);
        BL.setPower(backleftPower);
    }

    /**
     * Strafes Mecanum Drivetrain
     *
     * @param speed
     */
    public void strafe(double speed) {
        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(speed);
    }

    /**
     * @param frontMotors Higher values = more right rotation
     * @param rearMotors  Higher values = more left rotation
     */
    public void strafe(double frontMotors, double rearMotors) {
        FR.setPower(frontMotors);
        FL.setPower(-frontMotors);
        BR.setPower(-rearMotors);
        BL.setPower(rearMotors);
    }

    /**
     * Rotates robot using equal speeds for all drive motors
     *
     * @param speed + values are left and - values are right
     */
    public void turn(double speed) {
        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(-speed);
        BL.setPower(speed);
    }

    /**
     * Controls Lifter motor
     *
     * @param moveUp Controls whether motor should move up or down
     * @param speed  Speed motor should travel at, can be either negative or positive
     */

    public void setLifterMotor(boolean moveUp, double speed) {
        if (!moveUp) {
            //System.out.println(LIFTER_MOTOR_DOWN);
            lifterMotor.setTargetPosition(LIFTER_MOTOR_DOWN);
            lifterMotor.setPower(Math.abs(speed)); //make sure speed is always positive
        } else if (moveUp) {
            //System.out.println(LIFTER_MOTOR_UP);
            speed = -Math.abs(speed); //make sure speed is negative
            lifterMotor.setTargetPosition(LIFTER_MOTOR_UP);
            lifterMotor.setPower(speed); //make sure speed is always positive


        }
    }

    /**
     * Drives robot southwest assuming North is tower side of field
     *
     * @param FLspeed
     * @param BRspeed
     */
    //This would be NorthEast if front of robot is intake side
    public void driveSouthWestAuto(double FLspeed, double BRspeed) {
        FL.setPower(FLspeed);
        BR.setPower(BRspeed);
    }

    //IMU Functions:

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     * @param power   Motor power during turn (should not be negative)
     */
    public void rotate(double degrees, double power) throws InterruptedException {

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            power = -power;

        } else if (degrees > 0) {   // turn left.
            //return; //power = power; dont need to change anything because assuming the power is already positive, that should make the robot turn right when put into the turn function
        } else return;

        // set power to rotate.
        turn(power);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0) {
            }

            while (opMode.opModeIsActive() && getAngle() > degrees) {
            }
        } else {  // left turn.
            System.out.println("Starting Left Turn..?  Starting Angle is: " + getAngle());
            while (opMode.opModeIsActive() && getAngle() < degrees) {
                System.out.println("Turning to angle... Angle right now is:" + getAngle());
            }
        }
        // turn the motors off.
        driveStop();
        System.out.println("Done rotating");

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Rotates to a global heading using IMU
     *
     * @param degrees
     * @param power
     * @throws InterruptedException
     */
    public void rotateToGlobalAngle(int degrees, double power) throws InterruptedException {

        // restart imu movement tracking.
        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = currentAngle.firstAngle;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            power = -power;

        } else if (degrees > 0) {   // turn left.
            return;//power = power; dont need to change anything because assuming the power is already positive, that should make the robot turn right when put into the turn function
        } else return;

        // set power to rotate.
        turn(power);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0) {
            }

            while (opMode.opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        driveStop();

        // wait for rotation to stop.
        Thread.sleep(100);

        // reset angle tracking on new heading.
        resetAngle();


//        //Old Code:
//        int currentAngle = (int) getGlobalAngle(); //Get current angle to determine which direction to rotate towards
//        System.out.println("Current Raw Angle: " + currentAngle);
//        //Convert target angle from (-180 -- 180) range to (0 -- 360)
//        int convertedTargetAngle; //IMPORTANT: You only need to use this if you are capturing an angle from the IMU in the -180 to 180 format, otherwise make sure this is off if you are trying to input your own angle. TODO: Double check this for acutal matches
//        if (targetAngle > 0) {
//            convertedTargetAngle = targetAngle;
//        } else if (targetAngle < 0) {
//            convertedTargetAngle = 180 + Math.abs(targetAngle);
//        } else {
//            convertedTargetAngle = targetAngle;
//        }
//        System.out.println("Converted Target Angle: " + convertedTargetAngle);
//        //Convert current angle from (-180 -- 180) range to (0 -- 360)
//        int convertedCurrentAngle;
//        if (currentAngle > 0) {
//            convertedCurrentAngle = currentAngle;
//        } else if (currentAngle < 0) {
//            convertedCurrentAngle = 180 + Math.abs(currentAngle);
//        } else {
//            convertedCurrentAngle = currentAngle;
//        }
//        System.out.println("Converted Current Angle: " + convertedCurrentAngle);
//        //Calculate which way to rotate:
//        //TARGET ANGLE = 0 DEGREES
//        //CURRENT ANGLE = idk 90 DEGREES
//        //SHOULD TURN RIGHT?
//
//        //TARGET ANGLE = 0 DEGREES
//        //CURRENT ANGLE = 270 DEGREES
//        //SHOULD TURN LEFT
//
//        //TARGET ANGLE = 270 DEGREES
//        //CURRENT ANGLE = 45 DEGREES
//        //SHOULD TURN RIGHT
//
//        int diff = Math.abs(convertedTargetAngle - convertedCurrentAngle);
////        if (diff < 0) {
////            diff += 360;
////        }
//
//
//        if (diff > 180) {
//            System.out.println("Starting Turning Right...");
//            InitialPower = -InitialPower; //turn right
//
//        } else {
//            System.out.println("Starting Turning Left...");
//
//        }
//
//
//        turn(InitialPower);
//
//        // rotate until turn is completed.
//        if (diff < 180) {
//            // On right turn we have to get off zero first.
////            while (opMode.opModeIsActive() && getGlobalAngle() == 0) {
////            }
//
//
//            while (opMode.opModeIsActive() && getGlobalAngle() > 22.5 + targetAngle) {
//            }
//            driveStop();
//            turn(.25);
//            while (opMode.opModeIsActive() && getGlobalAngle() > 8 + targetAngle) {
//            }
//            driveStop();
//            turn(.10);
//            while (opMode.opModeIsActive() && getGlobalAngle() > targetAngle) {
//            }
//
//        } else if (diff > 180) {   // left turn.
//            while (opMode.opModeIsActive() && getGlobalAngle() < targetAngle - 22.5) {
//            }
//            turn(-.25);
//            while (opMode.opModeIsActive() && getGlobalAngle() < targetAngle - 8) {
//            }
//            turn(.10);
//            while (opMode.opModeIsActive() && getGlobalAngle() < targetAngle - 8) {
//            }
//
//        }
//
//
//        // turn the motors off.
//        driveStop();
//        System.out.println("Turn Completed!");
//
//        // wait for rotation to stop.
//        Thread.sleep(100);

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Get current heading relative to intialization angle (i think)
     *
     * @return Current heading / angle from -180 to 180
     */
    public double getGlobalAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //Encoder Drive Functions:

    /**
     * Drives forward using 1 encoder
     * IMPORTANT: this is replaced by driveForwardByEncoderAndIMU which uses the imu to maintain heading
     *
     * @param positionChange This should be positive or negative based on direction
     * @param motor          rb.FL
     * @param power          Always positive, direction controlled by positionChange
     */
    public void driveForwardByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;
        double currentPower = 0.2; //Always start at 0.2 power

        if (positionChange > 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition * .75) {

                if (currentPower >= power) {
                    currentPower = power;
                } else {
                    currentPower = currentPower + DEFAULT_ACCELERATION_INCREMENT;
                }
                drive(currentPower);
            }
            //deceleration code:

            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {

                drive(Range.clip(Math.abs(motor.getCurrentPosition() - targetPosition) / motor.getCurrentPosition() + oldPosition, .1, 1));

            }

            driveStop();
        } else if (positionChange < 0) {
            drive(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    /**
     * Drives forward using 1 encoders
     *
     * @param positionChange        This should be positive or negative based on direction
     * @param motor                 rb.FL
     * @param power                 Always positive, direction controlled by positionChange
     * @param correctionGain        Controls how much corrections is applied to the motors if the angle is off, default is .06 i think
     * @param accelerationIncrement Positive Linear increment that is used to increase acceleration, default is DEFAULT_ACCELERATION_INCREMENT. To get no acceleration, just make this 1
     */
    public void driveForwardByEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain, double accelerationIncrement) {
        DcMotor oppositeMotor;
        if (motor == FL) {
            oppositeMotor = BL;
        } else if (motor == FR) {
            oppositeMotor = BR;
        } else if (motor == BR) {

        }
        power = Math.abs(power);

        positionChange = (int) (positionChange * GEAR_RATIO_MULTIPLIER); //Used for changing gear ratios without changing all values in code

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;
        if (positionChange < 0) { //Make acceleration increment positive or negative based on power
            accelerationIncrement = -accelerationIncrement;
        }

        if (positionChange > 0) {
            double currentPower = 0.2; //Always start at 0.2 power
            drive(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition * .8) {
                currentPower += accelerationIncrement;
                if (currentPower > power) {
                    currentPower = power; //if .2 + accelerationIncrement is bigger than the target power (over max power)
                }

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                drive((currentPower + correction), (currentPower - correction));
//                Thread.yield();
            }

            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
//                double decelerationSpeed = (Range.clip(Math.abs(motor.getCurrentPosition() - FLtargetPosition) / motor.getCurrentPosition() + FLoldPosition, .1, power));
//                System.out.println(decelerationSpeed);
                if (currentPower > .35) { //If we're driving slower than .35, i don't think we need to rly worry about decceleration
                    correction = checkCorrection(correctionGain);
                    drive((currentPower / 2 + correction), (currentPower / 2 - correction));
                } else {
                    correction = checkCorrection(correctionGain);
                    drive((currentPower + correction), (currentPower - correction));
                }
            }

            driveStop();

        } else if (positionChange < 0) {
            double currentPower = -0.2; //Always start at 0.2 power
            drive(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition * 1.2) {

                if (currentPower >= -power) {
                    currentPower = -power;
                } else {
                    currentPower -= DEFAULT_ACCELERATION_INCREMENT;

                }
                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                drive((currentPower + correction), (currentPower - correction));
                Thread.yield();
            }

            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                double decelerationSpeed = (Range.clip(Math.abs(motor.getCurrentPosition() - targetPosition) / motor.getCurrentPosition() + oldPosition, power, -.1));
                System.out.println(decelerationSpeed);
                correction = checkCorrection(correctionGain);
                drive((decelerationSpeed + correction), (decelerationSpeed - correction));

            }

            driveStop();
        }

    }

    /**
     * Drives forward with IMU to maintain heading to line on field
     *
     * @param power     NEGATIVE OR POSITIVE POWER CONTROLS DIRECTION
     * @param lineColor Linecolor to detect (white, blue)
     */
//    public void driveForwardByIMUtoLine(double power, String lineColor) {
//
//
//        if (lineColor.equals("white")) {
//            if (power > 0) {
//                drive(power);
//                while (opMode.opModeIsActive() && remoteAuto.groundColorSensor.alpha() < WHITE_ALPHA_THRESHOLD) {
//
//                    int currentAngle; //Stores heading at beginning of function
//                    // Use IMU to drive in a straight line.
//                    correction = checkCorrection(.1);
//                    drive((power + correction), (power - correction));
//                    Thread.yield();
//                }
//
//                driveStop();
//            } else if (power < 0) {
//                drive(-power);
//                while (opMode.opModeIsActive() && remoteAuto.groundColorSensor.alpha() < WHITE_ALPHA_THRESHOLD) {
//                    // Use IMU to drive in a straight line.
//                    correction = checkCorrection(.1);
//                    drive((power - correction), (power + correction));
//                    Thread.yield();
//                }
//                driveStop();
//            }
//        }
//
//
//    }


    /**
     * Checks how much of a correction to make
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkCorrection(Double correctionGain) {
        // gain value determines how sensitive the correction is to direction changes
        //default = .10
        double correction, angle, gain = correctionGain;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Drives southwest, assuming tower goals are North and robot shooter is facing towards tower goals.
     *
     * @param positionChange Positive or negative value controls direction, Negative Values move north east
     * @param motor          RB.FL
     * @param power          Positive value, 0-1
     * @param correctionGain
     */
    public void autoDriveSouthWestWithEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain) {
        power = Math.abs(power);
        positionChange = (int) (positionChange * GEAR_RATIO_MULTIPLIER); //Used for changing gear ratios without changing all values in code

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition - positionChange;
        double currentPower = 0.2; //Always start at 0.2 power

        if (positionChange > 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto(-(power - correction), -(power + correction));
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto((power + correction), (power - correction));
                Thread.yield();
            }
            driveStop();
        }

    }

    /**
     * Drives southwest, assuming tower goals are North and robot shooter is facing towards tower goals.
     *
     * @param positionChange Positive or negative value controls direction, Negative Values move north east
     * @param motor          RB.FL
     * @param power          Positive value, 0-1
     * @param correctionGain
     */
    //We should incorporate this into the code now.
    public void autoDriveNorthEastWithEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain) {

        power = Math.abs(power);

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition - positionChange;

        if (positionChange > 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto(-(power - correction), -(power + correction));
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                driveSouthWestAuto((power - correction), (power + correction));
                Thread.yield();
            }
            driveStop();
        }

    }


    public void strafeRightByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            strafe(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            strafe(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    /**
     * Strafes to the right using IMU to maintain angle and encoders for position
     *
     * @param positionChange Use negative to strafe to the left
     * @param motor          rb.FL for example
     * @param power          0 - 1, positive values only
     */

    public void strafeRightByEncoderAndIMU(int positionChange, DcMotor motor, double power, double correctionGain) {

        power = Math.abs(power);
        positionChange = (int) (positionChange * GEAR_RATIO_MULTIPLIER); //Used for changing gear ratios without changing all values in code

        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition - positionChange;

        //Current pos = 500
        //pos change = 200
        //target = 700
        //new target = 300

        if (positionChange > 0) {
            double currentPower = 0.2; //Always start at 0.2 power
            strafe(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {

                if (currentPower >= power) {
                    currentPower = power;
                } else {
                    currentPower += DEFAULT_ACCELERATION_INCREMENT;

                }

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                strafe((currentPower - correction), (currentPower + correction));
                Thread.yield();
            }
            driveStop();
            System.out.println("Ok, done running.");


        } else if (positionChange < 0) {
            double currentPower = -0.2; //Always start at 0.2 power
            strafe(currentPower);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {

                if (currentPower >= -power) {
                    currentPower = -power;
                } else {
                    currentPower -= DEFAULT_ACCELERATION_INCREMENT;

                }

                // Use IMU to drive in a straight line.
                correction = checkCorrection(correctionGain);
                strafe((currentPower - correction), (currentPower + correction));

                Thread.yield();
            }
            driveStop();
        }

        driveStop();

    }


    public void turnClockwiseByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            turn(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            turn(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }
    }

    public void driveWithLeftMore(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            FR.setPower(power);
            FL.setPower(power);
            BR.setPower(power);
            BL.setPower(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            FR.setPower(-power * .66);
            FL.setPower(-power);
            BR.setPower(-power * .66);
            BL.setPower(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    public void driveWithRightMore(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            FR.setPower(power);
            FL.setPower(power);
            BR.setPower(power);
            BL.setPower(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }

            driveStop();
        } else if (positionChange < 0) {
            FR.setPower(-power);
            FL.setPower(-power * .66);
            BR.setPower(-power);
            BL.setPower(-power * .66);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    //MECHANISM FUNCTIONS:

    /**
     * Moves Servo that pushes rings into flywheel
     *
     * @param isOut True = servo goes out, False = idk
     */
    public void moveShooter(boolean isOut) {
        if (isOut) {
            shooter.setPosition(simpleBotConstants.SHOOTER_OUT);
        } else {
            shooter.setPosition(simpleBotConstants.SHOOTER_IN);
        }
    }

    /**
     * Turns flywheel on or off
     *
     * @param isOn True = flywheel on, False = flywheel off
     */
    public void runFlywheel(boolean isOn) {

        if (isOn) {
            flywheel.setPower(FLYWHEEL_SPEED);
        } else {
            flywheel.setPower(0);
        }
    }

//    public void setLifter(boolean isUp) {
//        if (isUp) {
//            lifter.setPosition(simpleBotConstants.LIFTER_UP);
//        } else {
//            lifter.setPosition(simpleBotConstants.LIFTER_DOWN);
//        }
//    }

    /**
     * Turns intake on and off
     *
     * @param isOn  True = intake on, False = intake off
     * @param eject True = runs in reverse to unstuck rings, False = Runs normally
     */
    public void runIntake(boolean isOn, boolean eject) {

        if (isOn) {
            if (eject)
                intake.setPower(-INTAKE_SPEED);
            else {
                intake.setPower(INTAKE_SPEED);
            }
        } else {
            intake.setPower(0);
        }
    }

    /**
     * Returns Number of Rings in the Hopper from the Range sensor
     *
     * @return 0-3 Rings
     */

    public int getNumberOfRingsInHopper() {

        int currentDistance = (int) hopperRangeSensor.getDistance(DistanceUnit.MM);

        int distance = Math.abs(ringDistanceArray[0] - currentDistance);
        int idx = 0;

        for (int c = 1; c < ringDistanceArray.length; c++) {
            int cdistance = Math.abs(ringDistanceArray[c] - currentDistance);
            if (cdistance < distance) {
                idx = c;
                distance = cdistance;
            }
        }//i don't think this works...? could be mistaken between distances, could have two of them
        //and then it will just take the latter. Ex: when you run the robot right now, 0 rings
        //will sometimes read 1 ring - need to work on this - wait nvm, this example doesn't check
        //out with the issue, but it seems to be off


        /* Numbers from 5/27/2021
        - 3 rings ~ 75-87 mm
        - 2 rings ~ 90 - 110
        - 1 ring  ~ 115 - 126
        - 0 rings ~ 126 - 135

         */

        return idx;

    }


//    //OLD ROBOT FUNCTIONS

////    public void ledColorFLashYellow() {
////        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
////        blinkinLedDriver.setPattern(pattern);
////    }
////
////    public void ledColorGreen() {
////        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
////        blinkinLedDriver.setPattern(pattern);
////    }
////
////    public void ledColorOrange() {
////        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
////        blinkinLedDriver.setPattern(pattern);
////    }
////
////    public void flashRed() {
////        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
//        blinkinLedDriver.setPattern(pattern);
//    }
//
//    public void ledOff() {
//        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
//        blinkinLedDriver.setPattern(pattern);
//    }

}

