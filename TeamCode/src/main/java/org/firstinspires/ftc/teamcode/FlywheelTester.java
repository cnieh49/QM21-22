package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.simpleBotCode.simpleBotConstants.FLYWHEEL_SPEED;


@TeleOp(name = "Flywheel Tester", group = "Test")
public class FlywheelTester extends LinearOpMode {

    // Define class members
    private double initialSpeed = FLYWHEEL_SPEED; // Start at halfway position so was .5

    private final double INCREMENT = 0.01;
    private final int DELAY = 100;//was 100

    @Override
    public void runOpMode() {

        DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        // Wait for the start button
        telemetry.addData(">", "Press Start to Start?.");
        telemetry.update();


        telemetry.addData(">", "Motor Speed: ");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                // Keep stepping up until we hit the max value.
                initialSpeed += INCREMENT;
            } else if (gamepad1.dpad_down) {
                // Keep stepping down until we hit the min value.
                initialSpeed -= INCREMENT;
            }


            // Display the current value
            telemetry.addData("Flywheel Speed (dpad up and dpad down)", "%5.2f", initialSpeed);

            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            flywheel.setPower(initialSpeed);


            sleep(DELAY);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}