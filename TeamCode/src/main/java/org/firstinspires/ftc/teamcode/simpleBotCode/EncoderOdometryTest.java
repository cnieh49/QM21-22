package org.firstinspires.ftc.teamcode.simpleBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "EncoderOdometryTest", group = "Test")
public class EncoderOdometryTest extends LinearOpMode {
    private final HardwareSimpleBot rb = new HardwareSimpleBot();

    private final MecanumOdometry odometry = new MecanumOdometry();

    @Override
    public void runOpMode() throws InterruptedException {
        rb.init(hardwareMap, this);
        odometry.init(hardwareMap);
        telemetry.addData("FR Encoder", rb.FR.getCurrentPosition());
        telemetry.addData("FL Encoder", rb.FL.getCurrentPosition());
        telemetry.addData("BR Encoder", rb.BR.getCurrentPosition());
        telemetry.addData("BL Encoder", rb.BL.getCurrentPosition());
        telemetry.addData("Flywheel Encoder", rb.flywheel.getCurrentPosition());
        telemetry.update();
        odometry.start(rb.FR.getCurrentPosition(), rb.FL.getCurrentPosition(), rb.BL.getCurrentPosition());
        waitForStart();
        while (opModeIsActive()) {
            odometry.update(rb.FR.getCurrentPosition(), rb.FL.getCurrentPosition(), rb.BL.getCurrentPosition());
            telemetry.addData("FR Encoder", rb.FR.getCurrentPosition());
            telemetry.addData("FL Encoder", rb.FL.getCurrentPosition());
            telemetry.addData("BR Encoder", rb.BR.getCurrentPosition());
            telemetry.addData("BL Encoder", rb.BL.getCurrentPosition());
            telemetry.addData("Flywheel Encoder", rb.flywheel.getCurrentPosition());
            telemetry.update();
        }
    }
}
