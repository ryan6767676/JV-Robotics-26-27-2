package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/*
 * TeleOp for driving the mecanum drivetrain with gamepad 1.
 *
 *   Left stick          drive forward/back and strafe left/right
 *   Right stick X       turn
 *   Right bumper (hold) slow mode for lining up
 *   Y / Triangle        switch between field-centric and robot-centric
 *   START / OPTIONS     reset heading (point the robot away from you first)
 */
@TeleOp(name = "Drive TeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private static final double SLOW_SPEED = 0.4;

    private Drivetrain drivetrain;
    private boolean fieldCentric = true;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        telemetry.addLine("Drivetrain ready");
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.startWasPressed()) {
            drivetrain.resetHeading();
        }

        drivetrain.setMaxSpeed(gamepad1.right_bumper ? SLOW_SPEED : 1.0);

        // Pushing the stick forward gives a negative Y value, so flip it.
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (fieldCentric) {
            drivetrain.driveFieldCentric(forward, strafe, turn);
        } else {
            drivetrain.drive(forward, strafe, turn);
        }

        telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
        drivetrain.addTelemetry(telemetry);
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
