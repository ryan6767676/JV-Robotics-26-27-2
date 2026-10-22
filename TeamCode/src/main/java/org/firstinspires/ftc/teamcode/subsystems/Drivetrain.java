package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Four-motor mecanum drivetrain.
 *
 * Supports robot-centric driving (forward is wherever the robot is facing) and
 * field-centric driving (forward is always away from the driver, using the IMU heading).
 *
 * Robot configuration names on the Driver Hub must match the constants below:
 *   front_left_drive, front_right_drive, back_left_drive, back_right_drive, imu
 *
 * Wheels must form an "X" roller pattern when viewed from above.
 * See https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 */
public class Drivetrain {
    public static final String FRONT_LEFT_NAME = "front_left_drive";
    public static final String FRONT_RIGHT_NAME = "front_right_drive";
    public static final String BACK_LEFT_NAME = "back_left_drive";
    public static final String BACK_RIGHT_NAME = "back_right_drive";
    public static final String IMU_NAME = "imu";

    // How the Control Hub is mounted on the robot. Change these to match your robot,
    // otherwise the heading (and field-centric driving) will be wrong.
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Mecanum wheels strafe slower than they drive forward because of roller slip.
    // Scaling strafe input up a little makes diagonal movement feel straight. Tune on the robot.
    private static final double STRAFE_CORRECTION = 1.1;

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;

    // Scales every wheel power, so 0.5 means half speed. Used for slow/precision mode.
    private double maxSpeed = 1.0;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        // Left motors are mounted mirrored to the right ones, so they spin the opposite way.
        // If the robot drives backward when you push forward, flip all four directions.
        // If it spins in place instead, flip the side that is going the wrong way.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            // BRAKE stops the robot quickly when the sticks are released instead of coasting.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // RUN_WITHOUT_ENCODER still counts encoder ticks, but is safe if the encoder
            // cables are not plugged in (RUN_USING_ENCODER would run the motor at full power).
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));
    }

    /**
     * Drives relative to the robot. All inputs are -1 to 1.
     *
     * @param forward positive drives forward
     * @param strafe  positive strafes right
     * @param turn    positive turns clockwise
     */
    public void drive(double forward, double strafe, double turn) {
        strafe *= STRAFE_CORRECTION;

        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        // If any wheel would go past full power, scale them all down together so the
        // robot still moves in the requested direction instead of the largest values clipping.
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        frontLeft.setPower(maxSpeed * frontLeftPower / max);
        frontRight.setPower(maxSpeed * frontRightPower / max);
        backLeft.setPower(maxSpeed * backLeftPower / max);
        backRight.setPower(maxSpeed * backRightPower / max);
    }

    /**
     * Drives relative to the field: forward always moves away from the driver no matter
     * which way the robot is facing. Call {@link #resetHeading()} with the robot facing
     * away from the driver to set which way is "forward". Inputs are the same as {@link #drive}.
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeading();

        // Rotate the joystick vector by the opposite of the robot's heading.
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double robotStrafe = strafe * cos - forward * sin;
        double robotForward = strafe * sin + forward * cos;

        drive(robotForward, robotStrafe, turn);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /** Sets the speed limit from 0 to 1, e.g. 0.4 for a precision/slow mode. */
    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = Range.clip(maxSpeed, 0.0, 1.0);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    /** Makes the direction the robot is currently facing the new "forward" (heading 0). */
    public void resetHeading() {
        imu.resetYaw();
    }

    /** Robot heading in radians, counter-clockwise positive, from -PI to PI. */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(getHeading()));
        telemetry.addData("Max speed", "%.2f", maxSpeed);
        telemetry.addData("Front L/R power", "%.2f / %.2f", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("Back  L/R power", "%.2f / %.2f", backLeft.getPower(), backRight.getPower());
        telemetry.addData("Front L/R ticks", "%d / %d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.addData("Back  L/R ticks", "%d / %d", backLeft.getCurrentPosition(), backRight.getCurrentPosition());
    }
}
