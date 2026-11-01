package org.firstinspires.ftc.teamcode.pedro;

import com.buildingblockrobotics.expander.BBRDigitalExpander;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro localizer using the mecanum drive motor encoders for distance and the BBR expander gyro
 * for heading. No odometry pods needed, but wheels slip so it drifts more than pods would.
 * Pose is in inches/radians, +X forward, +Y left, CCW positive.
 */
public class DriveEncoderLocalizer implements Localizer {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final BBRDigitalExpander exp;
    private final double forwardTicksPerInch;
    private final double strafeTicksPerInch;

    private int lastFl, lastFr, lastBl, lastBr;
    private double headingOffset;
    private double x, y, heading;
    private long lastNanos;
    private MotionState state = MotionState.zero();

    public DriveEncoderLocalizer(HardwareMap hardwareMap, MecanumConfig drive, String expanderName,
                                 double forwardTicksPerInch, double strafeTicksPerInch) {
        frontLeft = motor(hardwareMap, drive.frontLeftName.get(), drive.frontLeftDirection.get());
        frontRight = motor(hardwareMap, drive.frontRightName.get(), drive.frontRightDirection.get());
        backLeft = motor(hardwareMap, drive.backLeftName.get(), drive.backLeftDirection.get());
        backRight = motor(hardwareMap, drive.backRightName.get(), drive.backRightDirection.get());
        this.forwardTicksPerInch = forwardTicksPerInch;
        this.strafeTicksPerInch = strafeTicksPerInch;

        exp = hardwareMap.get(BBRDigitalExpander.class, expanderName);
        // Robot must be completely still here (~1 s)
        exp.calibrateGyro();
        reset();
    }

    // Directions must match the drivetrain so forward power counts up on every wheel
    private static DcMotor motor(HardwareMap hardwareMap, String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        return motor;
    }

    private double gyroHeading() {
        return Math.toRadians(exp.getHeading());
    }

    @Override
    public void setPose(Pose pose) {
        x = pose.x();
        y = pose.y();
        heading = pose.heading();
        headingOffset = heading - gyroHeading();
        state = MotionState.ofVelocity(pose, Velocity.zero());
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void update() {
        int fl = frontLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int br = backRight.getCurrentPosition();
        int dFl = fl - lastFl, dFr = fr - lastFr, dBl = bl - lastBl, dBr = br - lastBr;
        lastFl = fl; lastFr = fr; lastBl = bl; lastBr = br;

        // Mecanum kinematics in the robot frame, strafing right spins fl/br forward and fr/bl back
        double forward = (dFl + dFr + dBl + dBr) / 4.0 / forwardTicksPerInch;
        double left = -(dFl - dFr - dBl + dBr) / 4.0 / strafeTicksPerInch;

        double newHeading = gyroHeading() + headingOffset;
        double midHeading = heading + angleDiff(newHeading, heading) / 2;
        double dx = forward * Math.cos(midHeading) - left * Math.sin(midHeading);
        double dy = forward * Math.sin(midHeading) + left * Math.cos(midHeading);
        double dHeading = angleDiff(newHeading, heading);

        x += dx;
        y += dy;
        heading = newHeading;

        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;
        Velocity velocity = dt > 0 ? new Velocity(dx / dt, dy / dt, dHeading / dt) : Velocity.zero();
        state = MotionState.ofVelocity(new Pose(x, y, heading), velocity);
    }

    private static double angleDiff(double a, double b) {
        return Math.IEEEremainder(a - b, 2 * Math.PI);
    }

    @Override
    public void reset() {
        lastFl = frontLeft.getCurrentPosition();
        lastFr = frontRight.getCurrentPosition();
        lastBl = backLeft.getCurrentPosition();
        lastBr = backRight.getCurrentPosition();
        lastNanos = System.nanoTime();
        setPose(Pose.zero());
    }
}
