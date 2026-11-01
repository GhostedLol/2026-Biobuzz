package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.command.Commands.runOnce;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.buildingblockrobotics.expander.BBRDigitalExpander;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final BBRDigitalExpander exp;
    private final boolean gyroCalibrated;

    private static final double TURN_DEADBAND = 0.05;
    // Below this the stick is treated as released so the robot doesn't spin from stick noise
    private static final double DRIVE_DEADBAND = 0.2;
    // Turn power per radian of heading error, raise if it turns too slowly, lower if it wobbles
    private static final double FACE_KP = 1.0;

    public Drive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);


        exp = hardwareMap.get(BBRDigitalExpander.class, "expander");
        // Robot must be completely still here (~1 s); false means it moved and the old bias was kept
        gyroCalibrated = exp.calibrateGyro();
        exp.resetHeading();
    }

    public void drive(double y, double x, double rx) {
        double botHeading = Math.toRadians(getHeading());

        //intake forward
//        if (Math.abs(rx) < TURN_DEADBAND && Math.hypot(x, y) > DRIVE_DEADBAND) {
//            double targetHeading = Math.atan2(-x, y);
//            double error = Math.IEEEremainder(targetHeading - botHeading, 2 * Math.PI);
//            // Positive rx turns clockwise, so the correction is negated
//            rx = -FACE_KP * error;
//        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeft.setPower((rotY + rotX + rx) / denominator);
        backLeft.setPower((rotY - rotX + rx) / denominator);
        frontRight.setPower((rotY - rotX - rx) / denominator);
        backRight.setPower((rotY + rotX - rx) / denominator);

//        No IMU

//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = -(y + x + rx) / denominator;
//        double backLeftPower = -(y - x + rx) / denominator;
//        double frontRightPower = -(y - x - rx) / denominator;
//        double backRightPower = -(y + x - rx) / denominator;
//
//        frontLeft.setPower(frontLeftPower);
//        backLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        backRight.setPower(backRightPower);
    }

    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public Command fl(double power) {
        return runOnce(() -> frontLeft.setPower(power));
    }

    public Command fr(double power) {
        return runOnce(() -> frontRight.setPower(power));
    }

    public Command bl(double power) {
        return runOnce(() -> backLeft.setPower(power));
    }

    public Command br(double power) {
        return runOnce(() -> backRight.setPower(power));
    }

    public Command sequence() {
        return Commands.sequence(
            fl(1), Commands.waitMillis(1500),
            fr(1), Commands.waitMillis(1500),
            bl(1), Commands.waitMillis(1500),
            br(1));
    }

    public void resetYaw() {
        exp.resetHeading();
    }

    public double getHeading() {
        return exp.getHeading();
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("heading", getHeading());
        packet.put("gyroCalibrated", gyroCalibrated);

//        packet.put("pos", exp.getPose());
//        packet.put("heading", exp.getHeading());
        packet.put("flPos", frontLeft.getCurrentPosition());
        packet.put("frPos", frontRight.getCurrentPosition());
        packet.put("blPos", backLeft.getCurrentPosition());
        packet.put("brPos", backRight.getCurrentPosition());

        packet.put("flPower", frontLeft.getPower());
        packet.put("frPower", frontRight.getPower());
        packet.put("blPower", backLeft.getPower());
        packet.put("brPower", backRight.getPower());
    }
}
