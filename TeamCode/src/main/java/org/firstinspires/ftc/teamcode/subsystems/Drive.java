package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;

    private double left;
    private double right;

    public Drive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    public void drive(double drive, double turn) {
        left  = drive + turn;
        right = drive - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        frontLeft.setPower(left);
        frontRight.setPower(right);
        backLeft.setPower(left);
        backRight.setPower(right);
    }

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("left", left);
        packet.put("right", right);
        packet.put("heading", getHeading());

        packet.put("flPower", frontLeft.getPower());
        packet.put("frPower", frontRight.getPower());
        packet.put("blPower", backLeft.getPower());
        packet.put("brPower", backRight.getPower());

    }
}
