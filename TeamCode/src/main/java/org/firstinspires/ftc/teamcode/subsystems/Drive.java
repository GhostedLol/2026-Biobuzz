package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.command.Commands.runOnce;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.buildingblockrobotics.expander.BBRDigitalExpander;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

public class Drive extends SubsystemBase {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final BBRDigitalExpander exp;

    private final boolean gyroCalibrated;

    public Drive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);


        exp = hardwareMap.get(BBRDigitalExpander.class, "expander");
        gyroCalibrated = exp.calibrateGyro();
        exp.resetHeading();
    }

    public void drive(double y, double x, double rx) {
        double botHeading = Math.toRadians(getHeading());

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeft.setPower((rotY + rotX + rx) / denominator);
        backLeft.setPower((rotY - rotX + rx) / denominator);
        frontRight.setPower((rotY - rotX - rx) / denominator);
        backRight.setPower((rotY + rotX - rx) / denominator);
    }

    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void resetYaw() {
        exp.resetHeading();
    }

    public double getHeading() {
        return exp.getHeading();
    }

    public BBRDigitalExpander.Pose2D getPos() {
        return exp.getPose();
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("heading", getHeading());
        packet.put("gyroCalibrated", gyroCalibrated);

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
