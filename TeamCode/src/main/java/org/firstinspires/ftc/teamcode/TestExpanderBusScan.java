package org.firstinspires.ftc.teamcode;

import com.buildingblockrobotics.expander.BBRRegMap;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Wiring diagnostic for the BBR Digital Expander. Reads DEVICE_ID at every address the board can
 * be jumpered to, on every I2C bus of every hub, without going through the robot configuration.
 * A line marked FOUND is where the board really is; 0x00 everywhere means it never answered.
 */
@TeleOp(name = "Expander Bus Scan", group = "Diagnostics")
public class ExpanderBusScan extends LinearOpMode {
    private static final int BUSES_PER_HUB = 4;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        boolean found = false;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            String hub = "Hub (address " + module.getModuleAddress() + ")";
            for (int bus = 0; bus < BUSES_PER_HUB && !isStopRequested(); bus++) {
                LynxI2cDeviceSynch client = LynxFirmwareVersionManager.createLynxI2cDeviceSynch(
                        AppUtil.getDefContext(), module, bus);
                StringBuilder line = new StringBuilder();
                for (int addr = BBRRegMap.I2C_ADDR_MIN; addr <= BBRRegMap.I2C_ADDR_MAX; addr++) {
                    client.setI2cAddress(I2cAddr.create7bit(addr));
                    int id = client.read(BBRRegMap.REG_DEVICE_ID, 1)[0] & 0xFF;
                    line.append(String.format("0x%02X=0x%02X ", addr, id));
                    if (id == BBRRegMap.DEVICE_ID_VALUE) {
                        found = true;
                        line.append("<-FOUND ");
                    }
                }
                client.close();
                telemetry.addData(hub + " bus " + bus, line.toString());
                telemetry.update();
            }
        }

        telemetry.addLine(found
                ? "Expander FOUND - configure it on the hub, bus and address marked above."
                : "Expander did NOT answer on any bus (expected ID 0xB2) - wiring, power or firmware.");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            idle();
        }
    }
}
