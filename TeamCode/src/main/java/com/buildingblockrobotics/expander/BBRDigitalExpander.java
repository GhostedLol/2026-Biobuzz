// BBR Digital Expander — FTC SDK driver.
//
// Implements the BBR Digital Expander I2C protocol contract 1.2:
// identity and protocol verification (refuse on mismatch), the 3-byte
// command block write with idempotency tokens, bounded completion polling,
// single-transaction telemetry reads verified by CRC, write-through
// verification by readback, and fail-loud IMU access. Boards on protocol
// 1.1 still work; the 1.2 additions (telemetry CRC, diagnostics) are then
// skipped or reported as unavailable.
//
// The API has two tiers. The EVERYDAY tier at the top of this file is what
// a first-season programmer needs: one call per question ("how far?",
// "do I see red?", "how many counts?"), no bitmasks, no register knowledge,
// arguments checked with messages that name the fix, and setup helpers that
// persist automatically. The ADVANCED tier below it exposes the full
// register map — raw telemetry blocks, config windows, and runCommand —
// for teams that outgrow the helpers.
//
// Install: copy the com/buildingblockrobotics/expander folder into TeamCode/src/main/java,
// add the device to the robot configuration as "BBR Digital Expander" on an
// I2C bus, then hardwareMap.get(BBRDigitalExpander.class, "expander").
package com.buildingblockrobotics.expander;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(name = "BBR Digital Expander", xmlTag = "BBRDigitalExpander")
public class BBRDigitalExpander extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    /** Thrown for protocol violations, command failures, and misuse. */
    public static class BBRException extends RuntimeException {
        public BBRException(String message) { super(message); }
    }

    /**
     * Thrown when a command the firmware executed reports failure;
     * {@link #resultCode} is the BBRRegMap.ERR_* value for callers that
     * need to react to a specific error rather than the message.
     */
    public static class BBRCommandException extends BBRException {
        public final int resultCode;

        public BBRCommandException(String message, int resultCode) {
            super(message);
            this.resultCode = resultCode;
        }
    }

    /**
     * Thrown when a read never reached the device, as opposed to the device
     * answering something wrong — and only when there is no earlier good
     * reading to fall back on, i.e. the device has never answered at all.
     *
     * <p>OpModes normally never see this. The FTC SDK tears the I2C device
     * down when an OpMode stops, and a read already in flight comes back as
     * the SDK's placeholder rather than failing outright; the driver absorbs
     * that by returning the last good reading (check {@link #isDataFresh})
     * instead of throwing, the same way the goBILDA Pinpoint driver does.
     * Diagnosing a stop-time teardown as broken hardware is worse than
     * useless — it sent teams looking for ESD faults and, on the IMU path,
     * told them to replace a working board.
     *
     * <p>A device that has never answered still throws this. A failure
     * streak that outlives a stop-time teardown (see {@code FAIL_STREAK_*})
     * is logged as a warning and the last good reading keeps being served,
     * unless {@link #setReadFailurePolicy} asks for {@code STOP_OPMODE}.
     */
    public static class BBRTransportException extends BBRException {
        public BBRTransportException(String message) { super(message); }
    }

    /**
     * True if a read looks like it never happened. The SDK substitutes all
     * zeros for a failed transfer, and a bus that was released mid-transaction
     * reads as all 0xFF.
     *
     * <p>Length matters: over a whole register block, all-identical bytes
     * cannot occur naturally — the blocks this guards contain a CRC, a
     * timestamp, or a status byte with reserved bits, none of which are ever
     * 0x00 or 0xFF across the board while the device is alive.
     */
    private static boolean looksLikeFailedRead(byte[] d) {
        if (d == null || d.length < 2) return false;
        byte first = d[0];
        if (first != (byte) 0x00 && first != (byte) 0xFF) return false;
        for (byte b : d) {
            if (b != first) return false;
        }
        return true;
    }

    public enum SensorType {
        EMPTY, COLOR, DISTANCE, UNKNOWN;

        static SensorType from(int v) {
            switch (v) {
                case BBRRegMap.STYPE_COLOR: return COLOR;
                case BBRRegMap.STYPE_DISTANCE: return DISTANCE;
                case BBRRegMap.STYPE_EMPTY: return EMPTY;
                default: return UNKNOWN;
            }
        }
    }

    public enum ChannelMode { QUADRATURE, PULSE_WIDTH }

    /** Which way an encoder channel counts. Same idea, and the same
     *  two names, as {@code DcMotor.Direction}. */
    public enum Direction { FORWARD, REVERSE }

    /**
     * What the driver does when reads keep failing for over half a second
     * (a loose cable, or noise on the bus), as opposed to the moment of
     * failed reads every OpMode stop produces.
     */
    public enum ReadFailurePolicy {
        /** Log a warning and keep serving the last good reading. Check
         *  {@link #isDataFresh()} before acting on a reading. The default. */
        KEEP_RUNNING,
        /** Throw a BBRException, which ends the OpMode. */
        STOP_OPMODE
    }

    /** Why the expander last reset. See {@link #readDiagnostics()}. */
    public enum ResetCause {
        UNKNOWN,
        /** Power applied, or a brown-out: the chip cannot tell them apart. */
        POWER_ON,
        RUN_PIN,
        /** The firmware stopped running and the watchdog restarted it. */
        WATCHDOG,
        /** A deliberate restart, e.g. after a firmware update. */
        SOFTWARE,
        DEBUG;

        static ResetCause from(int v) {
            switch (v) {
                case BBRRegMap.RESET_POWER_ON: return POWER_ON;
                case BBRRegMap.RESET_RUN_PIN: return RUN_PIN;
                case BBRRegMap.RESET_WATCHDOG: return WATCHDOG;
                case BBRRegMap.RESET_SOFTWARE: return SOFTWARE;
                case BBRRegMap.RESET_DEBUG: return DEBUG;
                default: return UNKNOWN;
            }
        }
    }

    /** What drives a digital output. */
    public enum OutputSource {
        DISABLED(BBRRegMap.DOUT_SRC_DISABLED),
        SENSOR_CLASS(BBRRegMap.DOUT_SRC_SENSOR_CLASS),
        ENCODER(BBRRegMap.DOUT_SRC_ENCODER),
        IMU_HEADING(BBRRegMap.DOUT_SRC_IMU_HEADING);

        final int regValue;
        OutputSource(int regValue) { this.regValue = regValue; }
    }

    /** How a digital output behaves once its source matches. */
    public enum OutputMode {
        LEVEL(BBRRegMap.DOUT_MODE_LEVEL),
        LATCHED(BBRRegMap.DOUT_MODE_LATCHED),
        PULSE(BBRRegMap.DOUT_MODE_PULSE);

        final int regValue;
        OutputMode(int regValue) { this.regValue = regValue; }
    }

    // ---------------------------------------------------------------- setup

    private byte token = 0;
    private int capabilities;
    private int protocolMinor;
    private int hwVariant;
    private String firmwareVersion = "unknown";

    // Once the IMU has passed a full requireImu() check, the fitted/absent
    // question is settled for the life of the OpMode; readImu() then trusts
    // the status byte inside the snapshot it already fetched instead of
    // paying a separate status transaction on every loop iteration.
    private boolean imuVerified = false;

    // Short-lived telemetry snapshot backing the everyday getters, so a loop
    // that asks four questions costs one bus transaction, not four.
    private TelemetryBlock cachedTelemetry;
    private long cachedTelemetryNanos;
    private static final long TELEMETRY_CACHE_NS = 10_000_000L;  // 10 ms

    // Lazily-read copy of REG_CHANNEL_MODE for the hot-path mode checks.
    private int channelModeMask = -1;

    // Last-known-good snapshots, returned when a read never reaches the
    // device (Pinpoint-style) so a stop-time teardown, or a flaky bus, does
    // not detonate the OpMode loop. Callers check isDataFresh().
    private TelemetryBlock lastGoodTelemetry;
    private ImuState lastGoodImu;
    private LocalizerState lastGoodLocalizer;
    private boolean lastReadFresh = true;
    private int failStreak = 0;
    private long failStreakStartMs;
    private boolean failStreakLogged;
    private ReadFailurePolicy readFailurePolicy = ReadFailurePolicy.KEEP_RUNNING;
    // A teardown produces at most a handful of calls before the loop exits;
    // only a real fault can keep a streak alive this long AND this often.
    private static final int FAIL_STREAK_MIN_READS = 5;
    private static final long FAIL_STREAK_MIN_MS = 500;

    // Reboot detection: the device's uptime (us since its boot) as last seen
    // in each snapshot type. A drop of more than REBOOT_BACKSTEP_MICROS is a
    // reboot. Works on every protocol version; on 1.2 the reset cause is
    // then read and logged as well.
    private static final int SOURCE_TELEMETRY = 0;
    private static final int SOURCE_IMU = 1;
    private static final int SOURCE_LOCALIZER = 2;
    private final long[] lastDeviceMicros = new long[3];
    private int rebootCount;
    private static final long REBOOT_BACKSTEP_MICROS = 1_000_000L;
    // The clock is a u32 and wraps after ~71.6 min; a drop from near the top is that.
    private static final long CLOCK_WRAP_MICROS = 0x1_0000_0000L;

    public BBRDigitalExpander(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(BBRRegMap.I2C_ADDR_DEFAULT));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize() {
        // Identity block, one read: DEVICE_ID .. STATUS. A cut-short or
        // glitched transaction reads as trailing 0xFF (released bus) or all
        // zeros (SDK placeholder for a failed read) — both transient, so a
        // malformed identity block is retried before concluding the device
        // is wrong. Only a stable, well-formed-but-mismatched answer throws.
        byte[] id = null;
        int deviceId = 0;
        int major = 0;
        for (int attempt = 1; attempt <= 3; attempt++) {
            id = deviceClient.read(BBRRegMap.REG_DEVICE_ID, 9);
            deviceId = id[0] & 0xFF;
            major = id[BBRRegMap.REG_PROTOCOL_MAJOR] & 0xFF;
            if (deviceId == BBRRegMap.DEVICE_ID_VALUE && major != 0xFF) break;
            if (attempt == 3) break;
            try { Thread.sleep(20); } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new BBRException("Interrupted during identity read");
            }
        }
        if (deviceId != BBRRegMap.DEVICE_ID_VALUE) {
            throw new BBRException(String.format(
                    "Not a BBR Digital Expander: DEVICE_ID 0x%02X (expected 0x%02X)",
                    deviceId, BBRRegMap.DEVICE_ID_VALUE));
        }
        protocolMinor = id[BBRRegMap.REG_PROTOCOL_MINOR] & 0xFF;
        if (major != BBRRegMap.PROTOCOL_MAJOR_VALUE) {
            // Registers may have moved or changed meaning: refuse to operate.
            throw new BBRException(String.format(
                    "Protocol major %d not supported (driver expects %d); "
                            + "update the driver or the firmware", major,
                    BBRRegMap.PROTOCOL_MAJOR_VALUE));
        }
        hwVariant = id[BBRRegMap.REG_HW_VARIANT] & 0xFF;
        if (hwVariant != BBRRegMap.VARIANT_BASE && hwVariant != BBRRegMap.VARIANT_ODOMETRY) {
            // Unknown strap: guessing risks driving hardware that isn't there.
            throw new BBRException(String.format(
                    "Unknown HW_VARIANT 0x%02X — firmware older than the board, "
                            + "or a mis-assembled board", hwVariant));
        }
        capabilities = id[BBRRegMap.REG_CAPABILITIES] & 0xFF;
        firmwareVersion = (id[BBRRegMap.REG_FW_VERSION_MAJOR] & 0xFF) + "."
                + (id[BBRRegMap.REG_FW_VERSION_MINOR] & 0xFF) + "."
                + (id[BBRRegMap.REG_FW_VERSION_PATCH] & 0xFF);
        return true;
    }

    @Override
    public Manufacturer getManufacturer() { return Manufacturer.Other; }

    @Override
    public String getDeviceName() { return "BBR Digital Expander"; }

    public int getCapabilities() { return capabilities; }
    public int getProtocolMinor() { return protocolMinor; }

    /** The firmware version on the board, e.g. {@code "1.1.0"}. Worth
     *  showing in telemetry when reporting a problem. */
    public String getFirmwareVersion() { return firmwareVersion; }
    public boolean isOdometryVariant() { return hwVariant == BBRRegMap.VARIANT_ODOMETRY; }

    // ------------------------------------------------- argument validation
    // Every public method checks its indexes up front so a wrong number
    // fails with a message that names the fix, not an array exception or a
    // silent write to the wrong place.

    private static void checkRange(String what, int value, int min, int max) {
        if (value < min || value > max) {
            throw new BBRException(String.format(
                    "%s must be %d-%d, got %d", what, min, max, value));
        }
    }

    private static void checkPort(int port) {
        checkRange("sensorPort", port, 0, BBRRegMap.NUM_SENSOR_PORTS - 1);
    }

    private static void checkChannel(int channel) {
        checkRange("channel", channel, 0, BBRRegMap.NUM_ENCODERS - 1);
    }

    private static void checkOutput(int output) {
        checkRange("output", output, 0, BBRRegMap.NUM_DOUTS - 1);
    }

    private static void checkSlot(int slot) {
        checkRange("classSlot", slot, BBRRegMap.CLASS_SLOT_FIRST, BBRRegMap.CLASS_SLOT_LAST);
    }

    // --------------------------------------------------- failed-read policy

    /**
     * True when the most recent snapshot read actually reached the device.
     * {@code false} means the getters are serving the last good reading —
     * expected for a moment while an OpMode stops; a diagnostic worth
     * showing on a dashboard otherwise.
     */
    public synchronized boolean isDataFresh() { return lastReadFresh; }

    /**
     * What to do when reads keep failing for over half a second. The
     * default, {@code KEEP_RUNNING}, logs one warning per streak and keeps
     * serving the last good reading, so a moment of bus trouble mid-match
     * does not end the OpMode. Anything that acts on readings must then
     * check {@link #isDataFresh()}. {@code STOP_OPMODE} throws instead, for
     * bench testing where failing loudly is more useful than carrying on.
     */
    public synchronized void setReadFailurePolicy(ReadFailurePolicy policy) {
        if (policy == null) throw new BBRException("policy must not be null");
        readFailurePolicy = policy;
    }

    public synchronized ReadFailurePolicy getReadFailurePolicy() { return readFailurePolicy; }

    /**
     * A snapshot read never reached the device: serve the last good reading
     * rather than throwing mid-loop (a stopping OpMode reads this way, and so
     * does a flaky bus). Throws if the device has NEVER answered, as there is
     * then no reading to serve, and on a long streak only under
     * {@code STOP_OPMODE}.
     */
    private synchronized <T> T handleFailedRead(T lastGood, String what) {
        lastReadFresh = false;
        long now = System.currentTimeMillis();
        if (failStreak++ == 0) failStreakStartMs = now;
        if (lastGood == null) {
            throw new BBRTransportException(what
                    + " read did not reach the device and it has never answered — "
                    + "check that it is configured on this bus and powered");
        }
        if (failStreak >= FAIL_STREAK_MIN_READS
                && now - failStreakStartMs >= FAIL_STREAK_MIN_MS) {
            if (readFailurePolicy == ReadFailurePolicy.STOP_OPMODE) {
                throw new BBRException(what
                        + " reads have been failing for over half a second — not a normal "
                        + "OpMode stop; check wiring, cable routing, and power");
            }
            if (!failStreakLogged) {
                failStreakLogged = true;
                RobotLog.ww("BBRDigitalExpander", "%s reads have been failing for over half a"
                        + " second; serving stale data. Check wiring, cable routing, and power",
                        what);
            }
        }
        return lastGood;
    }

    private void noteGoodRead(int source, long deviceMicros) {
        long previous;
        int reboots;
        synchronized (this) {
            previous = lastDeviceMicros[source];
            if (!recordGoodRead(source, deviceMicros)) return;
            reboots = rebootCount;
        }
        // Outside the lock: an extra bus transaction, right after a reboot,
        // must not stall other threads' accessors.
        String cause = "power loss, brownout, or watchdog (hang / ESD)";
        if (protocolMinor >= 2) {
            try {
                cause = "reset cause " + readDiagnostics().resetCause;
            } catch (BBRTransportException e) {
                cause = "reset cause unknown (diagnostics read failed)";
            }
        }
        RobotLog.ww("BBRDigitalExpander", "expander REBOOTED (#%d since the app started):"
                + " uptime now %.2f s, was %.2f s at the last good read; %s",
                reboots, deviceMicros / 1e6, previous / 1e6, cause);
    }

    /** Updates the read-health state; returns whether the device rebooted. */
    private synchronized boolean recordGoodRead(int source, long deviceMicros) {
        // The device's microsecond clock counts from its own boot, so a reading that
        // runs backwards means it rebooted in between: a power loss, a brownout, or
        // the watchdog firing after a hang or an ESD strike. A short reboot recovers
        // inside FAIL_STREAK_MIN_MS and would otherwise leave no trace at all.
        // Each snapshot keeps its own clock: the IMU's only advances on a fusion
        // sample, so comparing across snapshots would read it as going backwards.
        long previous = lastDeviceMicros[source];
        boolean rebooted = previous != 0L
                && deviceMicros + REBOOT_BACKSTEP_MICROS < previous
                && previous < CLOCK_WRAP_MICROS - REBOOT_BACKSTEP_MICROS;
        lastDeviceMicros[source] = deviceMicros;
        if (rebooted) {
            rebootCount++;
            // Keep the other snapshots from reporting the same reboot again.
            for (int i = 0; i < lastDeviceMicros.length; i++) {
                if (i != source) lastDeviceMicros[i] = 0L;
            }
        }
        if (failStreakLogged) {
            RobotLog.ii("BBRDigitalExpander", "reads recovered after %d failures; expander"
                    + " uptime %.2f s (%s)", failStreak, deviceMicros / 1e6,
                    rebooted ? "it rebooted" : "no reboot, so the bus itself was blocked");
        }
        lastReadFresh = true;
        failStreak = 0;
        failStreakLogged = false;
        return rebooted;
    }

    /**
     * How many times the expander has been seen to reboot since the Robot Controller app
     * started (the driver outlives OpModes), detected from its uptime clock running
     * backwards between good reads. Also catches a reboot that happened between OpModes.
     */
    public synchronized int getRebootCount() {
        return rebootCount;
    }

    /**
     * What the expander knows about its own health: why it last reset, and
     * how often its I2C link to the Hub has been disturbed. See
     * {@link #readDiagnostics()}.
     */
    public static class Diagnostics {
        /** Why the expander last reset. */
        public ResetCause resetCause;
        /** Resets since power was applied (0-255). Above 0 with
         *  {@code WATCHDOG} means the firmware hung. */
        public int resetsSincePowerOn;
        /** Times the expander freed a stuck I2C bus since it booted. Should
         *  stay 0; climbing means transfers are being cut off mid-way. */
        public int i2cRecoveryCount;
        /** Times something other than the expander pulled the I2C data
         *  line while it was replying, since it booted. Should stay 0;
         *  climbing usually means electrical noise on the I2C cable. */
        public int i2cArbitrationLostCount;
    }

    /**
     * Read the expander's diagnostics in one bus transaction. Needs firmware
     * 1.1.0 or newer (protocol 1.2). Read it again later and compare the
     * counters: their size matters less than whether they grow while the
     * robot runs. Throws {@link BBRTransportException} if the read never
     * reached the device.
     */
    public Diagnostics readDiagnostics() {
        if (protocolMinor < 2) {
            throw new BBRException("Diagnostics need expander firmware 1.1.0 or newer"
                    + " (this board has " + firmwareVersion + "); update the firmware");
        }
        byte[] d = deviceClient.read(BBRRegMap.SNAP_DIAGNOSTICS_START,
                BBRRegMap.SNAP_DIAGNOSTICS_END - BBRRegMap.SNAP_DIAGNOSTICS_START + 1);
        // RESET_CAUSE is set at every boot, so a uniform block is a transfer
        // that never happened, not a healthy board with zero counters.
        if (looksLikeFailedRead(d)) {
            throw new BBRTransportException("Diagnostics read did not reach the device");
        }
        ByteBuffer b = ByteBuffer.wrap(d).order(ByteOrder.LITTLE_ENDIAN);
        int base = BBRRegMap.SNAP_DIAGNOSTICS_START;
        Diagnostics g = new Diagnostics();
        g.resetCause = ResetCause.from(d[BBRRegMap.REG_RESET_CAUSE - base] & 0xFF);
        g.resetsSincePowerOn = d[BBRRegMap.REG_RESETS_SINCE_POWER_ON - base] & 0xFF;
        g.i2cRecoveryCount = b.getShort(BBRRegMap.REG_I2C_RECOVERY_COUNT - base) & 0xFFFF;
        g.i2cArbitrationLostCount = b.getShort(BBRRegMap.REG_I2C_ARB_LOST_COUNT - base) & 0xFFFF;
        return g;
    }

    // ======================================================================
    // EVERYDAY TIER — one call per question, no bitmasks, no register map.
    // ======================================================================

    // ------------------------------------------------------ encoder reads

    /**
     * Encoder position of {@code channel} (0-3) in counts. On a channel in
     * PULSE_WIDTH mode this is the pulse width instead — use
     * {@link #getPulseWidthUs} there so the name tells the truth.
     */
    public int getEncoderCount(int channel) {
        checkChannel(channel);
        return cachedTelemetry().encoderCounts[channel];
    }

    /** Encoder velocity of {@code channel} (0-3), signed counts per second. */
    public int getEncoderVelocity(int channel) {
        checkChannel(channel);
        return cachedTelemetry().encoderVelocities[channel];
    }

    /**
     * Pulse width on {@code channel} (0-3) in microseconds — the absolute
     * position of a PWM encoder (multi-turn accumulated when wrap tracking
     * is on). {@code 0} means NO SIGNAL: check for it instead of treating
     * it as a position. Throws if the channel is still in QUADRATURE mode.
     */
    public int getPulseWidthUs(int channel) {
        checkChannel(channel);
        if ((channelModeMaskCached() & (1 << channel)) == 0) {
            throw new BBRException(String.format(
                    "Channel %d is in QUADRATURE mode — call setChannelMode(%d, "
                            + "ChannelMode.PULSE_WIDTH) first", channel, channel));
        }
        return cachedTelemetry().encoderCounts[channel];
    }

    /** Zero one encoder channel (0-3). */
    public void resetEncoder(int channel) {
        checkChannel(channel);
        resetEncoders(1 << channel);
    }

    /** Zero all four encoder channels at once. */
    public void resetAllEncoders() {
        resetEncoders((1 << BBRRegMap.NUM_ENCODERS) - 1);
    }

    // ------------------------------------------------------- sensor reads

    /** True while a sensor is detected on {@code port} (0-3). */
    public boolean isSensorConnected(int port) {
        checkPort(port);
        return (cachedTelemetry().presentMask & (1 << port)) != 0;
    }

    /** What kind of sensor is on {@code port} (0-3); EMPTY when none. */
    public SensorType getSensorType(int port) {
        checkPort(port);
        return cachedTelemetry().sensorType[port];
    }

    /**
     * Which taught colour the sensor on {@code port} (0-3) currently sees:
     * a slot number 1-7, or {@code 0} for "nothing I was taught". Fails
     * safe — a dark, saturated, stale, or unplugged sensor reads 0, never
     * a confident wrong answer. Teach colours with {@link #teachColor}.
     */
    public int getColorClass(int port) {
        checkPort(port);
        TelemetryBlock t = cachedTelemetry();
        rejectWrongSensorType(port, t, SensorType.DISTANCE, "getColorClass needs a COLOR sensor");
        return t.sensorClass[port];
    }

    /**
     * True while the colour sensor on {@code port} (0-3) sees the colour
     * taught into {@code classSlot} (1-7, see {@link #teachColor}). Fails
     * safe: an unplugged or unreadable sensor answers {@code false}.
     */
    public boolean seesColor(int port, int classSlot) {
        checkSlot(classSlot);
        return getColorClass(port) == classSlot;
    }

    /**
     * Distance measured by the sensor on {@code port} (0-3), in
     * millimetres. Returns {@code Double.POSITIVE_INFINITY} when nothing is
     * in range (so {@code getDistanceMm(p) < 300} is always safe to write).
     * Throws if the port holds a colour sensor — that is a wiring mix-up,
     * not a reading.
     */
    public double getDistanceMm(int port) {
        checkPort(port);
        TelemetryBlock t = cachedTelemetry();
        rejectWrongSensorType(port, t, SensorType.COLOR, "getDistanceMm needs a DISTANCE sensor");
        if ((t.presentMask & (1 << port)) == 0) return Double.POSITIVE_INFINITY;
        return t.isDistanceValid(port) ? t.distanceMm[port] : Double.POSITIVE_INFINITY;
    }

    /** Throw when the port definitely holds the wrong kind of sensor. */
    private static void rejectWrongSensorType(int port, TelemetryBlock t, SensorType wrong,
                                              String why) {
        if ((t.presentMask & (1 << port)) != 0 && t.sensorType[port] == wrong) {
            throw new BBRException(String.format(
                    "Port %d has a %s sensor — %s; check which port the sensor is plugged into",
                    port, t.sensorType[port], why));
        }
    }

    // ------------------------------------------------ teach and triggers
    // One-call versions of the common configurations, with the tuning values
    // (debounce, hysteresis, signal-rate floor) preset to defaults that work,
    // and the result saved to flash automatically so it survives power-off.
    // Use OutputConfig / DistanceClass / writeColorClass directly when you
    // need full control (those advanced calls do NOT auto-save).

    /**
     * Teach a colour by example: hold the target in front of the sensor on
     * {@code sensorPort} (0-3) and call this — the reading becomes colour
     * number {@code classSlot} (1-7). May take a few seconds, and is saved
     * to flash automatically. Refuses (throws) rather than storing a class
     * that would never — or always — fire: too dark, saturated, or no
     * sensor on the port.
     */
    public void teachColor(int sensorPort, int classSlot) {
        checkPort(sensorPort);
        checkSlot(classSlot);
        selectSensorClass(sensorPort, classSlot);
        runCommand(BBRRegMap.CMD_SENSOR_CAPTURE, 0, BBRRegMap.CMD_SENSOR_CAPTURE_MAX_MS);
        runCommand(BBRRegMap.CMD_CFG_STORE, 0, 10);
        autoSaveIfDirty();
    }

    /**
     * Digital output {@code output} (0-3) goes high while the colour sensor
     * on {@code sensorPort} (0-3) sees the colour taught into
     * {@code classSlot} (1-7, teach it with {@link #teachColor}). Call once
     * during setup; saved to flash automatically.
     */
    public void triggerOnColor(int output, int sensorPort, int classSlot) {
        checkOutput(output);
        checkPort(sensorPort);
        checkSlot(classSlot);
        attachOutputToClass(output, sensorPort, classSlot);
        autoSaveIfDirty();
    }

    /**
     * Digital output {@code output} (0-3) goes high while the distance
     * sensor on {@code sensorPort} (0-3) sees something within
     * {@code maxMm} millimetres. Call once during setup; saved to flash
     * automatically. The range window lives in class slot 7 of that port
     * (kept clear of slots 1-6 so it cannot collide with taught classes).
     */
    public void triggerWhenNear(int output, int sensorPort, int maxMm) {
        checkOutput(output);
        checkPort(sensorPort);
        checkRange("maxMm", maxMm, 1, BBRRegMap.DISTANCE_INVALID - 1);
        DistanceClass cls = new DistanceClass();
        cls.distMax = maxMm;
        cls.minSignalRate = 100;  // reject weak edge-of-range returns
        writeDistanceClass(sensorPort, BBRRegMap.CLASS_SLOT_LAST, cls);
        attachOutputToClass(output, sensorPort, BBRRegMap.CLASS_SLOT_LAST);
        autoSaveIfDirty();
    }

    /**
     * Digital output {@code output} (0-3) goes high while encoder channel
     * {@code channel} (0-3) reads {@code counts} or more (counts on a
     * quadrature channel, microseconds on a pulse-width channel; the
     * comparison is signed). Call once during setup; saved to flash
     * automatically.
     */
    public void triggerWhenEncoderPast(int output, int channel, int counts) {
        checkOutput(output);
        checkChannel(channel);
        OutputConfig c = new OutputConfig();
        c.source = OutputSource.ENCODER;
        c.srcIndex = channel;
        c.threshMin = counts;
        c.threshMax = Integer.MAX_VALUE;
        configureOutput(output, c);
        autoSaveIfDirty();
    }

    /**
     * Digital output {@code output} (0-3) goes high while the robot is
     * facing {@code headingDeg} give or take {@code toleranceDeg} — so
     * {@code triggerWhenFacing(0, 90, 15)} holds output 0 high between 75°
     * and 105°. Headings are the same ones {@link #getHeading} reports:
     * degrees, counter-clockwise positive, wrapping at ±180. The window
     * may straddle the ±180 seam; the board handles that.
     *
     * <p>Call once during setup; saved to flash automatically. Needs the
     * IMU variant of the board, and the output stays low until the gyro
     * has settled after power-on.
     */
    public void triggerWhenFacing(int output, double headingDeg, double toleranceDeg) {
        checkOutput(output);
        if (!isFinite(headingDeg)) {
            throw new BBRException("headingDeg must be a number, got " + headingDeg);
        }
        // Written as a positive test so NaN falls through to the throw.
        if (!(toleranceDeg > 0 && toleranceDeg <= 180)) {
            throw new BBRException(
                    "toleranceDeg must be more than 0 and at most 180, got " + toleranceDeg);
        }
        requireImu();  // a heading trigger on a board with no IMU never fires

        OutputConfig c = new OutputConfig();
        c.source = OutputSource.IMU_HEADING;
        int tolCdeg = (int) Math.round(toleranceDeg * 100.0);
        if (tolCdeg >= 18000) {
            // The whole circle. Written as a non-wrapping full-range window
            // so it reads back as obviously always-true rather than as a
            // wrap window one centidegree wide.
            c.threshMin = BBRRegMap.YAW_MIN_CDEG;
            c.threshMax = BBRRegMap.YAW_MAX_CDEG;
        } else {
            // Fold to (-360, 360) in double space first: a wild heading like
            // 1e9 would otherwise overflow the int cast into a valid-looking
            // window pointing somewhere else entirely.
            int centre = wrapCdeg((int) Math.round((headingDeg % 360.0) * 100.0));
            c.threshMin = wrapCdeg(centre - tolCdeg);
            c.threshMax = wrapCdeg(centre + tolCdeg);
            // threshMin > threshMax is not a mistake here: that is exactly
            // how the firmware is told the window crosses ±180.
        }
        configureOutput(output, c);
        autoSaveIfDirty();
    }

    /** Fold a heading in centidegrees into the firmware's -18000..17999. */
    private static int wrapCdeg(int cdeg) {
        return ((cdeg + 18000) % 36000 + 36000) % 36000 - 18000;
    }

    /** {@code Double.isFinite} without the API 24 floor it carries on Android. */
    private static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    /** Un-latch one digital output (0-3) configured in LATCHED mode. */
    public void clearOutputLatch(int output) {
        checkOutput(output);
        clearOutputLatches(1 << output);
    }

    /** Un-latch all four digital outputs. */
    public void clearAllOutputLatches() {
        clearOutputLatches((1 << BBRRegMap.NUM_DOUTS) - 1);
    }

    /**
     * Save any changed configuration to flash, riding out the firmware's
     * one-save-per-second rate limit. Used by the setup helpers so "it
     * worked yesterday, gone today" cannot happen to a first-year team.
     */
    private void autoSaveIfDirty() {
        if (!isConfigDirty()) return;
        try {
            saveConfigToFlash();
        } catch (BBRCommandException e) {
            if (e.resultCode != BBRRegMap.ERR_FLASH_RATE_LIMITED) throw e;
            try {
                Thread.sleep(BBRRegMap.FLASH_SAVE_RATE_LIMIT_MS);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                throw new BBRException("Interrupted while saving configuration");
            }
            saveConfigToFlash();
        }
    }

    // -------------------------------------------------- heading and pose

    /**
     * Heading in degrees, [-180, 180), positive = turning left. Zero it at
     * match start with {@link #resetHeading()}. Odometry variant only —
     * throws a specific message on a base board rather than reporting a
     * plausible heading of 0.00 forever.
     */
    public double getHeading() { return readImu().yawDeg; }

    /** Zero yaw at the current orientation. Call at match start. */
    public void resetHeading() {
        requireImu();
        runCommand(BBRRegMap.CMD_IMU_RESET_HEADING, 0, 10);
    }

    // ---------------------------------------------- mounting / orientation

    /**
     * Which robot axis points UP, so the IMU integrates yaw about the true
     * vertical when the expander is NOT mounted flat (e.g. on its edge). Pass
     * one of the {@code BBRRegMap.AXIS_*} values; flat is {@code AXIS_POS_Z}.
     * Written to RAM — call {@link #saveConfigToFlash} to persist — and
     * re-calibrate the IMU (e.g. {@link #resetLocalizerAndCalibrateImu}) after
     * changing it so the new axis is used.
     */
    public void setImuAxisUp(int axisUp) {
        checkRange("axisUp", axisUp, BBRRegMap.AXIS_POS_X, BBRRegMap.AXIS_NEG_Z);
        writeVerified(BBRRegMap.REG_IMU_AXIS_UP, (byte) axisUp, "IMU_AXIS_UP");
    }

    /** Current IMU axis-up setting (a {@code BBRRegMap.AXIS_*} value). */
    public int getImuAxisUp() {
        return deviceClient.read(BBRRegMap.REG_IMU_AXIS_UP, 1)[0] & 0xFF;
    }

    /**
     * Make {@code channel} (0-3) count UP when the thing it measures moves the
     * way you consider forward. {@code REVERSE} flips both the count and the
     * velocity for that channel.
     *
     * <p>This is the fix for an encoder that counts backwards, and it belongs
     * here rather than in your own code: the odometry localizer reads the
     * channels straight off the board, so negating a number in an OpMode
     * never reaches it. Push the robot and check — driving forward should
     * raise the X pod's count, strafing left should raise the Y pod's.
     *
     * <p>Saved to flash automatically, so the direction survives a power
     * cycle — a channel that counts backwards after every reboot is a bug
     * nobody traces back to a missing save call. Setting the same direction
     * repeatedly costs nothing: the firmware compares the payload it would
     * write against what is already stored and skips the write.
     *
     * <p>Like the other setup helpers, run this once rather than in a loop.
     */
    public void setEncoderDirection(int channel, Direction direction) {
        checkChannel(channel);
        int bit = 1 << channel;
        int mask = getEncoderInvertMask();
        setEncoderInvertMask(direction == Direction.REVERSE ? (mask | bit) : (mask & ~bit));
        autoSaveIfDirty();
    }

    /** Which way {@code channel} (0-3) currently counts. */
    public Direction getEncoderDirection(int channel) {
        checkChannel(channel);
        return (getEncoderInvertMask() & (1 << channel)) != 0
                ? Direction.REVERSE : Direction.FORWARD;
    }

    /**
     * How long, in milliseconds (1-255), the expander measures over to work
     * out each encoder velocity. Applies to all four channels. Longer is
     * smoother and finer but slower to react; shorter reacts faster but is
     * noisier. The default is 25 ms, where one count of change is
     * 40 counts/s.
     *
     * <p>Saved to flash automatically, like {@link #setEncoderDirection}.
     * Run it once rather than in a loop. This does not change the
     * odometry localizer, which has its own window in its parameters.
     */
    public void setEncoderVelocityWindowMs(int ms) {
        checkRange("ms", ms, 1, 255);
        writeVerified(BBRRegMap.REG_VEL_INTERVAL_MS, (byte) ms, "VEL_INTERVAL_MS");
        autoSaveIfDirty();
    }

    /** The encoder velocity window in milliseconds; see
     *  {@link #setEncoderVelocityWindowMs}. */
    public int getEncoderVelocityWindowMs() {
        return deviceClient.read(BBRRegMap.REG_VEL_INTERVAL_MS, 1)[0] & 0xFF;
    }

    /**
     * All four channels at once as a bitmask: bit n set inverts channel n.
     * {@link #setEncoderDirection} says the same thing one channel at a time
     * and reads better; this is here for setting several in one transaction.
     * Written to RAM — call {@link #saveConfigToFlash} to persist.
     */
    public void setEncoderInvertMask(int mask) {
        checkRange("mask", mask, 0, (1 << BBRRegMap.NUM_ENCODERS) - 1);
        writeVerified(BBRRegMap.REG_ENC_INVERT_MASK, (byte) mask, "ENC_INVERT_MASK");
    }

    /** Current per-channel encoder invert mask (bit n = channel n inverted). */
    public int getEncoderInvertMask() {
        return deviceClient.read(BBRRegMap.REG_ENC_INVERT_MASK, 1)[0] & 0xFF;
    }

    /**
     * Re-estimate gyro bias (~1 s). Keep the robot completely still. Returns
     * {@code true} with a fresh bias, or {@code false} if the robot moved:
     * the previous bias is kept and the IMU carries on, so a bump never ends
     * the OpMode. Call again while still if heading drift matters.
     */
    public boolean calibrateGyro() {
        requireImu();
        return runCalibration(BBRRegMap.CMD_CALIBRATE_GYRO, BBRRegMap.CMD_CALIBRATE_GYRO_MAX_MS,
                "calibrateGyro");
    }

    /**
     * A calibration that saw the robot move is not a failure worth an
     * exception: the firmware keeps the previous bias (and LOC_RESET still
     * resets the pose and starts the localizer), so the robot is exactly as
     * usable as before. Throwing here ended OpModes mid-match on a nudge.
     */
    private boolean runCalibration(int opcode, int maxMs, String what) {
        try {
            runCommand(opcode, 0, maxMs);
            return true;
        } catch (BBRCommandException e) {
            if (e.resultCode != BBRRegMap.ERR_IMU_NOT_STATIONARY) throw e;
            RobotLog.ww("BBRDigitalExpander", "%s: the robot moved during gyro calibration;"
                    + " keeping the previous bias", what);
            return false;
        }
    }

    /** Current pose. Throws unless the localizer is RUNNING. */
    public Pose2D getPose() { return readLocalizer().pose(); }

    /**
     * Block until the localizer reaches RUNNING after
     * {@link #resetLocalizerAndCalibrateImu()} — keep the robot still while
     * it waits. Returns {@code true} when ready, {@code false} on timeout;
     * throws immediately on states that waiting cannot fix (no IMU, reset
     * never called).
     */
    public boolean waitForLocalizerReady(long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (true) {
            LocalizerState s = readLocalizerRaw();
            switch (s.status) {
                case RUNNING:
                    return true;
                case FAULT_NO_IMU:
                    requireImu();  // produces the specific base-variant/fault message
                    throw new BBRException("Localizer faulted: IMU unavailable");
                case NOT_READY:
                    throw new BBRException(
                            "Localizer not started — call resetLocalizerAndCalibrateImu() first");
                default:
                    break;  // WARMING_UP_IMU / CALIBRATING_IMU: keep waiting
            }
            if (System.currentTimeMillis() > deadline) return false;
            try { Thread.sleep(50); } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new BBRException("Interrupted waiting for the localizer");
            }
        }
    }

    // ======================================================================
    // ADVANCED TIER — the full register map. Nothing below auto-saves;
    // persist explicitly with saveConfigToFlash().
    // ======================================================================

    // ------------------------------------------------------------- commands

    private static String resultName(int code) {
        // The hints on the common codes are aimed at a driver-station screen:
        // the student reading them has not seen the register map.
        switch (code) {
            case BBRRegMap.OK: return "OK";
            case BBRRegMap.ERR_UNKNOWN_COMMAND: return "ERR_UNKNOWN_COMMAND";
            case BBRRegMap.ERR_BAD_ARG: return "ERR_BAD_ARG";
            case BBRRegMap.ERR_NOT_SUPPORTED: return "ERR_NOT_SUPPORTED";
            case BBRRegMap.ERR_TOKEN_REUSE: return "ERR_TOKEN_REUSE";
            case BBRRegMap.ERR_FLASH_WRITE: return "ERR_FLASH_WRITE";
            case BBRRegMap.ERR_FLASH_VERIFY: return "ERR_FLASH_VERIFY";
            case BBRRegMap.ERR_FLASH_RATE_LIMITED:
                return "ERR_FLASH_RATE_LIMITED (saving too often — wait a second between saves)";
            case BBRRegMap.ERR_SENSOR_ABSENT:
                return "ERR_SENSOR_ABSENT (no sensor plugged into that port)";
            case BBRRegMap.ERR_SENSOR_NAK: return "ERR_SENSOR_NAK";
            case BBRRegMap.ERR_SENSOR_TIMEOUT: return "ERR_SENSOR_TIMEOUT";
            case BBRRegMap.ERR_SENSOR_SATURATED:
                return "ERR_SENSOR_SATURATED (too much light on the sensor)";
            case BBRRegMap.ERR_INSUFFICIENT_SIGNAL:
                return "ERR_INSUFFICIENT_SIGNAL (too dark or target too far — move it closer)";
            case BBRRegMap.ERR_CFG_INVALID_SLOT: return "ERR_CFG_INVALID_SLOT";
            case BBRRegMap.ERR_IMU_NOT_STATIONARY:
                return "ERR_IMU_NOT_STATIONARY (the robot moved — keep it completely still)";
            case BBRRegMap.ERR_LOC_BAD_PARAMS:
                return "ERR_LOC_BAD_PARAMS (localizer parameters missing or out of range — "
                        + "call setLocalizerParams first)";
            default: return String.format("error 0x%02X", code);
        }
    }

    /**
     * Issue a command per the driver sequence in the register map: one
     * 3-byte block write (arg, token, opcode), then poll COMMAND_STATUS
     * bounded by the command's maximum duration. Throws on error results.
     * Returns the result code (always OK unless a command legitimately
     * returns data via its result in a future protocol minor).
     */
    public synchronized int runCommand(int opcode, int arg, int maxMs) {
        // Any command can move counts, classes, or modes: drop the caches.
        cachedTelemetry = null;
        channelModeMask = -1;
        token++;
        if (token == BBRRegMap.TOKEN_NONE) token = 1;  // skip 0x00
        byte[] block = {(byte) arg, token, (byte) opcode};
        deviceClient.write(BBRRegMap.REG_COMMAND_ARG, block, I2cWaitControl.WRITTEN);

        long deadline = System.currentTimeMillis() + maxMs + 50;
        while (true) {
            byte[] out = deviceClient.read(BBRRegMap.REG_COMMAND_STATUS, 3);
            int status = out[0] & 0xFF;
            int result = out[1] & 0xFF;
            if (status == BBRRegMap.CMDSTAT_DONE) return result;
            if (status == BBRRegMap.CMDSTAT_ERROR) {
                throw new BBRCommandException(String.format("Command 0x%02X failed: %s",
                        opcode, resultName(result)), result);
            }
            if (System.currentTimeMillis() > deadline) {
                // An async command overrunning its max indicates a firmware
                // or hardware problem a retry will not fix: surface a fault.
                throw new BBRException(String.format(
                        "Command 0x%02X still busy after %d ms", opcode, maxMs));
            }
            try { Thread.sleep(5); } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new BBRException("Interrupted waiting for command");
            }
        }
    }

    /** Zero the encoder channels whose bits are set; prefer resetEncoder(). */
    public void resetEncoders(int channelMask) {
        checkRange("channelMask", channelMask, 0, (1 << BBRRegMap.NUM_ENCODERS) - 1);
        runCommand(BBRRegMap.CMD_RESET_ENCODERS, channelMask, 10);
    }

    // ------------------------------------------------------- channel modes

    /** Set all four channel modes at once; bit n set = channel n PULSE_WIDTH. */
    public synchronized void setChannelModes(int pwmMask) {
        checkRange("pwmMask", pwmMask, 0, (1 << BBRRegMap.NUM_ENCODERS) - 1);
        writeVerified(BBRRegMap.REG_CHANNEL_MODE, (byte) pwmMask, "CHANNEL_MODE");
        channelModeMask = pwmMask;
    }

    public void setChannelMode(int channel, ChannelMode mode) {
        checkChannel(channel);
        int mask = deviceClient.read(BBRRegMap.REG_CHANNEL_MODE, 1)[0] & 0xFF;
        int bit = 1 << channel;
        setChannelModes(mode == ChannelMode.PULSE_WIDTH ? (mask | bit) : (mask & ~bit));
    }

    public synchronized ChannelMode getChannelMode(int channel) {
        checkChannel(channel);
        int mask = deviceClient.read(BBRRegMap.REG_CHANNEL_MODE, 1)[0] & 0xFF;
        channelModeMask = mask;
        return (mask & (1 << channel)) != 0 ? ChannelMode.PULSE_WIDTH : ChannelMode.QUADRATURE;
    }

    private synchronized int channelModeMaskCached() {
        if (channelModeMask < 0) {
            channelModeMask = deviceClient.read(BBRRegMap.REG_CHANNEL_MODE, 1)[0] & 0xFF;
        }
        return channelModeMask;
    }

    /** Multi-turn wrap tracking for one pulse-width channel (0-3). */
    public void setPwmWrapEnabled(int channel, boolean enabled) {
        checkChannel(channel);
        int mask = deviceClient.read(BBRRegMap.REG_PWM_WRAP_MASK, 1)[0] & 0xFF;
        int bit = 1 << channel;
        setPwmWrapMask(enabled ? (mask | bit) : (mask & ~bit));
    }

    /** Enable multi-turn wrap tracking per channel; needs valid PWM params. */
    public void setPwmWrapMask(int mask) {
        checkRange("mask", mask, 0, (1 << BBRRegMap.NUM_ENCODERS) - 1);
        writeVerified(BBRRegMap.REG_PWM_WRAP_MASK, (byte) mask, "PWM_WRAP_MASK");
    }

    /**
     * Calibrate one pulse-width channel: the pulse widths at the start and
     * end of a revolution (e.g. 1 and 1024 for a REV Through Bore absolute
     * output). Preserves the other channels' calibration. Persist with
     * {@link #saveConfigToFlash()}.
     */
    public void setPwmChannelParams(int channel, int minUs, int maxUs) {
        checkChannel(channel);
        checkRange("minUs", minUs, 1, BBRRegMap.PWM_WIDTH_MAX_US - 1);
        checkRange("maxUs", maxUs, minUs + 1, BBRRegMap.PWM_WIDTH_MAX_US);
        runCommand(BBRRegMap.CMD_PWM_PARAMS_LOAD, 0, 10);
        byte[] w = deviceClient.read(BBRRegMap.CFGWIN_BASE, 20);
        ByteBuffer b = ByteBuffer.wrap(w).order(ByteOrder.LITTLE_ENDIAN);
        b.putShort(BBRRegMap.PWMWIN_PWM0_MIN_US - BBRRegMap.CFGWIN_BASE + 4 * channel,
                (short) minUs);
        b.putShort(BBRRegMap.PWMWIN_PWM0_MAX_US - BBRRegMap.CFGWIN_BASE + 4 * channel,
                (short) maxUs);
        deviceClient.write(BBRRegMap.CFGWIN_BASE, w, I2cWaitControl.WRITTEN);
        runCommand(BBRRegMap.CMD_PWM_PARAMS_STORE, 0, 10);
    }

    /** {@code [minUs, maxUs]} for one channel; {@code [0, 0]} = unconfigured. */
    public int[] getPwmChannelParams(int channel) {
        checkChannel(channel);
        runCommand(BBRRegMap.CMD_PWM_PARAMS_LOAD, 0, 10);
        byte[] w = deviceClient.read(BBRRegMap.CFGWIN_BASE, 20);
        ByteBuffer b = ByteBuffer.wrap(w).order(ByteOrder.LITTLE_ENDIAN);
        return new int[] {
                b.getShort(BBRRegMap.PWMWIN_PWM0_MIN_US - BBRRegMap.CFGWIN_BASE + 4 * channel)
                        & 0xFFFF,
                b.getShort(BBRRegMap.PWMWIN_PWM0_MAX_US - BBRRegMap.CFGWIN_BASE + 4 * channel)
                        & 0xFFFF};
    }

    public void clearFaults() { runCommand(BBRRegMap.CMD_CLEAR_FAULTS, 0, 10); }

    /** Persist all configuration to flash. Setup-time only (up to 500 ms). */
    public void saveConfigToFlash() {
        runCommand(BBRRegMap.CMD_CFG_SAVE_FLASH, 0, BBRRegMap.CMD_CFG_SAVE_FLASH_MAX_MS);
    }

    public void loadFactoryDefaults() { runCommand(BBRRegMap.CMD_CFG_LOAD_DEFAULTS, 0, 10); }

    // ------------------------------------------------------------ telemetry

    /**
     * One 96-byte snapshot of the full telemetry block, self-consistent.
     * On a channel in PULSE_WIDTH mode, {@code encoderCounts} is the pulse
     * width in microseconds (wrap-accumulated when enabled; {@code 0} means
     * NO SIGNAL) and {@code encoderVelocities} is µs per second.
     */
    public static class TelemetryBlock {
        public final int[] encoderCounts = new int[4];
        public final int[] encoderVelocities = new int[4];  // signed counts/s (or µs/s)
        public final int[] sensorClass = new int[4];        // 0 = no match
        public final int[] classConfidence = new int[4];    // 0-255
        public final int[] sensorStatus = new int[4];       // BBRRegMap.SSTAT_* bits
        public final SensorType[] sensorType = new SensorType[4];
        public int presentMask;
        public long timestampMicros;
        // Colour ports: rescaled 16-bit channels; distance ports: overlay.
        public final int[][] rawColor = new int[4][5];  // r,g,b,ir,prox
        public final int[] distanceMm = new int[4];     // 0xFFFF = invalid
        public final int[] signalRate = new int[4];
        public final int[] ambientRate = new int[4];
        public final int[] rangeStatus = new int[4];

        public boolean isDistanceValid(int port) {
            return distanceMm[port] != BBRRegMap.DISTANCE_INVALID;
        }
    }

    /**
     * Read the whole telemetry block in one bus transaction. If the read
     * never reaches the device (stop-time teardown), the previous good
     * block is returned instead — see {@link #isDataFresh}.
     *
     * <p>On protocol 1.2 and later the block is checked against
     * TELEMETRY_CRC16 and re-read on mismatch, the same way the localizer
     * block is: a corrupted encoder velocity would otherwise arrive looking
     * valid. After three corrupt reads the last good block is served.
     */
    public TelemetryBlock readTelemetry() {
        byte[] d;
        for (int attempt = 1; ; attempt++) {
            d = deviceClient.read(BBRRegMap.SNAP_TELEMETRY_START, 96);
            // The timestamp is nonzero from the first millisecond of boot, so a
            // uniform block can only be a transfer that never happened.
            if (looksLikeFailedRead(d)) {
                return handleFailedRead(lastGoodTelemetry, "Telemetry");
            }
            if (protocolMinor < 2 || telemetryCrcMatches(d)) break;
            if (attempt == 3) {
                return handleFailedRead(lastGoodTelemetry, "Telemetry");
            }
        }
        ByteBuffer b = ByteBuffer.wrap(d).order(ByteOrder.LITTLE_ENDIAN);
        TelemetryBlock t = new TelemetryBlock();
        int base = BBRRegMap.SNAP_TELEMETRY_START;
        for (int i = 0; i < 4; i++) {
            t.encoderCounts[i] = b.getInt(BBRRegMap.REG_ENC0_COUNT - base + 4 * i);
            t.encoderVelocities[i] = b.getInt(BBRRegMap.REG_ENC0_VELOCITY - base + 4 * i);
            t.sensorClass[i] = d[BBRRegMap.REG_S0_CLASS - base + i] & 0xFF;
            t.classConfidence[i] = d[BBRRegMap.REG_CLASS_CONFIDENCE - base + i] & 0xFF;
            t.sensorStatus[i] = d[BBRRegMap.REG_SENSOR_STATUS - base + i] & 0xFF;
            t.sensorType[i] = SensorType.from(d[BBRRegMap.REG_SENSOR_TYPE - base + i] & 0xFF);

            int slot = BBRRegMap.RAW_BASE - base + BBRRegMap.RAW_STRIDE * i;
            if (t.sensorType[i] == SensorType.DISTANCE) {
                t.distanceMm[i] = b.getShort(slot + BBRRegMap.RAW_DISTANCE_MM) & 0xFFFF;
                t.signalRate[i] = b.getShort(slot + BBRRegMap.RAW_SIGNAL_RATE) & 0xFFFF;
                t.ambientRate[i] = b.getShort(slot + BBRRegMap.RAW_AMBIENT_RATE) & 0xFFFF;
                t.rangeStatus[i] = d[slot + BBRRegMap.RAW_RANGE_STATUS] & 0xFF;
            } else {
                for (int c = 0; c < 5; c++) {
                    t.rawColor[i][c] = b.getShort(slot + 2 * c) & 0xFFFF;
                }
            }
        }
        t.presentMask = d[BBRRegMap.REG_SENSOR_PRESENT_MASK - base] & 0xFF;
        t.timestampMicros = b.getInt(BBRRegMap.REG_TELEMETRY_TIMESTAMP - base) & 0xFFFFFFFFL;
        noteGoodRead(SOURCE_TELEMETRY, t.timestampMicros);
        lastGoodTelemetry = t;
        return t;
    }

    /**
     * Snapshot backing the everyday getters, refreshed when older than
     * 10 ms — a loop that asks several one-value questions still costs one
     * bus transaction per pass, and every answer within a pass is
     * self-consistent.
     */
    private synchronized TelemetryBlock cachedTelemetry() {
        long now = System.nanoTime();
        if (cachedTelemetry == null || now - cachedTelemetryNanos > TELEMETRY_CACHE_NS) {
            cachedTelemetry = readTelemetry();
            cachedTelemetryNanos = now;
        }
        return cachedTelemetry;
    }

    /**
     * Live encoder input levels (ENC_PIN_STATE, protocol 1.1): bit 2n =
     * channel n's A line, bit 2n+1 = its B line. Idle inputs read 1
     * (pull-ups). A bring-up diagnostic: a bit that never toggles while the
     * shaft turns is the dead line. Point sample per 1 ms firmware pass —
     * poll fast, turn slowly. Reads 0x00 on protocol 1.0 firmware.
     */
    public int readEncoderPinState() {
        return deviceClient.read(BBRRegMap.REG_ENC_PIN_STATE, 1)[0] & 0xFF;
    }

    /** Device STATUS bitfield (BBRRegMap.STATUS_* bits). */
    public int readStatus() {
        return deviceClient.read(BBRRegMap.REG_STATUS, 1)[0] & 0xFF;
    }

    /** True when configuration changed in RAM has not been saved to flash. */
    public boolean isConfigDirty() {
        return (readStatus() & BBRRegMap.STATUS_CFG_DIRTY) != 0;
    }

    // ------------------------------------------------------- configuration

    private void writeVerified(int reg, byte value, String name) {
        // The bus layer never NAKs: an invalid write is silently ignored and
        // the previous value preserved. Write, read back, compare.
        deviceClient.write(reg, new byte[] {value}, I2cWaitControl.WRITTEN);
        byte back = deviceClient.read(reg, 1)[0];
        if (back != value) {
            throw new BBRException(String.format(
                    "%s rejected value %d (still %d) — out of range?", name, value & 0xFF,
                    back & 0xFF));
        }
    }

    private void selectSensorClass(int port, int slot) {
        writeVerified(BBRRegMap.REG_CFG_SENSOR_SELECT, (byte) port, "CFG_SENSOR_SELECT");
        writeVerified(BBRRegMap.REG_CFG_CLASS_SELECT, (byte) slot, "CFG_CLASS_SELECT");
    }

    /** A colour class window on normalized chromaticity (0-1000 per channel). */
    public static class ColorClass {
        public int rMin, rMax, gMin, gMax, bMin, bMax;
        public boolean proximityGate = false;
        public int proxMin = 0, proxMax = BBRRegMap.PROX_MAX_VALUE;
    }

    public void writeColorClass(int port, int slot, ColorClass c) {
        checkPort(port);
        checkSlot(slot);
        checkChromaWindow("r", c.rMin, c.rMax);
        checkChromaWindow("g", c.gMin, c.gMax);
        checkChromaWindow("b", c.bMin, c.bMax);
        checkRange("proxMin", c.proxMin, 0, BBRRegMap.PROX_MAX_VALUE);
        checkRange("proxMax", c.proxMax, c.proxMin, BBRRegMap.PROX_MAX_VALUE);
        selectSensorClass(port, slot);
        ByteBuffer w = ByteBuffer.allocate(20).order(ByteOrder.LITTLE_ENDIAN);
        w.put((byte) 1).put((byte) (c.proximityGate ? BBRRegMap.CLASSF_PROX_GATE : 0));
        w.putShort((short) c.rMin).putShort((short) c.rMax);
        w.putShort((short) c.gMin).putShort((short) c.gMax);
        w.putShort((short) c.bMin).putShort((short) c.bMax);
        w.putShort((short) c.proxMin).putShort((short) c.proxMax);
        deviceClient.write(BBRRegMap.CFGWIN_BASE, w.array(), I2cWaitControl.WRITTEN);
        runCommand(BBRRegMap.CMD_CFG_STORE, 0, 10);
    }

    private static void checkChromaWindow(String channel, int min, int max) {
        checkRange(channel + "Min", min, 0, BBRRegMap.CHROMA_SCALE);
        checkRange(channel + "Max", max, min, BBRRegMap.CHROMA_SCALE);
    }

    /** A distance class window in millimetres, inclusive bounds. */
    public static class DistanceClass {
        public int distMin = 0, distMax;
        public int hysteresisMm = 10;
        public int minSignalRate = 0;
    }

    public void writeDistanceClass(int port, int slot, DistanceClass c) {
        checkPort(port);
        checkSlot(slot);
        checkRange("distMin", c.distMin, 0, BBRRegMap.DISTANCE_INVALID - 1);
        checkRange("distMax", c.distMax, c.distMin, BBRRegMap.DISTANCE_INVALID - 1);
        selectSensorClass(port, slot);
        ByteBuffer w = ByteBuffer.allocate(20).order(ByteOrder.LITTLE_ENDIAN);
        w.put((byte) 1).put((byte) 1);  // enabled; require valid RANGE_STATUS
        w.putShort((short) c.distMin).putShort((short) c.distMax);
        w.putShort((short) c.hysteresisMm).putShort((short) c.minSignalRate);
        deviceClient.write(BBRRegMap.CFGWIN_BASE, w.array(), I2cWaitControl.WRITTEN);
        runCommand(BBRRegMap.CMD_CFG_STORE, 0, 10);
    }

    /** Digital output configuration. See the "Digital outputs and triggers" guide. */
    public static class OutputConfig {
        public OutputSource source = OutputSource.DISABLED;
        public int srcIndex = 0;    // sensor port or encoder channel
        public int classIndex = 0;  // for SENSOR_CLASS sources
        public boolean activeLow = false;
        public OutputMode mode = OutputMode.LEVEL;
        public int debounceAssert = 2, debounceRelease = 2;
        public int pulseMs = 0;
        public int threshMin = 0, threshMax = 0;  // encoder counts or yaw x100
    }

    public void configureOutput(int output, OutputConfig c) {
        checkOutput(output);
        switch (c.source) {
            case SENSOR_CLASS:
                checkPort(c.srcIndex);
                checkSlot(c.classIndex);
                break;
            case ENCODER:
                checkChannel(c.srcIndex);
                break;
            case IMU_HEADING:
                // threshMin > threshMax is legal — it means the window wraps
                // through +/-180 — so only the ends are checked, not order.
                checkRange("threshMin", c.threshMin,
                        BBRRegMap.YAW_MIN_CDEG, BBRRegMap.YAW_MAX_CDEG);
                checkRange("threshMax", c.threshMax,
                        BBRRegMap.YAW_MIN_CDEG, BBRRegMap.YAW_MAX_CDEG);
                break;
            default:
                break;
        }
        checkRange("debounceAssert", c.debounceAssert, 0, 255);
        checkRange("debounceRelease", c.debounceRelease, 0, 255);
        checkRange("pulseMs", c.pulseMs, 0, 255);
        writeVerified(BBRRegMap.REG_DOUT_SELECT, (byte) output, "DOUT_SELECT");
        ByteBuffer w = ByteBuffer.allocate(16).order(ByteOrder.LITTLE_ENDIAN);
        w.put((byte) c.source.regValue).put((byte) c.srcIndex).put((byte) c.classIndex);
        w.put((byte) ((c.activeLow ? 1 : 0) | (c.mode.regValue << 1)));
        w.put((byte) c.debounceAssert).put((byte) c.debounceRelease);
        w.put((byte) c.pulseMs).put((byte) 0);
        w.putInt(c.threshMin).putInt(c.threshMax);
        deviceClient.write(BBRRegMap.DOUTWIN_BASE, w.array(), I2cWaitControl.WRITTEN);
        runCommand(BBRRegMap.CMD_DOUT_STORE, 0, 10);
    }

    /** Un-latch the outputs whose bits are set; prefer clearOutputLatch(). */
    public void clearOutputLatches(int mask) {
        checkRange("mask", mask, 0, (1 << BBRRegMap.NUM_DOUTS) - 1);
        runCommand(BBRRegMap.CMD_DOUT_CLEAR_LATCH, mask, 10);
    }

    /** Level-mode, debounced output following a (port, class) match. */
    private void attachOutputToClass(int output, int sensorPort, int classSlot) {
        OutputConfig c = new OutputConfig();
        c.source = OutputSource.SENSOR_CLASS;
        c.srcIndex = sensorPort;
        c.classIndex = classSlot;
        configureOutput(output, c);
    }

    // ------------------------------------------------------------------ IMU

    public static class ImuState {
        public double quatW, quatX, quatY, quatZ;
        public double yawDeg, pitchDeg, rollDeg;      // robot frame, md conventions
        public double[] gyroDps = new double[3];      // robot frame
        public double[] linearAccel = new double[3];  // body frame, m/s^2, gravity removed
        public int calibGrade;                        // 0-3
        public int imuStatus;                         // BBRRegMap.ISTAT_* bits
        public long timestampMicros;

        /**
         * True while the turn rate exceeds what the gyro can measure —
         * usually a hard collision. The heading is suspect from the first
         * moment this reads true.
         */
        public boolean isGyroSaturated() {
            return (imuStatus & BBRRegMap.ISTAT_SATURATED) != 0;
        }

        /** True once the robot has been still long enough to trust the bias. */
        public boolean isBiasValid() {
            return (imuStatus & BBRRegMap.ISTAT_BIAS_VALID) != 0;
        }

        /** True while the fusion output (yaw/pitch/roll/quat) is usable. */
        public boolean isFusionValid() {
            return (imuStatus & BBRRegMap.ISTAT_FUSION_VALID) != 0;
        }
    }

    /**
     * All-zero IMU registers would read as a heading of exactly 0.00 that
     * never changes — the worst possible failure, because it looks like
     * working software. So this throws, loudly and specifically, whenever
     * the IMU cannot be trusted (md, "When the IMU is unavailable").
     */
    private void requireImu() {
        boolean fitted = (capabilities & BBRRegMap.CAP_IMU) != 0;
        int istat = deviceClient.read(BBRRegMap.REG_IMU_STATUS, 1)[0] & 0xFF;
        boolean responding = (istat & BBRRegMap.ISTAT_PRESENT) != 0;

        // A single status byte cannot tell "no IMU" from "the read never
        // happened" — both read 0x00, and a released bus reads 0xFF (which
        // would even set PRESENT and let garbage through). So before accusing
        // the hardware, confirm the BOARD is still answering by re-reading
        // DEVICE_ID, whose value is known and non-zero. Costs one extra
        // transaction, and only ever on the failure path.
        if (!responding || istat == 0xFF) {
            int id = deviceClient.read(BBRRegMap.REG_DEVICE_ID, 1)[0] & 0xFF;
            if (id != BBRRegMap.DEVICE_ID_VALUE) {
                throw new BBRTransportException(
                        "IMU read did not reach the device (DEVICE_ID read back as "
                                + String.format("0x%02X", id) + ") — normal if the "
                                + "OpMode is stopping; otherwise check that the device "
                                + "is still configured and powered");
            }
        }
        if (istat == 0xFF) {
            throw new BBRTransportException(
                    "IMU status read as 0xFF on a board that is otherwise answering — "
                            + "treat as a failed transfer, not a fitted IMU");
        }
        if (!fitted && !responding) {
            throw new BBRException(
                    "This board has no IMU (base variant) — odometry needs the odometry variant");
        }
        if (fitted && !responding) {
            throw new BBRException(
                    "IMU fitted but not responding — hardware fault, not a wrong purchase; "
                            + "request a replacement");
        }
        if (!fitted) {
            throw new BBRException("IMU responding on a board not strapped for one — "
                    + "build inconsistency, check the variant straps");
        }
        imuVerified = true;
    }

    public ImuState readImu() {
        // The full status pre-check runs until it passes once; after that the
        // fitted/absent question cannot change, and the status byte inside
        // the snapshot below still guards every read — so the hot path
        // (getHeading() every loop) is one transaction, not two.
        if (!imuVerified) requireImu();
        byte[] d = deviceClient.read(BBRRegMap.SNAP_IMU_START, 32);
        // An all-zero block would decode to a heading of exactly 0.00 that
        // never changes — the failure this class exists to prevent. Serve
        // the last real reading instead; the fail-streak policy still turns
        // a persistent failure into a loud error.
        if (looksLikeFailedRead(d)) {
            return handleFailedRead(lastGoodImu, "IMU");
        }
        ByteBuffer b = ByteBuffer.wrap(d).order(ByteOrder.LITTLE_ENDIAN);
        int base = BBRRegMap.SNAP_IMU_START;
        ImuState s = new ImuState();
        s.imuStatus = d[BBRRegMap.REG_IMU_STATUS - base] & 0xFF;
        if ((s.imuStatus & BBRRegMap.ISTAT_PRESENT) == 0) {
            // The IMU vanished after verification: a transient dropout serves
            // stale data like any failed read, and a persistent one escalates
            // through the streak policy into a hardware-fault error.
            return handleFailedRead(lastGoodImu, "IMU");
        }
        s.quatW = b.getShort(BBRRegMap.REG_QUAT_W - base) / (double) BBRRegMap.QUAT_SCALE;
        s.quatX = b.getShort(BBRRegMap.REG_QUAT_X - base) / (double) BBRRegMap.QUAT_SCALE;
        s.quatY = b.getShort(BBRRegMap.REG_QUAT_Y - base) / (double) BBRRegMap.QUAT_SCALE;
        s.quatZ = b.getShort(BBRRegMap.REG_QUAT_Z - base) / (double) BBRRegMap.QUAT_SCALE;
        s.yawDeg = b.getShort(BBRRegMap.REG_YAW - base) / 100.0;
        s.pitchDeg = b.getShort(BBRRegMap.REG_PITCH - base) / 100.0;
        s.rollDeg = b.getShort(BBRRegMap.REG_ROLL - base) / 100.0;
        for (int i = 0; i < 3; i++) {
            s.gyroDps[i] = b.getShort(BBRRegMap.REG_GYRO_X - base + 2 * i)
                    / (double) BBRRegMap.GYRO_SCALE_LSB_PER_DPS;
            s.linearAccel[i] = b.getShort(BBRRegMap.REG_ACCEL_X - base + 2 * i)
                    / (double) BBRRegMap.ACCEL_SCALE_LSB_PER_MPS2;
        }
        s.calibGrade = d[BBRRegMap.REG_IMU_CALIB - base] & 0xFF;
        s.timestampMicros = b.getInt(BBRRegMap.REG_IMU_TIMESTAMP - base) & 0xFFFFFFFFL;
        noteGoodRead(SOURCE_IMU, s.timestampMicros);
        lastGoodImu = s;
        return s;
    }

    // ------------------------------------------------------------ localizer

    /** Robot pose: mm and radians, +X forward, +Y left, CCW positive. */
    public static class Pose2D {
        public double xMm, yMm, headingRad;

        public Pose2D() {}

        public Pose2D(double xMm, double yMm, double headingRad) {
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingRad = headingRad;
        }

        /** The same pose with the heading given in degrees. */
        public static Pose2D fromDegrees(double xMm, double yMm, double headingDeg) {
            return new Pose2D(xMm, yMm, Math.toRadians(headingDeg));
        }

        /** Heading in degrees, for telemetry and degree-based math. */
        public double headingDeg() { return Math.toDegrees(headingRad); }
    }

    public enum LocalizerStatus {
        NOT_READY, WARMING_UP_IMU, CALIBRATING_IMU, RUNNING, FAULT_NO_IMU, UNKNOWN;

        static LocalizerStatus from(int v) {
            switch (v) {
                case BBRRegMap.LOC_NOT_READY: return NOT_READY;
                case BBRRegMap.LOC_WARMING_UP_IMU: return WARMING_UP_IMU;
                case BBRRegMap.LOC_CALIBRATING_IMU: return CALIBRATING_IMU;
                case BBRRegMap.LOC_RUNNING: return RUNNING;
                case BBRRegMap.LOC_FAULT_NO_IMU: return FAULT_NO_IMU;
                default: return UNKNOWN;
            }
        }
    }

    /** One 20-byte snapshot of the localizer block, self-consistent. */
    public static class LocalizerState {
        public LocalizerStatus status;
        public int flags;  // BBRRegMap.LOCF_* bits
        public double xMm, yMm, headingRad;
        public double velXMmPerSec, velYMmPerSec;  // world frame
        public double headingVelRadPerSec;
        public long timestampMicros;

        public Pose2D pose() { return new Pose2D(xMm, yMm, headingRad); }

        /**
         * True if the gyro has clipped at any point since the last
         * localizer reset (latched) — usually a hard collision. The
         * heading, and everything integrated from it, is suspect from
         * that moment on: relocalize off an AprilTag or a wall touch.
         */
        public boolean hasGyroEverSaturated() {
            return (flags & BBRRegMap.LOCF_GYRO_SATURATED_EVER) != 0;
        }

        /** True while the pose sits outside ±32.7 m and reads clamped. */
        public boolean isPoseClipped() {
            return (flags & BBRRegMap.LOCF_POSE_CLIPPED) != 0;
        }

        /**
         * True if a pod's encoder channel was disturbed while the localizer
         * was running — its invert mask, channel mode or PWM parameters
         * changed. The firmware swallows one tick delta rather than
         * integrating the jump, so the pose is missing that tick and this
         * latches to say so. It is a "something changed under me" report,
         * not a standing misconfiguration: set direction and mode once at
         * setup, before starting the localizer.
         */
        public boolean hasPortConflict() {
            return (flags & BBRRegMap.LOCF_PORT_CONFLICT) != 0;
        }
    }

    /**
     * Same fail-loud contract as the IMU: a localizer stuck at (0, 0, 0)
     * looks exactly like working software, so any state in which the pose
     * cannot be trusted throws instead of returning zeros.
     */
    private void requireLocalizer(LocalizerStatus status) {
        switch (status) {
            case RUNNING:
                return;
            case FAULT_NO_IMU:
                requireImu();  // produces the specific base-variant/fault message
                throw new BBRException("Localizer faulted: IMU unavailable");
            case NOT_READY:
                throw new BBRException(
                        "Localizer not started — call resetLocalizerAndCalibrateImu() first");
            case WARMING_UP_IMU:
            case CALIBRATING_IMU:
                throw new BBRException("Localizer still " + status
                        + " — call waitForLocalizerReady() before reading the pose");
            default:
                throw new BBRException("Localizer in unknown state — firmware newer than driver?");
        }
    }

    /**
     * PROFIBUS CRC16 (poly 0x1DCF, init 0xFFFF, no reflection, xor-out
     * 0xFFFF) — the same algorithm the OctoQuad uses, so ported verification
     * code works unchanged.
     */
    private static int crc16Profibus(byte[] data, int len) {
        int crc = 0xFFFF;
        for (int i = 0; i < len; i++) {
            crc ^= (data[i] & 0xFF) << 8;
            for (int b = 0; b < 8; b++) {
                crc = ((crc & 0x8000) != 0) ? ((crc << 1) ^ 0x1DCF) : (crc << 1);
                crc &= 0xFFFF;
            }
        }
        return crc ^ 0xFFFF;
    }

    /**
     * TELEMETRY_CRC16 sits inside the block it covers, so it is computed
     * over all 96 bytes with its own two bytes taken as zero.
     */
    private static boolean telemetryCrcMatches(byte[] d) {
        int off = BBRRegMap.REG_TELEMETRY_CRC16 - BBRRegMap.SNAP_TELEMETRY_START;
        int stored = (d[off] & 0xFF) | ((d[off + 1] & 0xFF) << 8);
        byte[] copy = d.clone();
        copy[off] = 0;
        copy[off + 1] = 0;
        return crc16Profibus(copy, copy.length) == stored;
    }

    /**
     * Raw localizer snapshot, any state; check {@code status} yourself.
     * Verifies LOC_CRC16 against the block — I2C acknowledges bytes without
     * verifying them, and a noise-flipped bit would otherwise arrive looking
     * like a valid pose. Re-reads on mismatch; after three corrupt reads
     * serves the last good pose rather than returning a suspect one, leaving
     * the read-failure policy to deal with corruption that persists.
     */
    public LocalizerState readLocalizerRaw() {
        byte[] d;
        int crcOff = BBRRegMap.REG_LOC_CRC16 - BBRRegMap.SNAP_LOCALIZER_START;
        for (int attempt = 1; ; attempt++) {
            d = deviceClient.read(BBRRegMap.SNAP_LOCALIZER_START, 22);
            // A block that is entirely 0x00 or 0xFF never left the device, so
            // the CRC below would "fail" and blame the wiring for what is
            // usually just an OpMode stopping. Serve the last good state
            // instead; the fail-streak policy catches persistent failure.
            if (looksLikeFailedRead(d)) {
                return handleFailedRead(lastGoodLocalizer, "Localizer");
            }
            int stored = (d[crcOff] & 0xFF) | ((d[crcOff + 1] & 0xFF) << 8);
            if (crc16Profibus(d, 20) == stored) break;
            // Three corrupt reads in a row is a bad moment on the bus, not
            // proof the bus is gone: a burst of noise mid-match would
            // otherwise crash the OpMode outright. Treat it as a failed read
            // — serve the last good pose and let the fail-streak policy
            // escalate if the corruption actually persists.
            if (attempt == 3) {
                return handleFailedRead(lastGoodLocalizer, "Localizer");
            }
        }
        ByteBuffer b = ByteBuffer.wrap(d).order(ByteOrder.LITTLE_ENDIAN);
        int base = BBRRegMap.SNAP_LOCALIZER_START;
        LocalizerState s = new LocalizerState();
        s.status = LocalizerStatus.from(d[BBRRegMap.REG_LOC_STATUS - base] & 0xFF);
        s.flags = d[BBRRegMap.REG_LOC_FLAGS - base] & 0xFF;
        s.velXMmPerSec = b.getShort(BBRRegMap.REG_LOC_VEL_X - base);
        s.velYMmPerSec = b.getShort(BBRRegMap.REG_LOC_VEL_Y - base);
        s.headingVelRadPerSec = b.getShort(BBRRegMap.REG_LOC_VEL_H - base)
                / (double) BBRRegMap.LOC_HEADING_VEL_SCALE;
        s.xMm = b.getShort(BBRRegMap.REG_LOC_X - base);
        s.yMm = b.getShort(BBRRegMap.REG_LOC_Y - base);
        s.headingRad = b.getShort(BBRRegMap.REG_LOC_H - base)
                / (double) BBRRegMap.LOC_HEADING_SCALE;
        s.timestampMicros = b.getInt(BBRRegMap.REG_LOC_TIMESTAMP - base) & 0xFFFFFFFFL;
        noteGoodRead(SOURCE_LOCALIZER, s.timestampMicros);
        lastGoodLocalizer = s;
        return s;
    }

    /** Pose + velocities in one bus transaction. Throws unless RUNNING. */
    public LocalizerState readLocalizer() {
        LocalizerState s = readLocalizerRaw();
        requireLocalizer(s.status);
        return s;
    }

    /**
     * Localizer parameters. Stored parameters take effect only on the next
     * {@link #resetLocalizerAndCalibrateImu()}.
     */
    public static class LocalizerParams {
        /** Pod resolution, ticks per mm of travel. Measure it: push the
         *  robot a couple of meters along one axis and divide. */
        public double ticksPerMmX, ticksPerMmY;
        /** Tracking-point offset from robot center, mm, robot frame. */
        public double tcpOffsetXMm = 0, tcpOffsetYMm = 0;
        /** Gyro scale correction (spin-10-turns calibration), ~1.0. */
        public double imuScalar = 1.0;
        /** Encoder channels of the X and Y pods, 0-3. */
        public int portX, portY;
        /** Velocity averaging window, ms. */
        public int velocityIntervalMs = BBRRegMap.DEFAULT_LOC_VEL_INTERVAL_MS;
    }

    public void setLocalizerParams(LocalizerParams p) {
        checkChannel(p.portX);
        checkChannel(p.portY);
        if (p.portX == p.portY) {
            throw new BBRException(String.format(
                    "portX and portY are both %d — the X and Y pods need different "
                            + "encoder channels", p.portX));
        }
        int tpmX = (int) Math.round(p.ticksPerMmX * 1000.0);  // wire unit: ticks/meter
        int tpmY = (int) Math.round(p.ticksPerMmY * 1000.0);
        checkRange("ticksPerMmX (x1000)", tpmX, BBRRegMap.LOC_TPM_MIN, BBRRegMap.LOC_TPM_MAX);
        checkRange("ticksPerMmY (x1000)", tpmY, BBRRegMap.LOC_TPM_MIN, BBRRegMap.LOC_TPM_MAX);
        int scalar = (int) Math.round(p.imuScalar * BBRRegMap.LOC_IMU_SCALAR_SCALE);
        checkRange("imuScalar (x10000)", scalar,
                BBRRegMap.LOC_IMU_SCALAR_MIN, BBRRegMap.LOC_IMU_SCALAR_MAX);
        checkRange("velocityIntervalMs", p.velocityIntervalMs, 1, 255);
        ByteBuffer w = ByteBuffer.allocate(20).order(ByteOrder.LITTLE_ENDIAN);
        w.putInt(tpmX);
        w.putInt(tpmY);
        w.putShort((short) Math.round(p.tcpOffsetXMm));
        w.putShort((short) Math.round(p.tcpOffsetYMm));
        w.putShort((short) scalar);
        w.put((byte) p.portX).put((byte) p.portY);
        w.put((byte) p.velocityIntervalMs);
        deviceClient.write(BBRRegMap.CFGWIN_BASE, w.array(), I2cWaitControl.WRITTEN);
        runCommand(BBRRegMap.CMD_LOC_PARAMS_STORE, 0, 10);
    }

    public LocalizerParams getLocalizerParams() {
        runCommand(BBRRegMap.CMD_LOC_PARAMS_LOAD, 0, 10);
        byte[] d = deviceClient.read(BBRRegMap.CFGWIN_BASE, 20);
        ByteBuffer b = ByteBuffer.wrap(d).order(ByteOrder.LITTLE_ENDIAN);
        int base = BBRRegMap.CFGWIN_BASE;
        LocalizerParams p = new LocalizerParams();
        p.ticksPerMmX = b.getInt(BBRRegMap.LOCWIN_TPM_X - base) / 1000.0;
        p.ticksPerMmY = b.getInt(BBRRegMap.LOCWIN_TPM_Y - base) / 1000.0;
        p.tcpOffsetXMm = b.getShort(BBRRegMap.LOCWIN_TCP_X - base);
        p.tcpOffsetYMm = b.getShort(BBRRegMap.LOCWIN_TCP_Y - base);
        p.imuScalar = (b.getShort(BBRRegMap.LOCWIN_IMU_SCALAR - base) & 0xFFFF)
                / (double) BBRRegMap.LOC_IMU_SCALAR_SCALE;
        p.portX = d[BBRRegMap.LOCWIN_PORT_X - base] & 0xFF;
        p.portY = d[BBRRegMap.LOCWIN_PORT_Y - base] & 0xFF;
        p.velocityIntervalMs = d[BBRRegMap.LOCWIN_VEL_INTERVAL_MS - base] & 0xFF;
        return p;
    }

    /**
     * Latch stored parameters, zero the pose, and calibrate the gyro (~1 s).
     * Keep the robot completely still. Call at match start, then
     * {@link #waitForLocalizerReady} before reading the pose. Persist
     * parameters and the new bias with {@link #saveConfigToFlash()} at
     * setup time.
     *
     * <p>Returns {@code true} with a fresh gyro bias, or {@code false} if the
     * robot moved during calibration. Either way the pose is reset and the
     * localizer starts; {@code false} only means the previous bias is kept.
     * Bad localizer parameters still throw.
     */
    public boolean resetLocalizerAndCalibrateImu() {
        requireImu();
        return runCalibration(BBRRegMap.CMD_LOC_RESET, BBRRegMap.CMD_LOC_RESET_MAX_MS,
                "resetLocalizerAndCalibrateImu");
    }

    /**
     * Teleport the reported pose — the relocalization primitive after an
     * AprilTag sighting or a wall touch. The heading is normalized into
     * [-pi, pi) automatically; build degree-based poses with
     * {@link Pose2D#fromDegrees}.
     */
    public void setPose(Pose2D pose) {
        int x = (int) Math.round(pose.xMm);
        int y = (int) Math.round(pose.yMm);
        checkRange("xMm", x, Short.MIN_VALUE, Short.MAX_VALUE);
        checkRange("yMm", y, Short.MIN_VALUE, Short.MAX_VALUE);
        // IEEEremainder lands in [-pi, pi]; fold the closed top end down so
        // exactly +pi (e.g. Math.PI from student code) is accepted as -pi.
        double h = Math.IEEEremainder(pose.headingRad, 2.0 * Math.PI);
        if (h >= Math.PI) h -= 2.0 * Math.PI;
        ByteBuffer w = ByteBuffer.allocate(6).order(ByteOrder.LITTLE_ENDIAN);
        w.putShort((short) x);
        w.putShort((short) y);
        w.putShort((short) Math.round(h * BBRRegMap.LOC_HEADING_SCALE));
        // Written immediately before the command on purpose: the window is
        // shared between overlays and LOC_SET_POSE reads what is there now.
        deviceClient.write(BBRRegMap.CFGWIN_BASE, w.array(), I2cWaitControl.WRITTEN);
        runCommand(BBRRegMap.CMD_LOC_SET_POSE, 0, 10);
    }
}
