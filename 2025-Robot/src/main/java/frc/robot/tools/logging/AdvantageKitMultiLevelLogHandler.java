package frc.robot.tools.logging;

import edu.wpi.first.wpilibj.DataLogManager;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.*;

public class AdvantageKitMultiLevelLogHandler extends Handler {
    private final Map<Level, String> levelLogEntries = new HashMap<>();

    public AdvantageKitMultiLevelLogHandler() {
        // Start the log manager if not already started
        DataLogManager.start();

        // Create a StringLogEntry for each level you want to track
        levelLogEntries.put(Level.SEVERE, "Logs/SEVERE");
        levelLogEntries.put(Level.WARNING, "Logs/WARNING");
        levelLogEntries.put(Level.INFO, "Logs/INFO");
        levelLogEntries.put(Level.CONFIG, "Logs/CONFIG");
        levelLogEntries.put(Level.FINE, "Logs/FINE");
        levelLogEntries.put(Level.FINER, "Logs/FINER");
        levelLogEntries.put(Level.FINEST, "Logs/FINEST");
        // Optional: set default formatter
        setFormatter(new SimpleFormatter());
    }

    @Override
    public void publish(LogRecord record) {
        if (!isLoggable(record))
            return;

        String message = getFormatter().format(record);
        org.littletonrobotics.junction.Logger.recordOutput("Logs/ALL", message);

        if (record.getLevel().intValue() < Level.INFO.intValue()) {
            // Log to the "Debug" level
            org.littletonrobotics.junction.Logger.recordOutput("Logs/DEBUG", message);
        } else {
            org.littletonrobotics.junction.Logger.recordOutput("Logs/IMPORTANT", message);
        }
        // Level-specific
        String entry = levelLogEntries.get(record.getLevel());
        if (entry != null) {
            org.littletonrobotics.junction.Logger.recordOutput(entry, message);
        } else {
            // Optional: handle unknown levels
            org.littletonrobotics.junction.Logger.recordOutput("/Logs/OTHER", message);
        }
    }

    @Override
    public void flush() {
        // Nothing buffered
    }

    @Override
    public void close() {
        // Nothing to close
    }
}
