package frc.robot.util;

import frc.robot.Constants.Config;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Map;

/**
 * <p> Huge credit to FRC Team 581: Littleton Robotics. </p>
 *
 * A logger that is almost fully compatible with the AdvantageKit `Logger` class. It uses WPILib's
 * DataLog to log the data to a file, and streams the data using NetworkTables 4.
 *
 * @see https://github.com/team581/frc-2023-charged-up/pull/64
 */
public class Logger {
  private static final String OUTPUT_LOG_DIR = "/RealOutputs/";
  private static final String METADATA_LOG_DIR = "/RealMetadata/";

  private static final DataLog log = DataLogManager.getLog();

  private static NetworkTable outputTable;
  private static NetworkTable metadataTable;

  /** If the logger has been started. Metadata is no longer accepted, and logged values are. */
  private static boolean running = false;

  private static final ArrayList<String> indexedKeys = new ArrayList<>();

  private static final Map<Integer, BooleanLogEntry> booleanLogs = new HashMap<>();
  private static final Map<Integer, DoubleLogEntry> doubleLogs = new HashMap<>();
  private static final Map<Integer, FloatLogEntry> floatLogs = new HashMap<>();
  private static final Map<Integer, IntegerLogEntry> integerLogs = new HashMap<>();
  private static final Map<Integer, StringLogEntry> stringLogs = new HashMap<>();
  private static final Map<Integer, BooleanArrayLogEntry> booleanArrayLogs = new HashMap<>();
  private static final Map<Integer, DoubleArrayLogEntry> doubleArrayLogs = new HashMap<>();
  private static final Map<Integer, FloatArrayLogEntry> floatArrayLogs = new HashMap<>();
  private static final Map<Integer, IntegerArrayLogEntry> integerArrayLogs = new HashMap<>();
  private static final Map<Integer, StringArrayLogEntry> stringArrayLogs = new HashMap<>();

  private static final Map<Integer, BooleanPublisher> booleanPublishers = new HashMap<>();
  private static final Map<Integer, DoublePublisher> doublePublishers = new HashMap<>();
  private static final Map<Integer, FloatPublisher> floatPublishers = new HashMap<>();
  private static final Map<Integer, IntegerPublisher> integerPublishers = new HashMap<>();
  private static final Map<Integer, StringPublisher> stringPublishers = new HashMap<>();
  private static final Map<Integer, BooleanArrayPublisher> booleanArrayPublishers = new HashMap<>();
  private static final Map<Integer, DoubleArrayPublisher> doubleArrayPublishers = new HashMap<>();
  private static final Map<Integer, FloatArrayPublisher> floatArrayPublishers = new HashMap<>();
  private static final Map<Integer, IntegerArrayPublisher> integerArrayPublishers = new HashMap<>();
  private static final Map<Integer, StringArrayPublisher> stringArrayPublishers = new HashMap<>();

  // There should be no instances of the class
  private Logger() {}

  // Start log manager
  static {
    if (RobotBase.isSimulation()) {
      // Log to project directory in simulation
      DataLogManager.start();
    } else {
      DataLogManager.start(Config.DATA_LOG_DIR);
    }
    // Disable logging NT values to separate logger and networktables
    DataLogManager.logNetworkTables(false);
    // Log inputs from DriverStation
    DriverStation.startDataLog(log);
  }

  /** Set up the logger. Metadata is no longer accepted, and logged values are. */
  public static synchronized void start() {
    // Should only run once
    if (running) {
      return;
    }
    running = true;

    // Set up network tables for data stream
    if (Config.NTStream) {
      var table = NetworkTableInstance.getDefault().getTable("/AdvantageKit");
      outputTable = table.getSubTable("RealOutputs");
      metadataTable = table.getSubTable("RealMetadata");
    }
  }

  public void recordOutput(String key, boolean value) {
    if (running) {
      int id = getId(key);
      booleanLogs
        .computeIfAbsent(id, k -> new BooleanLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve or create log entry
        .append(value); // Record value to log file

      if (Config.NTStream) {
        booleanPublishers
          .computeIfAbsent(id, k -> outputTable.getBooleanTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve or create publisher
          .set(value); // Record value to network tables
      }
    }
  }

  public static void recordMetadata(String key, String value) {
    if (!running) {
      // Record metadata in log file
      var entry = new StringLogEntry(log, METADATA_LOG_DIR + key);
      entry.append(value);
      entry.finish(); // Close unused entry

      // Record metadata to network tables
      if (Config.NTStream) {
        var publisher = metadataTable.getStringTopic(key).publish();
        publisher.set(value);
        publisher.close(); // Close unused publisher
      }
    }
  }

  /** @return id of given key (newly created if necessary) to be used in log and entry maps */
  private static synchronized int getId(String key) {
    // Add key if new
    if (indexedKeys.contains(key)) {
      indexedKeys.add(key);
    }
    return indexedKeys.indexOf(key);
  }
}