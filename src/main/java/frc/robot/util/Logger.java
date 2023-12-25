package frc.robot.util;

import frc.robot.Constants.Config;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.StringArrayPublisher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * <p> Huge credit to FRC Team 581: Littleton Robotics for the basis of this implementation. </p>
 *
 * <p> For consistency, logging calls should be done by subsystems and the main robot class (after
 * control logic), and be under a directory corresponding to the same. </p>
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

    // Set up network tables for data stream
    if (Config.NTStream) {
      var table = NetworkTableInstance.getDefault().getTable("/AdvantageKit");
      outputTable = table.getSubTable("RealOutputs");
      metadataTable = table.getSubTable("RealMetadata");
    }
  }

  /**
   * Log a message to the "messages" entry. The message is also printed to standard output.
   *
   * @param msg message
   */
  public static void log(String msg) {
    DataLogManager.log(msg);
  }

  /** Start the logger. Metadata is no longer accepted, and logged values are. */
  public static synchronized void start() {
    // Should only run once
    if (running) {
      return;
    }
    running = true;
  }

  public static void recordOutput(String key, boolean value) {
    if (running) {
      int id = getId(key);
      booleanLogs
        .computeIfAbsent(id, k -> new BooleanLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(value); // Record value to log file

      if (Config.NTStream) {
        booleanPublishers
          .computeIfAbsent(id, k -> outputTable.getBooleanTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(value); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, double value) {
    if (running) {
      int id = getId(key);
      doubleLogs
        .computeIfAbsent(id, k -> new DoubleLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(value); // Record value to log file

      if (Config.NTStream) {
        doublePublishers
          .computeIfAbsent(id, k -> outputTable.getDoubleTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(value); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, float value) {
    if (running) {
      int id = getId(key);
      floatLogs
        .computeIfAbsent(id, k -> new FloatLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(value); // Record value to log file

      if (Config.NTStream) {
        floatPublishers
          .computeIfAbsent(id, k -> outputTable.getFloatTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(value); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, int value) {
    if (running) {
      int id = getId(key);
      integerLogs
        .computeIfAbsent(id, k -> new IntegerLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(value); // Record value to log file

      if (Config.NTStream) {
        integerPublishers
          .computeIfAbsent(id, k -> outputTable.getIntegerTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(value); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, String value) {
    if (running) {
      int id = getId(key);
      stringLogs
        .computeIfAbsent(id, k -> new StringLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(value); // Record value to log file

      if (Config.NTStream) {
        stringPublishers
          .computeIfAbsent(id, k -> outputTable.getStringTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(value); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, boolean[] values) {
    if (running) {
      int id = getId(key);
      booleanArrayLogs
        .computeIfAbsent(id, k -> new BooleanArrayLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(values); // Record value to log file

      if (Config.NTStream) {
        booleanArrayPublishers
          .computeIfAbsent(id, k -> outputTable.getBooleanArrayTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(values); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, double[] values) {
    if (running) {
      int id = getId(key);
      doubleArrayLogs
        .computeIfAbsent(id, k -> new DoubleArrayLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(values); // Record value to log file

      if (Config.NTStream) {
        doubleArrayPublishers
          .computeIfAbsent(id, k -> outputTable.getDoubleArrayTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(values); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, float[] values) {
    if (running) {
      int id = getId(key);
      floatArrayLogs
        .computeIfAbsent(id, k -> new FloatArrayLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(values); // Record value to log file

      if (Config.NTStream) {
        floatArrayPublishers
          .computeIfAbsent(id, k -> outputTable.getFloatArrayTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(values); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, long[] values) {
    if (running) {
      int id = getId(key);
      integerArrayLogs
        .computeIfAbsent(id, k -> new IntegerArrayLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(values); // Record value to log file

      if (Config.NTStream) {
        integerArrayPublishers
          .computeIfAbsent(id, k -> outputTable.getIntegerArrayTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(values); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, int[] values) {
    // Transform int[] into long[]
    long[] longs = new long[values.length];
    for (int i = 0; i < values.length; i++) {
      longs[i] = values[i];
    }

    recordOutput(key, longs);
  }

  public static void recordOutput(String key, String[] values) {
    if (running) {
      int id = getId(key);
      stringArrayLogs
        .computeIfAbsent(id, k -> new StringArrayLogEntry(log, OUTPUT_LOG_DIR + key)) // Retrieve/create log entry
        .append(values); // Record value to log file

      if (Config.NTStream) {
        stringArrayPublishers
          .computeIfAbsent(id, k -> outputTable.getStringArrayTopic(key).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
          .set(values); // Record value to network tables
      }
    }
  }

  public static void recordOutput(String key, Pose2d... value) {
    // Map Pose2d[] into double[]
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }

    recordOutput(key, data);
  }

  public static void recordOutput(String key, Pose3d... value) {
    // Map Pose3d[] into double[]
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }

    recordOutput(key, data);
  }

  public static void recordOutput(String key, Trajectory value) {
    // Map trajectory into Pose2d[]
    recordOutput(key,
      value.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new));
  }

  public static void recordOutput(String key, SwerveModuleState... values) {
    // Map states into double[]
    double[] data = new double[values.length * 2];
    for (int i = 0; i < values.length; i++) {
      data[i * 2] = values[i].angle.getRadians(); // Angle recorded in radians
      data[i * 2 + 1] = values[i].speedMetersPerSecond;
    }

    recordOutput(key, data);
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