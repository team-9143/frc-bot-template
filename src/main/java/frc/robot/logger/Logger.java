package frc.robot.logger;

import frc.robot.Constants.Config;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.HashMap;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotBase;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import edu.wpi.first.wpilibj.DriverStation;

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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * <p> Huge thanks to FRC Team 581: Littleton Robotics for the basis for this implementation. </p>
 *
 * <p> For consistency, logging calls should be done by subsystems and the main robot class (after control logic), and be under a directory corresponding to the same. </p>
 *
 * A logger that is almost fully compatible with the AdvantageKit `Logger` class. It uses WPILib's DataLog to log the data to a file, and streams the data using NetworkTables 4.
 *
 * @see https://github.com/team581/frc-2023-charged-up/pull/64
 */
public class Logger {
  private static final String OUTPUT_LOG_DIR = "/RealOutputs/";
  private static final String METADATA_LOG_DIR = "/RealMetadata/";

  /** List of loggables to be updated each period. */
  private static final ArrayList<Loggable> loggables = new ArrayList<>();

  // Entry maps for logging to disk
  private static final HashMap<String, BooleanLogEntry> booleanLogs = new HashMap<>();
  private static final HashMap<String, DoubleLogEntry> doubleLogs = new HashMap<>();
  private static final HashMap<String, FloatLogEntry> floatLogs = new HashMap<>();
  private static final HashMap<String, IntegerLogEntry> integerLogs = new HashMap<>();
  private static final HashMap<String, StringLogEntry> stringLogs = new HashMap<>();
  private static final HashMap<String, BooleanArrayLogEntry> booleanArrayLogs = new HashMap<>();
  private static final HashMap<String, DoubleArrayLogEntry> doubleArrayLogs = new HashMap<>();
  private static final HashMap<String, FloatArrayLogEntry> floatArrayLogs = new HashMap<>();
  private static final HashMap<String, IntegerArrayLogEntry> integerArrayLogs = new HashMap<>();
  private static final HashMap<String, StringArrayLogEntry> stringArrayLogs = new HashMap<>();

  // Network tables Tables for data logging directories
  private static NetworkTable outputTable;
  private static NetworkTable metadataTable;

  // Publisher maps for logging to network tables
  private static final HashMap<String, BooleanPublisher> booleanPublishers = new HashMap<>();
  private static final HashMap<String, DoublePublisher> doublePublishers = new HashMap<>();
  private static final HashMap<String, FloatPublisher> floatPublishers = new HashMap<>();
  private static final HashMap<String, IntegerPublisher> integerPublishers = new HashMap<>();
  private static final HashMap<String, StringPublisher> stringPublishers = new HashMap<>();
  private static final HashMap<String, BooleanArrayPublisher> booleanArrayPublishers = new HashMap<>();
  private static final HashMap<String, DoubleArrayPublisher> doubleArrayPublishers = new HashMap<>();
  private static final HashMap<String, FloatArrayPublisher> floatArrayPublishers = new HashMap<>();
  private static final HashMap<String, IntegerArrayPublisher> integerArrayPublishers = new HashMap<>();
  private static final HashMap<String, StringArrayPublisher> stringArrayPublishers = new HashMap<>();

  // Cannot be initialized until after start to ensure proper file creation
  private static DataLog log;

  // Start log manager
  static {
    DataLogManager.start(
      RobotBase.isSimulation() ? "" : Config.DATA_LOG_DIR, // Log to project directory in simulation
      "FRC_" + LocalDateTime.now(ZoneId.of("UTC-8")).format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH.mm.ss")) + ".wpilog"); // File name

    log = DataLogManager.getLog();
    // Disable logging NT values to separate logger and networktables
    DataLogManager.logNetworkTables(false);
    // Log inputs from DriverStation
    DriverStation.startDataLog(log);

    // Set up network tables for data stream
    if (Config.NTStream) {
      var table = NetworkTableInstance.getDefault().getTable("AdvantageKit");
      outputTable = table.getSubTable("RealOutputs");
      metadataTable = table.getSubTable("RealMetadata");
    }
  }

  /** @param loggable loggable to register for periodic update calls */
  public static void registerLoggable(Loggable loggable) {
    loggables.add(loggable);
    log("Device " + loggable.getDirectory() + " initialized");
  }

  /** Update all registered loggables. Should be called every period. */
  public static void updateLogs() {
    loggables.forEach(e -> e.log());
  }

  /**
   * Log a message to the "messages" entry. The message is also printed to standard output.
   *
   * @param msg message
   */
  public static void log(String msg) {
    DataLogManager.log(msg);
  }

  public static void recordOutput(String path, boolean value) {
    booleanLogs
      .computeIfAbsent(path, k -> new BooleanLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      booleanPublishers
        .computeIfAbsent(path, k -> outputTable.getBooleanTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  public static void recordOutput(String path, double value) {
    doubleLogs
      .computeIfAbsent(path, k -> new DoubleLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      doublePublishers
        .computeIfAbsent(path, k -> outputTable.getDoubleTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  public static void recordOutput(String path, float value) {
    floatLogs
      .computeIfAbsent(path, k -> new FloatLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      floatPublishers
        .computeIfAbsent(path, k -> outputTable.getFloatTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  public static void recordOutput(String path, int value) {
    integerLogs
      .computeIfAbsent(path, k -> new IntegerLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      integerPublishers
        .computeIfAbsent(path, k -> outputTable.getIntegerTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  public static void recordOutput(String path, String value) {
    stringLogs
      .computeIfAbsent(path, k -> new StringLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(value); // Record value to log file

    if (Config.NTStream) {
      stringPublishers
        .computeIfAbsent(path, k -> outputTable.getStringTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(value); // Record value to network tables
    }
  }

  public static void recordOutput(String path, boolean[] values) {
    booleanArrayLogs
      .computeIfAbsent(path, k -> new BooleanArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      booleanArrayPublishers
        .computeIfAbsent(path, k -> outputTable.getBooleanArrayTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  public static void recordOutput(String path, double[] values) {
    doubleArrayLogs
      .computeIfAbsent(path, k -> new DoubleArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      doubleArrayPublishers
        .computeIfAbsent(path, k -> outputTable.getDoubleArrayTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  public static void recordOutput(String path, float[] values) {
    floatArrayLogs
      .computeIfAbsent(path, k -> new FloatArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      floatArrayPublishers
        .computeIfAbsent(path, k -> outputTable.getFloatArrayTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  public static void recordOutput(String path, long[] values) {
    integerArrayLogs
      .computeIfAbsent(path, k -> new IntegerArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      integerArrayPublishers
        .computeIfAbsent(path, k -> outputTable.getIntegerArrayTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  public static void recordOutput(String path, int[] values) {
    // Transform int[] into long[]
    long[] longs = new long[values.length];
    for (int i = 0; i < values.length; i++) {
      longs[i] = values[i];
    }

    recordOutput(path, longs);
  }

  public static void recordOutput(String path, String[] values) {
    stringArrayLogs
      .computeIfAbsent(path, k -> new StringArrayLogEntry(log, OUTPUT_LOG_DIR + path)) // Retrieve/create log entry
      .append(values); // Record value to log file

    if (Config.NTStream) {
      stringArrayPublishers
        .computeIfAbsent(path, k -> outputTable.getStringArrayTopic(path).publish(PubSubOption.keepDuplicates(true))) // Retrieve/create publisher
        .set(values); // Record value to network tables
    }
  }

  public static void recordOutput(String path, Pose2d... value) {
    // Map Pose2d[] into double[]
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }

    recordOutput(path, data);
  }

  public static void recordOutput(String path, Pose3d... value) {
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

    recordOutput(path, data);
  }

  public static void recordOutput(String path, Trajectory value) {
    // Map trajectory into Pose2d[]
    recordOutput(path,
      value.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new));
  }

  public static void recordOutput(String path, ChassisSpeeds value) {
    // Map speeds in double[]
    recordOutput(path, new double[] {value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond});
  }

  public static void recordOutput(String path, SwerveModuleState... values) {
    // Map states into double[]
    double[] data = new double[values.length * 2];
    for (int i = 0; i < values.length; i++) {
      data[i * 2] = values[i].angle.getRadians(); // Angle recorded in radians
      data[i * 2 + 1] = values[i].speedMetersPerSecond;
    }

    recordOutput(path, data);
  }

  public static void recordMetadata(String path, String value) {
    // Record metadata in log file
    var entry = new StringLogEntry(log, METADATA_LOG_DIR + path);
    entry.append(value);
    entry.finish(); // Close unused entry

    // Record metadata to network tables
    if (Config.NTStream) {
      var publisher = metadataTable.getStringTopic(path).publish();
      publisher.set(value);
      publisher.close(); // Close unused publisher
    }
  }
}