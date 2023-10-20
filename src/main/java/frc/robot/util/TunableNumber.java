package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;

import java.util.ArrayList;
import java.util.HashSet;
import edu.wpi.first.networktables.GenericEntry;

/** Represents a double that can be changed during runtime. */
public class TunableNumber implements DoubleSupplier, DoubleConsumer {
  /** List of instances. */
  private static final ArrayList<TunableNumber> s_instances = new ArrayList<TunableNumber>();
  /** List of groups. */
  private static final HashSet<String> s_groups = new HashSet<String>();

  /** Descriptor of the TunableNumber for use with dashboards. */
  public final String m_name;
  /** Descriptor of associated TunableNumbers for use with dashboards. */
  public final String m_group;

  /** Value of the TunableNumber at startup. */
  public final double m_default;
  /** Value of the TunableNumber. */
  private double m_value;

  /** NetworkTables entry of the number. */
  private GenericEntry m_entry;

  /** True if the value of the number can be changed. */
  private boolean m_mutable = false;

  /** Function to run when the value is changed. */
  private DoubleConsumer m_bindOnChange;

  /**
   * Create a new TunableNumber.
   *
   * @param name descriptor for dashboards
   * @param val initial value
   * @param group descriptor for associated TunableNumbers
   */
  public TunableNumber(String name, double val, String group) {
    m_name = name;
    m_default = val;
    m_value = m_default;

    s_instances.add(this);

    m_group = group;
    s_groups.add(group);
  }

  /**
   * Create a new TunableNumber in the {@code default} group.
   *
   * @param name descriptor for dashboards
   * @param val initial value
   */
  public TunableNumber(String name, double val) {
    this(name, val, "default");
  }

  /** @return an array of all instances for class-wide changes (e.g. making all numbers mutable) */
  public static TunableNumber[] getAllInstances() {
    return s_instances.toArray(new TunableNumber[s_instances.size()]);
  }

  /** @return an array of all TunableNumber groups */
  public static String[] getAllGroups() {
    return s_groups.toArray(new String[s_groups.size()]);
  }

  /**
   * Get all instances within a specified group.
   *
   * @param group group of instances to return (case sensitive)
   * @return an array of all instances in the group
   */
  public static TunableNumber[] getGroup(String group) {
    final ArrayList<TunableNumber> fullGroup = new ArrayList<TunableNumber>();

    fullGroup.addAll(s_instances);

    fullGroup.removeIf(e -> !e.m_group.equals(group));
    return fullGroup.toArray(new TunableNumber[fullGroup.size()]);
  }

  /** Update all numbers with values from NetworkTables, if provided. */
  public static void updateAll() {
    s_instances.forEach(e -> e.update());
  }

  /**
   * Sets NetworkTables entry to allow for updates through NetworkTables.
   *
   * @param entry new entry
   */
  public void setEntry(GenericEntry entry) {
    m_entry = entry;
  }

  /** @return {@code true} if the number is bound to a NetworkTables entry */
  public boolean hasEntry() {
    return m_entry != null;
  }

  /** Update number with value from NetworkTables entry, if provided. */
  public void update() {
    if (m_entry != null) {
      accept(m_entry.getDouble(m_value));
    }
  }

  /** @return {@code true} if the number is mutable */
  public boolean getMutable() {
    return m_mutable;
  }

  /** @param mutable {@code true} if the number should be mutable */
  public void setMutable(boolean mutable) {
    m_mutable = mutable;
  }

  /**
   * Set a function to be called when the value is changed.
   *
   * @param bindOnChange {@link DoubleConsumer} to be called with the new value
   */
  public void bindTo(DoubleConsumer bindOnChange) {
    m_bindOnChange = bindOnChange;
  }

  /** Resets value to initialized value. */
  public void reset() {
    accept(m_default);
  }

  /** @return the current value */
  @Override
  public double getAsDouble() {
    return m_value;
  }

  /**
   * Change the value, ignoring duplicates. Only works if the number is mutable.
   *
   * @param val new value
   *
   * @see TunableNumber#getMutable
   * @see TunableNumber#setMutable
   */
  @Override
  public void accept(double val) {
    if (m_mutable && m_value != val) {
      m_value = val;
      if (m_entry != null) {
        m_entry.setDouble(val);
      }
      if (m_bindOnChange != null) {
        m_bindOnChange.accept(m_value);
      }
    }
  }
}