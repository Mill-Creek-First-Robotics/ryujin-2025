package org.steeltalons.lib;

import static org.steeltalons.Constants.kTuningModeEnabled;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
public class TunableNumber {
  private final String key;
  private double defaultValue = 0.0;
  private double lastHasChangedValue = defaultValue;
  private String tableKey = "TunableNumbers";

  /**
   * Creates a new TunableNumber. Returns default (0.0) if the value is not found
   * in the dashboard.
   *
   * @param dashboardKey the key to put this TunableNumber under on NetworkTables.
   */
  public TunableNumber(String dashboardKey) {
    key = tableKey + "/" + dashboardKey;
  }

  /**
   * Creates a new TunableNumber. Returns default (0.0) if the value is not found
   * in the dashboard.
   *
   * @param dashboardKey the key to put this TunableNumber under on NetworkTables.
   * @param defaultValue the default value to use if the number isn't found under
   *                     the provided key.
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    this.defaultValue = defaultValue;
  }

  /**
   * Get the current value from the dashboard if in tuning mode. Otherwise, the
   * default value will be returned.
   */
  public double get() {
    if (kTuningModeEnabled) {
      return SmartDashboard.getNumber(key, defaultValue);
    }
    return defaultValue;
  }

  /** Checks whether the number has changed since the last check. */
  public boolean hasChanged() {
    if (get() != lastHasChangedValue) {
      lastHasChangedValue = get();
      return true;
    }
    return false;
  }
}
