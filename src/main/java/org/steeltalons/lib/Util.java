package org.steeltalons.lib;

/**
 * Contains general utility functions.
 */
public class Util {
  /**
   * Clamps a double within a range.
   *
   * @param value the value to clamp.
   * @param high  the upper limit.
   * @param low   the lower limit.
   * @return the newly clamped value.
   */
  public static double clamp(double value, double high, double low) {
    return Math.max(low, Math.min(value, high));
  }
}
