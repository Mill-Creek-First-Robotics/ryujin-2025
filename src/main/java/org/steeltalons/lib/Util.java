package org.steeltalons.lib;

/**
 * Contains general utility functions.
 */
public class Util {
  /**
   * Clamps a double within a range.
   *
   * @param value the value to clamp.
   * @param low   the lower limit.
   * @param high  the upper limit.
   * @return the newly clamped value.
   */
  public static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(value, high));
  }
}
