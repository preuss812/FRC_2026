// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} or SlewRateLimiter instead.
 */

/** Add your docs here. */
public class ModifiedSlewRateLimiter {
  private final double m_increasingRateLimit;
  private final double m_decreasingRateLimit;
  private double m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiter with the given increasing and decreasing rate limits and initial
   * value.
   *
   * @param increasingRateLimit The rate-of-change limit in the increasing magnitude, in units per
   *     second. This is expected to be positive.
   * @param decreasingRateLimit The rate-of-change limit in the decreasing magnitude, in units per
   *     second. This is expected to be positive.
   * @param initialValue The initial value of the input.
   */
  public ModifiedSlewRateLimiter(double increasingRateLimit, double decreasingRateLimit, double initialValue) {
    m_increasingRateLimit = increasingRateLimit;
    m_decreasingRateLimit = decreasingRateLimit;
    m_prevVal = initialValue;
    m_prevTime = MathSharedStore.getTimestamp();
  }

  /**
   * Creates a new SlewRateLimiter with the given increasing rate limit and decreasing rate limit of
   * -rateLimit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public ModifiedSlewRateLimiter(double rateLimit) {
    this(rateLimit, -rateLimit, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    elapsedTime = 1.0/50.0;
    if ( (Math.signum(input) == Math.signum(m_prevVal) || Math.signum(m_prevVal) == 0)
        && Math.abs(input) > Math.abs(m_prevVal)) {
        m_prevVal +=
            MathUtil.clamp(
                input - m_prevVal,
                - m_increasingRateLimit * elapsedTime,
                  m_increasingRateLimit * elapsedTime);
    } else {
        m_prevVal +=
        MathUtil.clamp(
            input - m_prevVal,
            - m_decreasingRateLimit * elapsedTime,
              m_decreasingRateLimit * elapsedTime);
    }
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = MathSharedStore.getTimestamp();
  }
}

