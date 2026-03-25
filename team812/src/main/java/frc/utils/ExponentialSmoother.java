// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

/*
 * ExponentialSmoother uses and exponential smoother to smooth a noisy input.
 */
public class ExponentialSmoother {
    private double alpha = 0.0; // weight of the new sample. (1-alpha is the weight for the old samples)
    private boolean reset = true;
    private double value = 0.0;
    
    /**
     * ExponentialSmoother creates a smoother object
     * @param alpha (double) the percent (from 0.0 to 1.0) of samples to average into the smoothed result. 1.0 = no smoothing.
    */
    public ExponentialSmoother(double alpha) {
      this.alpha = alpha;
      this.reset = true;
    }

    /**
     * reset - forget the past history and start smoothing with the next sample.
     */
    public void reset() {
      reset = true;
    }

    /**
     * addSample - add an additional sample to the smoothed result.
     * @param x (double) the new sample..
     * @return (double) the updated smoothed output.
     */
    public double addSample(double x) {
      if (reset) {
        value = x;
        reset = false;
      } else {
        value = x * alpha + value * (1.0 - alpha);
      }
      return value;
    }
  }