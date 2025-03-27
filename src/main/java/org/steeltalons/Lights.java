package org.steeltalons;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Random;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  public static final int kLength = 75;
  private AddressableLED leds = new AddressableLED(0);
  private AddressableLEDBuffer buf = new AddressableLEDBuffer(kLength);
  private AddressableLEDBufferView left = buf.createView(0, kLength - 1);
  // private AddressableLEDBufferView right = buf.createView(kLength, kLength * 2 - 1).reversed();
  private Animation currentAnimation;

  public Lights() {
    leds.setLength(buf.getLength());
    leds.start();
    buf.forEach((i, r, g, b) -> buf.setRGB(i, 0xff, 0, 0));
    leds.setData(buf);

    startColorWipe();
  }

  public void updateDefaultCommand() {
    setDefaultCommand(
        Commands.repeatingSequence(
            runOnce(currentAnimation),
            Commands.waitTime(currentAnimation.getDelay())).ignoringDisable(true));
  }

  public void startFlame(Alliance alliance) {
    if (getDefaultCommand() != null) {
      getDefaultCommand().cancel();
    }
    clearLights();
    currentAnimation = new Flame(this, 50, 120, alliance);
    updateDefaultCommand();
  }

  public void startColorWipe() {
    if (getDefaultCommand() != null) {
      getDefaultCommand().cancel();
    }
    clearLights();
    currentAnimation = new ColorWipe(this, kLength);
    updateDefaultCommand();
  }

  public void setRGB(int index, int r, int g, int b) {
    left.setRGB(index, r, g, b);
    // right.setRGB(index, r, g, b);
  }

  public void clearLights() {
    left.forEach((i, r, g, b) -> {
      setRGB(i, 0, 0, 0);
    });
  }

  public void updateBuf() {
    leds.setData(buf);
  }
}

interface Animation extends Runnable {
  default void run() {
    play();
  }

  /** Plays an iteration of the Animation. */
  void play();

  default boolean isFinished() {
    return false;
  }

  void stop();

  /** Returns the amount of time between iterations. */
  default Time getDelay() {
    return Milliseconds.of(0);
  }
}

class ColorWipe implements Animation {
  private Lights lights;
  private int index = 0;
  private int length;

  public ColorWipe(Lights lights, int length) {
    this.lights = lights;
    this.length = length;
  }

  @Override
  public void play() {
    if (index >= length - 1)
      return;
    lights.setRGB(index++, 0, 0xff, 0);
    lights.updateBuf();
  }

  @Override
  public void stop() {
  }

  @Override
  public Time getDelay() {
    return Milliseconds.of(50);
  }

  @Override
  public boolean isFinished() {
    return index >= length - 1;
  }
}

class Flame implements Animation {
  private Lights lights;
  private final int cooling;
  private final int sparking;
  private final int[] heat = new int[Lights.kLength];
  private final Random rand = new Random();
  private final Runnable updateColors;

  public Flame(Lights lights, int cooling, int sparking, Alliance alliance) {
    this.lights = lights;
    this.cooling = cooling;
    this.sparking = sparking;
    if (alliance == Alliance.Blue) {
      updateColors = this::updateColorBlue;
    } else {
      updateColors = this::updateColorRed;
    }
  }

  @Override
  public Time getDelay() {
    return Milliseconds.of(15);
  }

  @Override
  public void stop() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void play() {
    updateHeat();
    updateColors.run();
    lights.updateBuf();
  }

  private void updateColorBlue() {
    for (int i = 0; i < Lights.kLength; i++) {
      int tl92 = Math.round(heat[i] / 255f * 191);
      int heatramp = tl92 & 0x3f;
      heatramp <<= 2;

      if (tl92 > 0x80) {
        lights.setRGB(i, heatramp, heatramp, 255);
      } else if (tl92 > 0x40) {
        lights.setRGB(i, 0, heatramp, 255);
      } else {
        lights.setRGB(i, 0, 0, heatramp);
      }
    }
  }

  private void updateColorRed() {
    for (int i = 0; i < Lights.kLength; i++) {
      int tl92 = Math.round(heat[i] / 255f * 191);
      int heatramp = tl92 & 0x3f;
      heatramp <<= 2;

      if (tl92 > 0x80) {
        lights.setRGB(i, 255, 255, heatramp);
      } else if (tl92 > 0x40) {
        lights.setRGB(i, 255, heatramp, 0);
      } else {
        lights.setRGB(i, heatramp, 0, 0);
      }
    }
  }

  private void updateHeat() {
    int cooldown;

    for (int i = 0; i < heat.length; i++) {
      cooldown = rand.nextInt(cooling * 10 / 60 + 2);
      if (cooldown > heat[i]) {
        heat[i] = 0;
      } else {
        heat[i] = (heat[i] - cooldown);
      }
    }

    for (int i = heat.length - 1; i >= 2; i--) {
      heat[i] = Math.min((heat[i - 1] + 2 * heat[i - 2]) / 3, 255);
    }

    if (rand.nextInt(255) < sparking) {
      int y = rand.nextInt(7);
      heat[y] = Math.min(heat[y] + rand.nextInt(160, 255), 255);
    }
  }
}
