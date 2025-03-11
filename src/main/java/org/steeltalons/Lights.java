package org.steeltalons;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class Lights {
  private AddressableLED leds = new AddressableLED(1);
  @SuppressWarnings("unused")
  private AddressableLEDSim ledSim = AddressableLEDSim.createForChannel(1);
  private AddressableLEDBuffer buf = new AddressableLEDBuffer(120);
  private AddressableLEDBufferView left = buf.createView(0, 59);
  private AddressableLEDBufferView right = buf.createView(60, 119).reversed();
  private Flame flameAnimation;
  private ColorWipe colorWipeAnimation;
  private Thread flameThread;
  private Thread colorWipeThread;

  public Lights() {
    leds.setLength(buf.getLength());
    leds.setData(buf);
    leds.start();

    // green color wipe animation. Transitions to solid color
    // is a virtual thread so that the delay is non blocking to the main thread.
    startColorWipe();
  }

  public void startFlame(Alliance alliance) {
    try {
      if (colorWipeThread != null) {
        colorWipeAnimation.close();
        colorWipeThread.join();
      }
      if (flameThread != null && flameThread.isAlive()) {
        // continue current animation
        return;
      }
      flameAnimation = new Flame(this, 50, 120, 15, alliance);
      flameThread = Thread.startVirtualThread(flameAnimation);
    } catch (InterruptedException e) {
      DriverStation.reportWarning("[LIGHTS]: Something went wrong :(" + e.getMessage(), true);
    }
  }

  public void startColorWipe() {
    try {
      if (flameThread != null) {
        flameAnimation.close();
        flameThread.join();
      }
      if (colorWipeThread != null && colorWipeThread.isAlive()) {
        // reset animation
        colorWipeAnimation.close();
        colorWipeThread.join();
      }
      colorWipeAnimation = new ColorWipe(this);
      colorWipeThread = Thread.startVirtualThread(colorWipeAnimation);
    } catch (InterruptedException e) {
      DriverStation.reportWarning("[LIGHTS]: Something went wrong :(" + e.getMessage(), true);
    }
  }

  public void clearLights() {
    left.forEach((i, r, g, b) -> {
      left.setRGB(i, 0, 0, 0);
      right.setRGB(i, 0, 0, 0);
    });
  }

  public AddressableLEDBufferView getLeft() {
    return left;
  }

  public AddressableLEDBufferView getRight() {
    return right;
  }

  public void updateBuf() {
    leds.setData(buf);
  }
}

class ColorWipe implements Runnable {
  private Lights lights;
  private boolean shouldExit = false;

  public ColorWipe(Lights lights) {
    this.lights = lights;
  }

  @Override
  public void run() {
    lights.getLeft().forEach((i, r, g, b) -> {
      if (shouldExit) {
        return;
      }
      lights.getLeft().setRGB(i, 0, 0xff, 0);
      lights.getRight().setRGB(i, 0, 0xff, 0);
      lights.updateBuf();
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        lights.clearLights();
      }
    });
  }

  public void close() {
    shouldExit = true;
  }
}

class Flame implements Runnable {
  private Lights lights;
  private AddressableLEDBufferView left;
  private AddressableLEDBufferView right;
  private final int cooling;
  private final int sparking;
  private final int speedDelay;
  private final int[] heat = new int[60];
  private final Random rand = new Random();
  private final Runnable updateColors;
  private boolean shouldExit = false;

  public Flame(Lights lights, int cooling, int sparking, int speedDelay, Alliance alliance) {
    this.lights = lights;
    left = lights.getLeft();
    right = lights.getRight();
    this.cooling = cooling;
    this.sparking = sparking;
    this.speedDelay = speedDelay;
    if (alliance == Alliance.Blue) {
      updateColors = this::updateColorBlue;
    } else {
      updateColors = this::updateColorRed;
    }
  }

  public void close() {
    shouldExit = true;
  }

  @Override
  public void run() {
    while (!shouldExit) {
      updateColors.run();

      try {
        Thread.sleep(speedDelay);
      } catch (InterruptedException e) {
        DriverStation.reportWarning("[LIGHTS]: " + e.getMessage(), true);
        break;
      }
    }
    lights.clearLights();
  }

  private void updateColorBlue() {
    updateHeat();
    for (int i = 0; i < 60; i++) {
      int tl92 = Math.round(heat[i] / 255f * 191);
      int heatramp = tl92 & 0x3f;
      heatramp <<= 2;

      if (tl92 > 0x80) {
        left.setRGB(i, heatramp, heatramp, 255);
        right.setRGB(i, heatramp, heatramp, 255);
      } else if (tl92 > 0x40) {
        left.setRGB(i, 0, heatramp, 255);
        right.setRGB(i, 0, heatramp, 255);
      } else {
        left.setRGB(i, 0, 0, heatramp);
        right.setRGB(i, 0, 0, heatramp);
      }
    }
    lights.updateBuf();
  }

  private void updateColorRed() {
    updateHeat();
    for (int i = 0; i < 60; i++) {
      int tl92 = Math.round(heat[i] / 255f * 191);
      int heatramp = tl92 & 0x3f;
      heatramp <<= 2;

      if (tl92 > 0x80) {
        left.setRGB(i, 255, 255, heatramp);
        right.setRGB(i, 255, 255, heatramp);
      } else if (tl92 > 0x40) {
        left.setRGB(i, 255, heatramp, 0);
        right.setRGB(i, 255, heatramp, 0);
      } else {
        left.setRGB(i, heatramp, 0, 0);
        right.setRGB(i, heatramp, 0, 0);
      }
    }
    lights.updateBuf();
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
