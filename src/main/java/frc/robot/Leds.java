package frc.robot;
import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
public class Leds {
    Timer ledTimer = new Timer();
    AddressableLED leds = new AddressableLED(0);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);
    int[] hues = new int[buffer.getLength()];
    int[] red = new int[buffer.getLength()];
    int[] green = new int[buffer.getLength()];
    int []blue = new int[buffer.getLength()];
    int testVar = 0;
    Random random = new Random();
    public Leds() {
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
        ledTimer.start();
        for (int i = 0; i < buffer.getLength(); i++) {
            int color = i * 3;
            hues[i] = color;
        }

        for (int i = 0; i < buffer.getLength(); i++) {
        int color = i * 3;
        hues[i] = color;
        }
        int thingy = 0;
        for (int i = 0; i < buffer.getLength() / 3; i++) {
        red[thingy] = 255;
        green[thingy] = 0;
        blue[thingy] = 0;
        thingy++;
        red[thingy] = 150;
        green[thingy] = 150;
        blue[thingy] = 150;
        thingy++;
        red[thingy] = 0;
        green[thingy] = 0;
        blue[thingy] = 255;
        thingy++;
        }
    }
    public void resetTimer() {
        ledTimer.reset();
    }
    public void setRGBFunction(int red, int green, int blue) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, red, green, blue);
        }
        leds.setData(buffer);
    }
    public void setHSVFunction(int hue, int s, int v) {
    for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setHSV(i, hue, s, v);
    }
    leds.setData(buffer);
    }
    public void rainbowHSV() {
    for (int i = 0; i < buffer.getLength(); i++) {
       buffer.setHSV(i, hues[testVar], 255, 255);
       testVar = testVar + 1;
    if (testVar >= 60) {
        testVar -= 60;
      }
    }
    testVar = testVar + 1;
    if (testVar >= 60) {
        testVar -= 60;
    }
    leds.setData(buffer);
  }
    public void aLED() {
    if (ledTimer.get() > 0.5) {
        for (int i = 0; i < buffer.getLength(); i++) {
       buffer.setRGB(i, red[testVar], green[testVar], blue[testVar]);
       testVar = testVar + 1;
    if (testVar >= buffer.getLength()) {
        testVar -= 60;
      }
    }
    testVar = testVar + 1;
    if (testVar >= buffer.getLength()) {
        testVar -= 60;
    }
    leds.setData(buffer);
    resetTimer();
    }

  }
  public boolean intakedRing() {
    if (ledTimer.get() > 0.5) {
        setRGBFunction(0, 0, 255);
    } else if(ledTimer.get() > 1) {
        setRGBFunction(255, 255, 255);
    } else if (ledTimer.get() > 1.5) {
        setRGBFunction(0, 0, 255);
    } else if (ledTimer.get() > 2) {
        setRGBFunction(255, 255, 255);
    } else if (ledTimer.get() < 2) {
        resetTimer();
        return true;
    }
    return false;
  }
  public void fadingRainbow() {
    // reset.///// timer before use
    double hsv = ledTimer.get() * 20;
    if (!(hsv > 180)){
    setHSVFunction((int)hsv, 255, 255);
    } else {
    resetTimer();
    }
    }
    public void fireLED() {
        // reset timer
        if (ledTimer.get() < 1) {
            setHSVFunction(12, 255, random.nextInt(130,204));
        } else if (ledTimer.get() < 2) {
            random.ints(75,87);
            setHSVFunction(12, 255, random.nextInt(140,221));
        } else if (ledTimer.get() < 3) {
            setHSVFunction(12, 255, random.nextInt(170,255));
        } else if (ledTimer.get() < 4) {
            resetTimer();
        }
        
    }
    public void lowHighHSV() {
        // reset timer
        // 117 
        // 87
        double ledNumber = ledTimer.get() * 30;
        if (ledTimer.get() < 1 ) {
            setHSVFunction(117 - (int)ledNumber, 255, 255);
        } else if ((ledTimer.get() >= 1) && (ledTimer.get() < 2) ) {
            setHSVFunction(87 + ((int)ledNumber - 30), 255, 255);
        } else {
            resetTimer();
        }
    }
    public void zoomingHSV() {
        if (ledTimer.get() * 40 > 59) {
            resetTimer();
        }
        double ledSpot = ledTimer.get() * 40;
        for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setHSV(i, 0,10, 10);
        }
        buffer.setHSV((int)ledSpot, 50, 204, 232);
        leds.setData(buffer);
    }
}
