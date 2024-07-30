// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// 21 inches left to right
// 20.5 front to back
package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.Drivetrain;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   AddressableLED leds;
   AddressableLEDBuffer buffer;
   Timer timer;
   int[] hues;
   int[] red;
   int[] green;
   int[] blue;
   int testVar = 0;
   int positions = 0;
   PhotonCamera camera = new PhotonCamera("shooterCamera");
   XboxController xbox = new XboxController(0);
   Drivetrain driveTrain = new Drivetrain();
   Intake intake = new Intake();
   Shooter shooter = new Shooter();
   double storingAngle = 0;
   double storingSpeed = 0;
   boolean ringRGB = false;
  public void robotInit()
  {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(60);
    timer = new Timer();

    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
    timer.start();

    hues = new int[buffer.getLength()];
    red = new int[buffer.getLength()];
    green = new int[buffer.getLength()];
    blue = new int[buffer.getLength()];

    for (int i = 0; i < buffer.getLength(); i++) {
      int color = i * 3;
      hues[i] = color;
    }
    //huesA = new int[buffer.getLength()];
    int thingy = 0;
    for (int i = 0; i < buffer.getLength() / 3; i++) {
      red[thingy] = 255;
      green[thingy] = 0;
      blue[thingy] = 0;
      thingy++;
      red[thingy] = 255;
      green[thingy] = 255;
      blue[thingy] = 255;
      thingy++;
      red[thingy] = 0;
      green[thingy] = 0;
      blue[thingy] = 255;
      thingy++;
      }
    }
  public void setRGB(int red, int green, int blue) {
  for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, red, green, blue);
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

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    

  }

  @Override
  public void teleopPeriodic()
  {
    double x = -xbox.getLeftY();
    double y = -xbox.getLeftX();
    double x2 = -xbox.getRightX();
    boolean a = xbox.getAButtonPressed();
    boolean b = xbox.getBButton();
    boolean yXbox = xbox.getYButton();
    x2 = x2 * 6;
    x = x * 2;
    y = y * 2;
    double angleRadians = Math.atan2(y, x);
    if(Math.sqrt(x*x + y*y) < 0.2) {
      y = 0;
      x = 0;
    }
    if(Math.sqrt(x2*x2) < 0.2) { 
      x2 = 0;
    }
    storingAngle = y;
    ChassisSpeeds speed = new ChassisSpeeds(x,y,x2);
    if (a) {
      driveTrain.zeroPigeon();
    }
    driveTrain.feildOrienteDrive(speed);
    if (b) {
      intake.getRing();
    } else {
      intake.stopMotor();
    }
    if (intake.hasRing() || ringRGB) {
      double timeMod = timer.get() % 5;
      setRGB(0,0,0);
      if (timeMod < 3) {
        setRGB(255,0,0);
        ringRGB = true;
      } else if (timeMod > 1 && timeMod < 3) {
        ringRGB = true;
      } else {
        ringRGB = false;
      }
    } else {
      rainbowHSV();
    }
    if (yXbox) {
      shooter.shoot();
    } else {
      shooter.notShoot();
    }
  }
  @Override
  public void disabledInit() {}

  public void aLED() {
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
  }
  public void yawRGNB() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets){
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      if (yaw > 0) {
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setRGB(i, 255, 0, 0);
          leds.setData(buffer);
        } 
      }
      if (yaw < 0){
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setRGB(i, 0, 0, 255);
          leds.setData(buffer);
      }
    }
   } else {
      for (int i = 0; i < buffer.getLength(); i++) {
       buffer.setHSV(i, hues[testVar], 255, 255);
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
    }

  }
  public void redBlue() {
        if ((timer.get() % 2) < 1) {
      for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 255);
      }
      leds.setData(buffer);
    } else {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 255, 0, 0);
      }
      leds.setData(buffer);
    }
  }
  public void rangeRGB() {
        var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (result.hasTargets()) {
       double rangeMeters = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
      if (rangeMeters < 0.5) {
         setRGB(255,0,0);
      }
      if (rangeMeters > 0.5 && rangeMeters < 1) {
        setRGB(0,255,0);
      }
      if (rangeMeters > 1 && rangeMeters < 1.5) {
       setRGB(0,0,255);
      }
      if (rangeMeters > 1.5 && rangeMeters < 2) {
        setRGB(255,0,255);
      }
    }
  }
  public void rainbowTimer(int startPosition) {
    int hue = 0;
    for (int i = 0; i < buffer.getLength(); i++){
      buffer.setHSV(startPosition, hue, 255, 255);
      if (startPosition == 59) {
        startPosition -= 59;
      } else {
        startPosition++;
      }
      if (hue == 0){
        hue = 3;
      } else {
        hue = hue + 3;
      }
    }
    leds.setData(buffer);
  }

  @Override
  public void disabledPeriodic() {
    rainbowHSV();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
