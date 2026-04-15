package frc.robot.orchestration;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Broken;
import frc.util.Constants;
import frc.util.Helpers;

public class BlinkyBlinkyOrchestrator {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;

    private Robot robot;

    private int m_strobeProgress = 0;
    private int frame = 0;

    private double m_brightnessPercent;

    public BlinkyBlinkyOrchestrator(Robot robot) {
        m_led = new AddressableLED(Constants.IOMap.BlinkyBlinky.kPWMport);

        m_buffer = new AddressableLEDBuffer(Constants.BlinkyBlinky.kLength);
        m_led.setLength(m_buffer.getLength());

        m_led.setData(m_buffer);
        m_led.start();

        this.robot = robot;

        m_brightnessPercent = 1d;
        SmartDashboard.putNumber("LEDs / Brightness", 1d);
    }

    private int percentToV(double percent) {
        return (int)Math.floor(255 * percent);
    }

    public void sparkle() {
        m_brightnessPercent = SmartDashboard.getNumber("LEDs / Brightness", 1d);

        if (!Broken.blinkyBlinkyDisableStatus()) {
            if (robot.pitMode.isOn()) {
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 0, 0, percentToV(m_brightnessPercent * 0.6));
                });
            } else if (robot.hang.climbClimbingButHasntClumbJustYet()) {
                double position = robot.hang.getPosition();
                double hangPercent = position / Constants.HangConstants.kMaxDeployDistanceRotations;
                int fullNumber = Helpers.clamp((int) Math.floor(hangPercent * 9) - 1, 0, 9);
                double leftover = (hangPercent * 9) - fullNumber;
                m_buffer.forEach((index, r, g, b) -> {
                    int value = index % 9 <= fullNumber ? 255 : percentToV(leftover);
                    m_buffer.setHSV(index, 150, 255, value);
                });
            } else if (robot.intake.isRunning()) {
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 15, 255, 255);
                });
            } else if (robot.conductor.cannonReady()) {
                m_strobeProgress = (m_strobeProgress + 1) & 0xff;
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 55, m_strobeProgress, percentToV(m_brightnessPercent));
                });
            } else if (robot.conductor.inStartingConfiguration()) {
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 50,  20,  percentToV(m_brightnessPercent));
                });
            } else if (robot.conductor.trenchSafe()) {
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 140, 255, percentToV(m_brightnessPercent));
                });
            } else {
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, ((index / Constants.BlinkyBlinky.kLength * 180) + frame / 5) % 180, 255, percentToV(m_brightnessPercent));
                });
            }
            ++frame;
        } else {
            m_buffer.forEach((index, r, g, b) -> {
                m_buffer.setHSV(index, 0, 0, 0);
            });
        }

        m_led.setData(m_buffer);
    }
}
