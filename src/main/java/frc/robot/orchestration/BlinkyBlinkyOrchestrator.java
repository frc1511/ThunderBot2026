package frc.robot.orchestration;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Hang.HangSubsystem;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;

public class BlinkyBlinkyOrchestrator {
    private Constants.BlinkyBlinky.Mode m_currentMode = Constants.BlinkyBlinky.Mode.NONE;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;

    private HangSubsystem hang;

    private int m_strobeProgress = 0;
    private int frame = 0;

    private double m_brightnessPercent;

    public BlinkyBlinkyOrchestrator(Robot robot) {
        m_led = new AddressableLED(Constants.IOMap.BlinkyBlinky.kPWMport);

        m_buffer = new AddressableLEDBuffer(Constants.BlinkyBlinky.kLength);
        m_led.setLength(m_buffer.getLength());

        m_led.setData(m_buffer);
        m_led.start();

        hang = robot.hang;

        m_brightnessPercent = 1d;
    }

    public void sparkle() {
        m_brightnessPercent = SmartDashboard.getNumber("LED_Brightness", 1d);

        if (!Broken.blinkyBlinkyDisableStatus()) {
            switch (m_currentMode) {
                case NONE:
                    m_buffer.forEach((index, r, g, b) -> {
                        m_buffer.setHSV(index, ((index / Constants.BlinkyBlinky.kLength * 180) + frame / 5) % 180, 255, (int)Math.floor(255 * m_brightnessPercent));
                    });
                    break;
                case INTAKING:
                    m_buffer.forEach((index, r, g, b) -> {
                        m_buffer.setHSV(index, 15, 255, 255);
                    });
                    break;
                case HUNG:
                    double position = hang.getPosition();
                    double hangPercent = position / Constants.HangConstants.kMaxDeployDistanceRotations;
                    int fullNumber = Helpers.clamp((int) Math.floor(hangPercent * 9) - 1, 0, 9);
                    double leftover = (hangPercent * 9) - fullNumber;
                    m_buffer.forEach((index, r, g, b) -> {
                        int value = index % 9 <= fullNumber ? 255 : (int) Math.floor(leftover * 255);
                        m_buffer.setHSV(index, 150, 255, value);
                    });
                    break;
                case FIRE_READY:
                    m_strobeProgress = (m_strobeProgress + 1) & 0xff;
                    m_buffer.forEach((index, r, g, b) -> {
                        m_buffer.setHSV(index, 55, m_strobeProgress, (int)Math.floor(255 * m_brightnessPercent));
                    });
                    break;
                case HOME:
                    m_buffer.forEach((index, r, g, b) -> {
                        m_buffer.setHSV(index, 50, 20, (int)Math.floor(255 * m_brightnessPercent));
                    });
                    break;
                case TRENCH_SAFE:
                    m_buffer.forEach((index, r, g, b) -> {
                        m_buffer.setHSV(index, 140, 255, (int)Math.floor(255 * m_brightnessPercent));
                    });
                    break;
                case PIT:
                    m_buffer.forEach((index, r, g, b) -> {
                        m_buffer.setHSV(index, 0, 0, (int)Math.floor(40 * m_brightnessPercent));
                    });
                    break;
                case OFF:
                    m_buffer.forEach((index, r, g, b) -> {
                        m_buffer.setHSV(index, 0, 0, 0);
                    });
                    break;
                default: break;
            }
            ++frame;
            m_led.setData(m_buffer);
        } else {
            m_buffer.forEach((index, r, g, b) -> {
                m_buffer.setHSV(index, 0, 0, 0);
            });
            m_led.setData(m_buffer);
        }
    }

    public Command set(Constants.BlinkyBlinky.Mode mode) {
        return new CommandBuilder()
            .onExecute(() -> {
                if (mode != m_currentMode) {
                    m_currentMode = mode;
                }
            })
            .onEnd(() -> {
                if (mode == m_currentMode) {
                    m_currentMode = Constants.BlinkyBlinky.Mode.NONE;
                }
            })
            .ignoringDisable(true)
            .withName("BlinkySet");
    }
}
