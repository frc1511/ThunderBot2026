package frc.robot.orchestration;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.util.CommandBuilder;
import frc.util.Constants;

public class BlinkyBlinkyOrchestrator {
    private Constants.BlinkyBlinky.Mode m_currentMode = Constants.BlinkyBlinky.Mode.NONE;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;

    public BlinkyBlinkyOrchestrator(Robot robot) {
        m_led = new AddressableLED(Constants.IOMap.BlinkyBlinky.kPWMport);

        m_buffer = new AddressableLEDBuffer(Constants.BlinkyBlinky.kLength);
        m_led.setLength(m_buffer.getLength());

        m_led.setData(m_buffer);
        m_led.start();
    }

    public void sparkle() {
        switch (m_currentMode) {
            case NONE:
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 0, 0, 0);
                });
                break;
            case INTAKING:
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 15, 255, 255);
                });
                break;
            case HUNG:
                // TODO: progress
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 150, 255, 255);
                });
                break;
            case FIRE_READY:
                // TODO: strobe
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 55, 255, 255);
                });
                break;
            case HOME:
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 0, 0, 255);
                });
                break;
            case TRENCH_SAFE:
                m_buffer.forEach((index, r, g, b) -> {
                    m_buffer.setHSV(index, 140, 255, 255);
                });
                break;
            default: break;
        }

        m_led.setData(m_buffer);
    }

    public Command set(Constants.BlinkyBlinky.Mode mode) {
        return new CommandBuilder()
            .onExecute(() -> {
                if (mode != m_currentMode) {
                    m_currentMode = mode;
                }
            })
            .isFinished(true);
    }
}
