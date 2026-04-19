package frc.robot.orchestration;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Broken;
import frc.util.Constants;
import frc.util.Helpers;

public class BlinkyBlinkyOrchestrator implements AutoCloseable {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;

    public double batteryVoltage = 0;

    public BlinkyBlinkyOrchestrator(Robot robot) {
        m_led = new AddressableLED(Constants.IOMap.BlinkyBlinky.kPWMport);

        m_buffer = new AddressableLEDBuffer(Constants.BlinkyBlinky.kLength);
        m_led.setLength(m_buffer.getLength());

        m_led.setData(m_buffer);
        m_led.start();

        SmartDashboard.putNumber("LEDs / Brightness", 1d);
    }

    @Override
    public void close() {
        m_buffer.forEach((index, r, g, b) -> {
            m_buffer.setHSV(index, index % 2 == 0 ? 30 : 60, 255, percentToV(1));
        });

        m_led.setData(m_buffer);
    }

    public void bootStatus(int status) {
        m_buffer.forEach((index, r, g, b) -> {
            if (status == 0) { // Pre-subsytems
                m_buffer.setHSV(index, 0, 255, percentToV(index % 2 == 0 ? 1 : 0));
            } else if (status == 1) { // Pre-orchestration
                m_buffer.setHSV(index, 30, 255, percentToV(index % 2 == 0 ? 1 : 0));
            } else if (status == 2) { // Pre-controllers / Pre-defaults
                m_buffer.setHSV(index, 90, 255, percentToV(index % 2 == 0 ? 1 : 0));
            } else if (status == 3) { // Pre-auto / Extra
                m_buffer.setHSV(index, 140, 255, percentToV(index % 2 == 0 ? 1 : 0));
            } else if (status == 4) { // End of construction
                m_buffer.setHSV(index, 60, 255, percentToV(index % 2 == 0 ? 1 : 0));
            }
        });

        m_led.setData(m_buffer);
    }

    private int percentToV(double percent) {
        return (int)Math.floor(255 * percent);
    }

    private int p1Score = 0;
    private int p2Score = 0;

    private int p1ScoreOffset = 27;
    private int p2ScoreOffset = 35;

    private double p1y = 4.5;
    private double p2y = 4.5;

    private double ballX = 2.5;
    private double ballY = 4.5;

    private double ballVX = 0;
    private double ballVY = 0;

    private int coordToIndex(double x, double y) {
        int roundedX = (int)x;
        int roundedY = (int)y;

        int index = 0;

        switch (roundedX) {
            case 0:
                index = 8 - roundedY;
                break;
            case 1:
                index = 9 + roundedY;
                break;
            case 2:
                index = 26 - roundedY;
                break;
            case 3:
                index = 45 + roundedY;
                break;
            case 4:
                index = 62 - roundedY;
                break;
            case 5:
                index = 63 + roundedY;
                break;
        }

        return index;
    }

    public void startRound() {
        Random rand = new Random();
        ballVX = (rand.nextDouble() - .5) / 20;
        ballVY = (rand.nextDouble() - .5) / 20;
        ballX = 2.5;
        ballY = 4.5;
    }

    public void restart() {
        p1Score = 0;
        p2Score = 0;
        startRound();
    }

    private void createBuffer(boolean p1Up, boolean p1Down, boolean p2Up, boolean p2Down) {
        if (p1Up) {
            p1y = Helpers.clamp(p1y + .1, 0, 8);
        }
        if (p1Down) {
            p1y = Helpers.clamp(p1y - .1, 0, 8);
        }
        if (p2Up) {
            p2y = Helpers.clamp(p2y + .1, 0, 8);
        }
        if (p2Down) {
            p2y = Helpers.clamp(p2y - .1, 0, 8);
        }

        // Clear All
        m_buffer.forEach((index, r, g, b) -> {
            m_buffer.setHSV(index, 0, 0, 0);
        });

        if (p1Score > 8) {
            m_buffer.setHSV(4, 60, 255, 255);
            m_buffer.setHSV(11, 60, 255, 255);
            m_buffer.setHSV(15, 60, 255, 255);
            m_buffer.setHSV(22, 60, 255, 255);
        } else if (p2Score > 8) {
            m_buffer.setHSV(71 - 4, 120, 255, 255);
            m_buffer.setHSV(71 - 11, 120, 255, 255);
            m_buffer.setHSV(71 - 15, 120, 255, 255);
            m_buffer.setHSV(71 - 22, 120, 255, 255);
        } else {
            ballX = ballX + ballVX;
            ballY = ballY + ballVY;

            if (ballY < 0) {
                ballVY *= -1;
                ballY = 0;
            }

            if (ballY > 8) {
                ballVY *= -1;
                ballY = 8;
            }

            if (ballX < 0) {
                double dY = Math.abs(ballY - p1y);
                if (dY < 1.5) {
                    ballVX *= -1;
                    ballX = 1;
                } else {
                    p2Score++;
                    startRound();
                }
            }
            if (ballX > 5) {
                double dY = Math.abs(ballY - p2y);
                if (dY < 1.5) {
                    ballVX *= -1;
                    ballX = 4;
                } else {
                    p1Score++;
                    startRound();
                }
            }
            // Players
            m_buffer.setHSV(coordToIndex(0, p1y), 0, 0, 255);
            m_buffer.setHSV(coordToIndex(5, p2y), 0, 0, 255);

            // Score
            m_buffer.setHSV(p1Score + p1ScoreOffset, 0, 0, 255);
            m_buffer.setHSV(p2Score + p2ScoreOffset, 0, 0, 255);

            // Ball
            m_buffer.setHSV(coordToIndex(ballX, ballY), 0, 0, 255);
        }
    }

    public void sparkle(boolean p1Up, boolean p1Down, boolean p2Up, boolean p2Down) {
        if (!Broken.blinkyBlinkyDisableStatus()) {
            createBuffer(p1Up, p1Down, p2Up, p2Down);
        } else {
            m_buffer.forEach((index, r, g, b) -> {
                m_buffer.setHSV(index, 0, 0, 0);
            });
        }

        m_led.setData(m_buffer);
    }
}
