package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class LED extends SubsystemBase {
    
    private AddressableLED m_led = new AddressableLED(0);
    
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(44);
    /*private AddressableLEDBufferView m_leftLED = m_ledBuffer.createView(1, 14);
    private AddressableLEDBufferView m_rightLED = m_ledBuffer.createView(15, 29);
    private AddressableLEDBufferView m_topLED = m_ledBuffer.createView(30, 43);*/

    private LEDPattern hasCoralLED = LEDPattern.solid(Color.kGreen);
    private LEDPattern noCoralLED = LEDPattern.solid(Color.kBlue);

    private LEDPattern coralModeLED = LEDPattern.solid(Color.kWhite);
    private LEDPattern algaeModeLED = LEDPattern.solid(Color.kRed);

    public LED() {
        //m_led.setLength(m_ledBuffer.getLength());
        //m_led.close();

        //CommandScheduler.getInstance().setDefaultCommand(this, runPattern(algaeModeLED).ignoringDisable(true));
        //this.setDefaultCommand(runPattern(algaeModeLED).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        //m_led.setData(m_ledBuffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return Commands.run(() -> {pattern.applyTo(m_ledBuffer);}, this);
    }

}
