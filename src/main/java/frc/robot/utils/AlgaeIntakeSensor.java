package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.testingdashboard.SubsystemBase;

public class AlgaeIntakeSensor {
    private DigitalInput m_sensor;
    private boolean m_state;

    public AlgaeIntakeSensor(int dioPort, SubsystemBase subsystem) {
        m_sensor = new DigitalInput(dioPort);
        m_state = false;
    }

    public void update() {
        m_state = m_sensor.get();
    }

    public boolean seesAlgae() {
        return m_state;
    }
}
