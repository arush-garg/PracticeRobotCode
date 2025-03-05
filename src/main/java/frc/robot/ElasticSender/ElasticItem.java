package frc.robot.ElasticSender;

import edu.wpi.first.networktables.NetworkTableEntry;

public class ElasticItem {
    String key;
    Object value;
    NetworkTableEntry entry;

    public ElasticItem(String key, Object value, NetworkTableEntry entry) {
        this.key = key;
        this.value = value;
        this.entry = entry;
        entry.setValue(value);
    }
}