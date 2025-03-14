package frc.robot.ElasticSender;

import edu.wpi.first.networktables.NetworkTableEntry;

public class ElasticItem {
    String key;
    Object value;
    NetworkTableEntry entry;
    boolean editable;

    public ElasticItem(String key, Object value, NetworkTableEntry entry, boolean edit) {
        this.key = key;
        this.value = value;
        this.entry = entry;
        this.editable = edit;
        entry.setValue(value);
    }
}