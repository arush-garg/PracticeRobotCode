package frc.robot.ElasticSender;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;

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
        if (!(value instanceof Command)) {
            entry.setValue(value);
        } else {
            entry.setValue(false);
        }
    }
}