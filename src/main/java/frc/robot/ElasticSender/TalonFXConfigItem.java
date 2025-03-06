package frc.robot.ElasticSender;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;

public class TalonFXConfigItem extends ElasticItem {
    public static enum TalonFXConfigType {
        PID_ONLY,
        FF_ONLY,
        ALL
    }

    public static enum TalonFXConfigSlot {
        SLOT_0,
        SLOT_1,
        SLOT_2
    }

    public static enum ConfigParameter {
        KP,
        KI,
        KD,
        KG,
        KS,
        KV,
        KA
    }

    public TalonFXConfiguration parent;
    public TalonFXConfigSlot slot;
    public ConfigParameter param;


    public TalonFXConfigItem(String key, Object value, NetworkTableEntry entry, TalonFXConfiguration parent, TalonFXConfigSlot slot, ConfigParameter param) {
        super(key, value, entry);
        this.parent = parent;
        this.slot = slot;
        this.param = param;
    }

    public TalonFXConfigItem(String key, Object value, NetworkTableEntry entry, TalonFXConfiguration parent, ConfigParameter param) {
        this(key, value, entry, parent, TalonFXConfigSlot.SLOT_0, param);
    }


    /**
     * Updates the value of the item if it has changed
     * Returns true if the value has changed
     */
    public boolean update() {
        double val = entry.getDouble((double) value);
        if(val == (double) value) {
            return false;
        }
        value = val;

        switch(slot) {
            case SLOT_0:
                switch(param) {
                    case KP:
                        parent.Slot0.kP = (double) value;
                        break;
                    case KI:
                        parent.Slot0.kI = (double) value;
                        break;
                    case KD:
                        parent.Slot0.kD = (double) value;
                        break;
                    case KG:
                        parent.Slot0.kG = (double) value;
                        break;
                    case KS:
                        parent.Slot0.kS = (double) value;
                        break;
                    case KV:
                        parent.Slot0.kV = (double) value;
                        break;
                    case KA:
                        parent.Slot0.kA = (double) value;
                        break;
                }
                break;
            case SLOT_1:
                switch(param) {
                    case KP:
                        parent.Slot1.kP = (double) value;
                        break;
                    case KI:
                        parent.Slot1.kI = (double) value;
                        break;
                    case KD:
                        parent.Slot1.kD = (double) value;
                        break;
                    case KG:
                        parent.Slot1.kG = (double) value;
                        break;
                    case KS:
                        parent.Slot1.kS = (double) value;
                        break;
                    case KV:
                        parent.Slot1.kV = (double) value;
                        break;
                    case KA:
                        parent.Slot1.kA = (double) value;
                        break;
                }
                break;
            case SLOT_2:
                switch(param) {
                    case KP:
                        parent.Slot2.kP = (double) value;
                        break;
                    case KI:
                        parent.Slot2.kI = (double) value;
                        break;
                    case KD:
                        parent.Slot2.kD = (double) value;
                        break;
                    case KG:
                        parent.Slot2.kG = (double) value;
                        break;
                    case KS:
                        parent.Slot2.kS = (double) value;
                        break;
                    case KV:
                        parent.Slot2.kV = (double) value;
                        break;
                    case KA:
                        parent.Slot2.kA = (double) value;
                        break;
                }
                break;
        }

        return true;
    }

}