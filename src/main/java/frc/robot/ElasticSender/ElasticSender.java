package frc.robot.ElasticSender;

import java.util.ArrayList;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElasticSender.TalonFXConfigItem.*;


public class ElasticSender {
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
    private NetworkTable m_tab; // This is the table that data will be sent to
    private ArrayList<ElasticItem> m_items;
    private ArrayList<ElasticItem> m_buttons;
    private ArrayList<TalonFXConfigItem> m_configItems;
    private int cycles = 0;
    private boolean enabled;

    public ElasticSender(String name, boolean enabled) {
        m_tab = inst.getTable(name);
        this.enabled = enabled;
        m_items = new ArrayList<ElasticItem>();
        m_buttons = new ArrayList<ElasticItem>();
        m_configItems = new ArrayList<TalonFXConfigItem>();
    }

    public void put(String name, Object value, boolean edit) { //Add an object
        if(!enabled) {
            return;
        }

        boolean found = false;
        for (ElasticItem item : m_items) {
            if (item.key.equals(name)) {
                if(!item.value.equals(value)) {
                    item.value = value;
                    item.entry.setValue(value);
                }
                found = true;
                break;
            }
        }
        if (!found) {
            ElasticItem item = new ElasticItem(name, value, m_tab.getEntry(name), edit);
            m_items.add(item);
        }
    }

    public double getNumber(String name) {
        for (ElasticItem item : m_items) {
            if (item.key.equals(name)) {
                return item.entry.getDouble(0.0);
            }
        }

        return 0.0;
    }

    public boolean getBoolean(String name) {
        for (ElasticItem item : m_items) {
            if (item.key.equals(name)) {
                return (boolean)(item.entry.getBoolean(false));
            }
        }

        return false;
    }

    public void addConfig(String name, TalonFXConfiguration config, TalonFXConfigSlot slot, TalonFXConfigItem.TalonFXConfigType type) {
        if(!enabled) {
            return;
        }

        switch(type) {
            case PID_ONLY:
                addPID(name, config, slot);
                break;
            
            case FF_ONLY:
                addFeedForward(name, config, slot);
                break;
            
            case ALL:
                addPID(name, config, slot);
                addFeedForward(name, config, slot);
                break;
        }
    }

    private void addPID(String name, TalonFXConfiguration config, TalonFXConfigSlot slot) {
        switch(slot) {
            case SLOT_0: //Should the names of the items be name or name + slot + " kP"
                m_configItems.add(new TalonFXConfigItem(name + slot + " kP", config.Slot0.kP, m_tab.getEntry(name + slot + " kP"), config, slot, ConfigParameter.KP));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kI", config.Slot0.kI, m_tab.getEntry(name + slot + " kI"), config, slot, ConfigParameter.KI));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kD", config.Slot0.kD, m_tab.getEntry(name + slot + " kD"), config, slot, ConfigParameter.KD));
                break;
            case SLOT_1:
                m_configItems.add(new TalonFXConfigItem(name + slot + " kP", config.Slot1.kP, m_tab.getEntry(name + slot + " kP"), config, slot, ConfigParameter.KP));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kI", config.Slot1.kI, m_tab.getEntry(name + slot + " kI"), config, slot, ConfigParameter.KI));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kD", config.Slot1.kD, m_tab.getEntry(name + slot + " kD"), config, slot, ConfigParameter.KD));
                break;
            case SLOT_2:
                m_configItems.add(new TalonFXConfigItem(name + slot + " kP", config.Slot2.kP, m_tab.getEntry(name + slot + " kP"), config, slot, ConfigParameter.KP));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kI", config.Slot2.kI, m_tab.getEntry(name + slot + " kI"), config, slot, ConfigParameter.KI));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kD", config.Slot2.kD, m_tab.getEntry(name + slot + " kD"), config, slot, ConfigParameter.KD));
                break;
        }

    }

    private void addFeedForward(String name, TalonFXConfiguration config, TalonFXConfigSlot slot) {
        switch(slot) {
            case SLOT_0:
                m_configItems.add(new TalonFXConfigItem(name + slot + " kS", config.Slot0.kS, m_tab.getEntry(name + slot + " kS"), config, slot, ConfigParameter.KS));                
                m_configItems.add(new TalonFXConfigItem(name + slot + " kV", config.Slot0.kV, m_tab.getEntry(name + slot + " kV"), config, slot, ConfigParameter.KV));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kA", config.Slot0.kA, m_tab.getEntry(name + slot + " kA"), config, slot, ConfigParameter.KA));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kG", config.Slot0.kG, m_tab.getEntry(name + slot + " kG"), config, slot, ConfigParameter.KG));
                break;
            case SLOT_1:
                m_configItems.add(new TalonFXConfigItem(name + slot + " kS", config.Slot1.kS, m_tab.getEntry(name + slot + " kS"), config, slot, ConfigParameter.KS));                
                m_configItems.add(new TalonFXConfigItem(name + slot + " kV", config.Slot1.kV, m_tab.getEntry(name + slot + " kV"), config, slot, ConfigParameter.KV));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kA", config.Slot1.kA, m_tab.getEntry(name + slot + " kA"), config, slot, ConfigParameter.KA));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kG", config.Slot1.kG, m_tab.getEntry(name + slot + " kG"), config, slot, ConfigParameter.KG));
                break;
            case SLOT_2:
                m_configItems.add(new TalonFXConfigItem(name + slot + " kS", config.Slot2.kS, m_tab.getEntry(name + slot + " kS"), config, slot, ConfigParameter.KS));                
                m_configItems.add(new TalonFXConfigItem(name + slot + " kV", config.Slot2.kV, m_tab.getEntry(name + slot + " kV"), config, slot, ConfigParameter.KV));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kA", config.Slot2.kA, m_tab.getEntry(name + slot + " kA"), config, slot, ConfigParameter.KA));
                m_configItems.add(new TalonFXConfigItem(name + slot + " kG", config.Slot2.kG, m_tab.getEntry(name + slot + " kG"), config, slot, ConfigParameter.KG));
                break;
        }
    }

    public void addButton(String name, Command action) {
        if(!enabled) {
            return;
        }
        NetworkTableEntry entry = m_tab.getEntry(name);
        entry.setBoolean(false);
        m_buttons.add(new ElasticItem(name, action, entry, true));
    }

    /**
     * This function should be called periodically. It will run once every 3 cycles (roughtly every 0.06 seconds)
     * Returns <code>true</code> if <code>TalonFXConfig</code> has been changed
     */
    public boolean periodic() {
        if(!enabled) {
            return false;
        }

        boolean updated = false;

        if(cycles == 0) {
            for (ElasticItem button : m_buttons) {
                if(button.entry.getBoolean(false)) {
                    ((Command) button.value).schedule();                
                    button.entry.setBoolean(false);
                }
            }

            for(TalonFXConfigItem item : m_configItems) {
                if(item.update()) {
                    updated = true;
                }
            }

            for(ElasticItem item : m_items) {
                if(item.value.equals(item.entry.getValue())) {
                    continue;
                }

                if(item.editable) {
                    item.value = item.entry.getValue();
                }
            }
        }

        cycles++;

        if(cycles == 3) { //Makes the function run every 0.06 seconds
            cycles = 0;
        }

        return updated;
    }
}
