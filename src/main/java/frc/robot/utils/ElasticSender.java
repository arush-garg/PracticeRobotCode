package frc.robot.utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ElasticSender {
    private ShuffleboardTab tab;
    private boolean update;
    private ArrayList<ElasticItem> entries = new ArrayList<ElasticItem>();
    private int DEFAULT_LENGTH = 1;
    private int DEFAULT_WIDTH = 1;

    private int DEFAULT_X = 0;
    private int DEFAULT_Y = 0;

    private class ElasticItem {
        public GenericEntry entry;
        public Object value;

        public ElasticItem(GenericEntry entry, Object value) {
            this.entry = entry;
            this.value = value;
        }
    }

    public ElasticSender(String tabName, boolean update) {
        tab = Shuffleboard.getTab(tabName);
        this.update = update;
    }

    public void update() {
        if (update) {
            for (ElasticItem item : entries) {
                if (item.value.getClass() == TalonFXConfiguration.class) {
                    TalonFXConfiguration cfg = (TalonFXConfiguration) item.value;
                    cfg.Slot0.kP = item.entry.get().getDoubleArray()[0];
                    cfg.Slot0.kI = item.entry.get().getDoubleArray()[1];
                    cfg.Slot0.kD = item.entry.get().getDoubleArray()[2];
                    cfg.Slot0.kV = item.entry.get().getDoubleArray()[3];
                    cfg.Slot0.kA = item.entry.get().getDoubleArray()[4];
                    cfg.Slot0.kS = item.entry.get().getDoubleArray()[5];
                    cfg.Slot0.kG = item.entry.get().getDoubleArray()[6];
                    item.value = cfg;
                } else {
                    item.value = item.entry.get();
                }
            }
        }
        for (ElasticItem item : entries) {
            if (item.value.getClass() == TalonFXConfiguration.class) {
                TalonFXConfiguration cfg = (TalonFXConfiguration) item.value;
                Supplier<double[]> supplier = () -> new double[] { cfg.Slot0.kP, cfg.Slot0.kI, cfg.Slot0.kD,
                        cfg.Slot0.kV, cfg.Slot0.kA, cfg.Slot0.kS, cfg.Slot0.kG };
                item.entry.setValue(supplier);
            } else {
                item.entry.setValue(item.value);
            }
        }
    }

    public void add(String name, String value) {
        GenericEntry newEntry = tab.add(name, value).withWidget(BuiltInWidgets.kTextView)
                .withPosition(DEFAULT_X, DEFAULT_Y).withSize(DEFAULT_LENGTH, DEFAULT_WIDTH).getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }

    public void add(String name, TalonFXConfiguration cfg) {
        double[] value = { cfg.Slot0.kP, cfg.Slot0.kI, cfg.Slot0.kD, cfg.Slot0.kV, cfg.Slot0.kA, cfg.Slot0.kS,
                cfg.Slot0.kG };
        Supplier<double[]> supplier = () -> value;

        // GenericEntry newEntry = tab.addDoubleArray(name,
        // supplier).withWidget(BuiltInWidgets.kTextView).withPosition(DEFAULT_X,
        // DEFAULT_Y).withSize(DEFAULT_LENGTH, DEFAULT_WIDTH);
        // entries.add(new ElasticItem(newEntry, cfg));
    }

    public void add(String name, Double value) {
        GenericEntry newEntry = tab.add(name, value).withWidget(BuiltInWidgets.kTextView)
                .withPosition(DEFAULT_X, DEFAULT_Y).withSize(DEFAULT_LENGTH, DEFAULT_WIDTH).getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }

    public void add(String name, Boolean value) {
        GenericEntry newEntry = tab.add(name, value).withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(DEFAULT_X, DEFAULT_Y).withSize(DEFAULT_LENGTH, DEFAULT_WIDTH).getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }

    public void addButton(String name, Runnable command) {
        GenericEntry newEntry = tab.add(name, command).withWidget(BuiltInWidgets.kCommand)
                .withPosition(DEFAULT_X, DEFAULT_Y).withSize(DEFAULT_LENGTH, DEFAULT_WIDTH).getEntry();
        entries.add(new ElasticItem(newEntry, command));
    }

    public <T> void add(String name, T value, BuiltInWidgets widget) {
        GenericEntry newEntry = tab.add(name, value).withWidget(widget).withPosition(DEFAULT_X, DEFAULT_Y)
                .withSize(DEFAULT_LENGTH, DEFAULT_WIDTH).getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }

    public void add(String name, String value, int x, int y, int length, int width) {
        GenericEntry newEntry = tab.add(name, value).withWidget(BuiltInWidgets.kTextView).withPosition(x, y)
                .withSize(length, width).getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }

    public void add(String name, Double value, int x, int y, int length, int width) {
        GenericEntry newEntry = tab.add(name, value).withWidget(BuiltInWidgets.kTextView).withPosition(x, y)
                .withSize(length, width).getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }

    public void add(String name, Boolean value, int x, int y, int length, int width) {
        GenericEntry newEntry = tab.add(name, value).withWidget(BuiltInWidgets.kBooleanBox).withPosition(x, y)
                .withSize(length, width).getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }

    public void addButton(String name, Runnable command, int x, int y, int length, int width) {
        GenericEntry newEntry = tab.add(name, command).withWidget(BuiltInWidgets.kCommand).withPosition(x, y)
                .withSize(length, width).getEntry();
        entries.add(new ElasticItem(newEntry, command));
    }

    public <T> void add(String name, T value, BuiltInWidgets widget, int x, int y, int length, int width) {
        GenericEntry newEntry = tab.add(name, value).withWidget(widget).withPosition(x, y).withSize(length, width)
                .getEntry();
        entries.add(new ElasticItem(newEntry, value));
    }
}