package frc.robot.subsystems.superstructure;

public enum GPMode {
    Coral,
    Algae;

    @Override
    public String toString() {
        switch (this) {
            case Coral:
            return "Coral";
            case Algae:
            return "Algae";
            default:
            return "Coral";
        }
    }
}
