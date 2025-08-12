package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import java.util.ArrayList;
import java.util.EventListener;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TestLayout {

    public interface PeriodicListener<T> extends EventListener {
        void periodic(T value);
    }

    public class PeriodicNotifier<T> {
        private final List<PeriodicListener<T>> listeners = new ArrayList<>();

        public void subscribe(PeriodicListener<T> listener) {
            listeners.add(listener);
        }

        protected void invoke(T value) {
            for (PeriodicListener<T> listener : listeners) {
                listener.periodic(value);
            }
        }
    }

    private final Map<GenericEntry, PeriodicNotifier<Double>> sliderMap = new HashMap<>();
    private final Map<GenericEntry, PeriodicNotifier<Boolean>> toggleMap = new HashMap<>();
    private final Map<GenericEntry, PeriodicNotifier<Rotation2d>> headingMap = new HashMap<>();
    private final Map<String, ComplexWidget> motorMap = new HashMap<>();

    public <T extends MotorController & Sendable> void addMotor(String name, T motor) {
        Optional<ShuffleboardComponent<?>> existingWidget = getTab().getComponents().stream()
                .filter((v) -> v.getTitle().equals(name))
                .findFirst();
        if (!existingWidget.isPresent()) {
            ComplexWidget entry = getTab().add(name, motor).withWidget(BuiltInWidgets.kMotorController);
            motorMap.put(name, entry);
        }
    }

    public void addBoolean(String name, BooleanSupplier value) {
        Optional<ShuffleboardComponent<?>> existingWidget = getTab().getComponents().stream()
                .filter((v) -> v.getTitle().equals(name))
                .findFirst();
        if (!existingWidget.isPresent()) {
            getTab().addBoolean(name, value);
        } else {
            GenericEntry entry = ((SimpleWidget) existingWidget.get()).getEntry();
            entry.setValue(value);
        }
    }

    public PeriodicNotifier<Boolean> addToggle(String name, BooleanSupplier value) {
        Optional<ShuffleboardComponent<?>> existingWidget = getTab().getComponents().stream()
                .filter((v) -> v.getTitle().equals(name))
                .findFirst();

        if (existingWidget.isPresent()) {
            GenericEntry entry = ((SimpleWidget) existingWidget.get()).getEntry();
            return toggleMap.get(entry);
        }

        GenericEntry entry = getTab().add(name, value)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        PeriodicNotifier<Boolean> notifier = new PeriodicNotifier<>();
        toggleMap.put(entry, notifier);
        return notifier;
    }

    public PeriodicNotifier<Double> addSlider(String name, double value, double min, double max) {
        Optional<ShuffleboardComponent<?>> existingWidget = getTab().getComponents().stream()
                .filter((v) -> v.getTitle().equals(name))
                .findFirst();
        if (existingWidget.isPresent()) {
            GenericEntry entry = ((SimpleWidget) existingWidget.get()).getEntry();
            // entry.setDouble(value); <- Ostrich algorithm (look it up)
            return sliderMap.get(entry);
        }

        GenericEntry entry = getTab().add(name, value)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", min, "max", max))
                .getEntry();

        PeriodicNotifier<Double> notifier = new PeriodicNotifier<>();
        sliderMap.put(entry, notifier);
        return notifier;
    }

    public PeriodicNotifier<Rotation2d> addHeading(String name, Rotation2d rotation) {
        Optional<ShuffleboardComponent<?>> existingWidget = getTab().getComponents().stream()
                .filter((v) -> v.getTitle().equals(name))
                .findFirst();
        if (existingWidget.isPresent()) {
            GenericEntry entry = ((SimpleWidget) existingWidget.get()).getEntry();
            entry.setDouble(rotation.getRadians());
            return headingMap.get(entry);
        }

        GenericEntry entry = getTab().add(name, rotation.getRadians())
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", Math.PI * 2))
                .getEntry();

        PeriodicNotifier<Rotation2d> notifier = new PeriodicNotifier<>();
        headingMap.put(entry, notifier);
        return notifier;
    }

    public void addNumber(String name, DoubleSupplier value) {
        Optional<ShuffleboardComponent<?>> existingWidget = getTab().getComponents().stream()
                .filter((v) -> v.getTitle().equals(name))
                .findFirst();
        if (!existingWidget.isPresent()) {
            getTab().addDouble(name, value);
        } else {
            GenericEntry entry = ((SimpleWidget) existingWidget.get()).getEntry();
            entry.setValue(value);
        }
    }

    public void testPeriodic() {
        for (GenericEntry key : sliderMap.keySet()) {
            sliderMap.get(key).invoke(key.getDouble(0));
        }

        for (GenericEntry key : headingMap.keySet()) {
            headingMap.get(key).invoke(new Rotation2d(key.getDouble(0)));
        }
    }

    public ShuffleboardTab getTab() {
        return Shuffleboard.getTab("Test");
    }
}
