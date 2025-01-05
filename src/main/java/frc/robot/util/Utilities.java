package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;

public class Utilities {
    private Utilities() {} // block instantiation

    /**
     * Returns whether the Driverstation is on the red alliance
     * @return true if on the red alliance, false if on the blue alliance or the alliance is not present
     */
    public static boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isEmpty() || (alliance.get() == DriverStation.Alliance.Red);
    }

    public static boolean isValidDouble(double value){
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
    
    /**
     * Run consumer if object is not null, else do nothing
     * @param <T> type of object
     * @param object input object
     * @param objectConsumer consumer to apply to object
     * @return returns optional input object
     */
    public static <T> Optional<T> runIfNotNull(T object, Consumer<T> objectConsumer) {
        if (object != null) {
            objectConsumer.accept(object);
        }
        return Optional.ofNullable(object);
    }

    public static void removeAndCancelDefaultCommand(Subsystem subsystem) {
        runIfNotNull(subsystem.getDefaultCommand(), (Command command) -> {
            subsystem.removeDefaultCommand();
            command.cancel();
        });
    }


    public static Translation2d squareTranslation(Translation2d translation) {
        return new Translation2d(Math.pow(translation.getX(), 2) * Math.signum(translation.getX()), Math.pow(translation.getY(), 2) * Math.signum(translation.getY()));
    }
    
    public static Translation2d cubeTranslation(Translation2d translation) {
        return new Translation2d(Math.pow(translation.getX(), 3), Math.pow(translation.getY(), 3));
    }

    public static double smartPow(double a, double b) {
        return Math.pow(a, b) * Math.signum(a);
    }

    @SafeVarargs
    public static <T> ArrayList<T> arrayListFromParams(T... items) {
        final ArrayList<T> arrayList = new ArrayList<>();
        for (T item : items) {
            arrayList.add(item);
        }
        return arrayList;
    }

    @SafeVarargs
    public static <T> T[] arrayFromParams(T... items) {
        return items;
    }

    @SafeVarargs
    public static <T> void applyConsumerToParams(Consumer<T> consumer, T... items) {
        for (T item : items) {
            consumer.accept(item);
        }
    }

    public static <T, R> ArrayList<R> map(Function<T, R> function, List<T> items) {
        ArrayList<R> output = new ArrayList<>(items.size());
        items.forEach(item -> output.add(function.apply(item)));
        return output;
    }

    @SafeVarargs
    public static <T, R> ArrayList<R> map(Function<T, R> function, T... items) {
        return map(function, Arrays.asList(items));
    }

    public static class WrappedDouble {
        public double val;

        public WrappedDouble(double val) {
            this.val = val;
        }
        
        public WrappedDouble() {
            this(0D);
        }
    }

    public static class WrappedBoolean {
        public boolean val;

        public WrappedBoolean(boolean val) {
            this.val = val;
        }
    }

    public static class WrappedInteger {
        public int val;

        public WrappedInteger(int val) {
            this.val = val;
        }
        
        public WrappedInteger() {
            this(0);
        }
    }
}
