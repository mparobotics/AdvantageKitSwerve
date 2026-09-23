package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;
public class TunableVariable{

    public static class TunableDouble {
        private String key;
        private double lastValue;

        public TunableDouble(String key, double defaultValue) {
            this.key = key;
            Preferences.initDouble(key, defaultValue);
            lastValue = Preferences.getDouble(key, defaultValue);
        }

        public double get() {
            return Preferences.getDouble(key, lastValue);
        }

        public void set(double number) {
            Preferences.setDouble(key, number);
        }

        public boolean hasChanged() {
            double currentValue = get();
            if(lastValue != currentValue) {
                lastValue = currentValue;
                return true;
            }
            else {return false;}
        }
    }

    public static class TunableInt {
        private String key;
        private int lastValue;

        public TunableInt(String key, int defaultValue) {
            key = this.key;
            Preferences.initInt(key, defaultValue);
            lastValue = Preferences.getInt(key, defaultValue);
        }

        public int get() {
            return Preferences.getInt(key, lastValue);
        }

        public boolean hasChanged() {
            int currentValue = get();
            if(lastValue != currentValue) {
                return true;
            }
            else {return false;}
        }

    }

    public static class TunableBoolean {
    private String key;
    private boolean lastValue;

    public TunableBoolean(String key, Boolean defaultValue) {
        key = this.key;
        Preferences.initBoolean(key, defaultValue);
        lastValue = Preferences.getBoolean(key, defaultValue);
    }

    public boolean get() {
        return Preferences.getBoolean(key, lastValue);
    }

    public boolean hasChanged() {
        boolean currentValue = get();
        if(lastValue != currentValue) {
            return true;
        }
        else {return false;}
    }

}

    public static class TunableFloat {
        private String key;
        private float lastValue;

        public TunableFloat(String key, float defaultValue) {
            key = this.key;
            Preferences.initFloat(key, defaultValue);
            lastValue = Preferences.getFloat(key, defaultValue);
        }

        public float get() {
            return Preferences.getFloat(key, lastValue);
        }

        public boolean hasChanged() {
            float currentValue = get();
            if(lastValue != currentValue) {
                return true;
            }
            else {return false;}
        }

    }


}



