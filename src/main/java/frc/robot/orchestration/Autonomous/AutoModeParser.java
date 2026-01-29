package frc.robot.orchestration.Autonomous;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;
import java.util.Set;

import com.google.gson.Gson;

import frc.util.Alert;

public class AutoModeParser {
    private static final String deployDirectory = "/home/lvuser/";

    private AutoModeParser() {} // No instantiation for you :3

    public class Actions {

    }

    public class ActionsOrder {

    }

    public class AutoMode {
        Set<Map<String, String>> steps;
    }

    public class State {
        Actions actions;
        ActionsOrder actions_order;
        Map<String, AutoMode> auto_modes;
    }

    public class Thunderauto {
        State state;
    }

    public static Thunderauto loadThunderauto(String filename) {
        File file = new File(deployDirectory + filename);

        String jsonString = "";
        try (Scanner scanner = new Scanner(file)) {
            while (scanner.hasNextLine()) {
                String data = scanner.nextLine();
                jsonString += data;
            }
        } catch (FileNotFoundException e) {
            Alert.warning(String.format("Thunderauto with name %s not found.", filename));
        }

        return new Gson().fromJson(jsonString, Thunderauto.class);
    }
}
