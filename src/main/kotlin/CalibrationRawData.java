import com.google.gson.Gson;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;

public class CalibrationRawData extends HashMap<String, CalibrationRawData.DataPoint> {

    static public class DataPoint {
        HashMap<String, Integer> servoPositions;
        Data data;
    }

    static public class Data {
        Plane plane;
    }

    static public class Plane {
        String ABCD;
        String center;
        String normal;
    }

}
