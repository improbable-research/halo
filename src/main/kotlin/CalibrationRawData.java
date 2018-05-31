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
