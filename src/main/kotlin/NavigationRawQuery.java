import java.util.HashMap;

public class NavigationRawQuery extends HashMap<String, NavigationRawQuery.Query> {

    static public class Query {
        Params params;
        ServoSettings presentControl;
        Geometry geometry;
    }

    static public class Params {
        String params;
    }

    static public class ServoSettings {
        HashMap<String, Integer> servoSettings;
    }

    static public class Geometry {
        String targetPoint;
        String source;
    }
}
