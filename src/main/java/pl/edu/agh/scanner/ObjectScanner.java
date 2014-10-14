package pl.edu.agh.scanner;

import pl.edu.agh.amber.common.AmberClient;
import pl.edu.agh.amber.hitec.HitecProxy;
import pl.edu.agh.amber.hokuyo.AngleRange;
import pl.edu.agh.amber.hokuyo.HokuyoProxy;
import pl.edu.agh.amber.hokuyo.Scan;
import pl.edu.agh.amber.roboclaw.RoboclawProxy;
import pl.edu.agh.scanner.pcl.PCLWrapper;
import pl.edu.agh.scanner.pcl.Point3d;
import pl.edu.agh.scanner.pcl.PointsSnapshot;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Logger;

public class ObjectScanner {

    private final static double DEFAULT_TRIMMED_DISTANCE = 1000;

    private static final int MAX_VERTICAL_ANGLE = 45;

    private static final int SAMPLE_COUNT = 3;

    private static final int VIEWS_COUNT = 100;

    private final RoboclawProxy roboclawProxy;

    private final HokuyoProxy hokuyoProxy;

    private final HitecProxy hitecProxy;

    private final Logger logger;

    private final double trimmedDistance;

    static {
        System.loadLibrary("joinedpdccreator");
    }

    public ObjectScanner(double trimmedDistance) {
        this.trimmedDistance = trimmedDistance;
        roboclawProxy = new RoboclawProxy(getRawClient(), 0);
        hokuyoProxy = new HokuyoProxy(getRawClient(), 0);
        hitecProxy = new HitecProxy(getRawClient(), 0);

        logger = Logger.getLogger("HokuyoProxy");
    }

    public static void main(String args[]) {
        System.load("/home/freakybit/agh/sria/workspace/java-3d-scanner/jni/libjoinedpdccreator.so");
        (new ObjectScanner(DEFAULT_TRIMMED_DISTANCE)).run(MAX_VERTICAL_ANGLE,
                SAMPLE_COUNT, VIEWS_COUNT);
    }

    public void run(int toAngle, int sampleCount, int viewsCount) {

        PointsSnapshot[] snapshots = new PointsSnapshot[viewsCount];

        for (int i = 0; i < viewsCount; i++) {
            Scan[] frontalScan = getFullScan(toAngle, sampleCount);
            PointsSnapshot snapshot = getPoints(frontalScan);
            snapshots[i] = snapshot;

            moveAroundObject();
        }

//		Point3d[] objI = new Point3d[] {new Point3d(10, 10, 0), new Point3d(10, 20, 0)};
//		Point3d[] bcgI = new Point3d[] {new Point3d(20, 10, 0), new Point3d(20, 20, 0)};
//		PointsSnapshot snapI = new PointsSnapshot(objI, bcgI);
//		
//		Point3d[] objII = new Point3d[] {new Point3d(10, 10, 1), new Point3d(10, 20, 1)};
//		Point3d[] bcgII = new Point3d[] {new Point3d(20, 10, 1), new Point3d(20, 20, 1)};
//		PointsSnapshot snapII = new PointsSnapshot(objII, bcgII);
//		
//		snapshots[0] = snapI;
//		snapshots[1] = snapII;


        (new PCLWrapper()).createJoinedPointCloud(snapshots);
    }

    private void moveAroundObject() {

    }

    public PointsSnapshot getPoints(Scan[] scans) {
        AngleRange range = scans[0].getAngleRange();

        List<Point3d> objectPoints = new LinkedList<Point3d>();
        List<Point3d> backgroundPoints = new LinkedList<Point3d>();

        for (int i = 0; i < scans.length; i++) {
            double verticalAngle = Math.toRadians(i);
            int distances[] = getDistances(scans[i]);

            for (int j = 0; j < distances.length; j++) {
                double horizontalAngle = Math.toRadians(range.getAngle(j));

                int distance = distances[j];
                double baseDistance = distance * Math.cos(verticalAngle);

                if (baseDistance > 0) {
                    double x = baseDistance * Math.cos(horizontalAngle);
                    double y = distance * Math.sin(verticalAngle);
                    double z = baseDistance * Math.sin(horizontalAngle);

                    Point3d point = new Point3d(x, y, z);

                    if (baseDistance < trimmedDistance) {
                        objectPoints.add(point);
                    } else {
                        backgroundPoints.add(point);
                    }
                }
            }
        }

        Point3d[] objectPointsArray = objectPoints.toArray(new Point3d[objectPoints.size()]);
        Point3d[] backgroundPointsArray = backgroundPoints.toArray(new Point3d[backgroundPoints.size()]);

        return new PointsSnapshot(objectPointsArray, backgroundPointsArray);
    }

    public AmberClient getRawClient() {
        try {
            AmberClient client = new AmberClient("127.0.0.1", 26233);
            return client;
        } catch (IOException e) {
            logger.severe("Unable to connect to robot.");
            return null;
        }
    }

    private Scan getAverageScan(int sampleCount) {
        int[][] samples = new int[sampleCount][];
        int[] averageDistances = null;
        AngleRange range = null;

        for (int i = 0; i < sampleCount; i++) {
            Scan scan = getScan();
            samples[i] = getDistances(scan);

            if (samples[i] != null) {
                averageDistances = new int[samples[i].length];
                range = scan.getAngleRange();
            }
        }

        if (averageDistances == null) {
            return null;
        }

        for (int i = 0; i < averageDistances.length; i++) {
            int sumDistance = 0;
            int validCount = 0;

            for (int j = 0; j < sampleCount; j++) {
                double value = samples[j][i];

                if (value > 0) {
                    sumDistance += value;
                    validCount++;
                }
            }

            if (validCount > 0) {
                averageDistances[i] = sumDistance / validCount;
            }
        }

        return new Scan(averageDistances, range);
    }

    public Scan[] getFullScan(int toAngle, int sampleCount) {
        Scan[] allScans = new Scan[toAngle];

        for (int i = 0; i < toAngle; i++) {
            setAngle(i);
            allScans[i] = getAverageScan(sampleCount);
        }

        return allScans;
    }

    public void setAngle(int angle) {
        hitecProxy.setAngle(0, angle);
    }

    public Scan getScan() {
        try {
            return hokuyoProxy.getSingleScan();
        } catch (IOException e) {
            logger.severe("Couldn't get scan from Hokuyo device.");
            return null;
        }
    }

    private int[] getDistances(Scan scan) {
        try {
            return scan.getDistances();
        } catch (Exception e) {
            logger.severe("Couldn't get distances from scan data.");
            return null;
        }
    }
}
