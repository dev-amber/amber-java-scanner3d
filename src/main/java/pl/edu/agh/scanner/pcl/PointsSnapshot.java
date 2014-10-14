package pl.edu.agh.scanner.pcl;

public class PointsSnapshot {

    public final double[] objectPoints;

    public final double[] backgroundPoints;

    public PointsSnapshot(Point3d[] objectPoints, Point3d[] backgroundPoints) {
        this.objectPoints = new double[objectPoints.length * 3];
        this.backgroundPoints = new double[objectPoints.length * 3];

        int index = 0;
        for (int i = 0; i < objectPoints.length; i++) {
            Point3d point = objectPoints[i];

            this.objectPoints[index++] = point.x;
            this.objectPoints[index++] = point.y;
            this.objectPoints[index++] = point.z;
        }

        index = 0;
        for (int i = 0; i < backgroundPoints.length; i++) {
            Point3d point = backgroundPoints[i];

            this.backgroundPoints[index++] = point.x;
            this.backgroundPoints[index++] = point.y;
            this.backgroundPoints[index++] = point.z;
        }
    }
}
