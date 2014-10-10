#include <iostream>
#include <vector>
#include <cstdio>
#include <fstream>

#include <jni.h>
#include "pl_edu_agh_scanner_pcl_PCLWrapper.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

struct PCD {
    PointCloud::Ptr cloud;
    std::string fileName;

    PCD() : cloud(new PointCloud) {
    };
};

struct PCDComparator {
    bool operator ()(const PCD& p1, const PCD& p2) {
        return (p1.fileName < p2.fileName);
    }
};

class MyPointRepresentation: public pcl::PointRepresentation<PointNormalT> {
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation() {
        nr_dimensions_ = 4;
    }

    virtual void copyToFloatArray(const PointNormalT &p, float * out) const {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

std::vector<pcl::PointXYZ> getPoints(JNIEnv *env, jobject snapshot, const char* fieldName) {
    jclass snapshotClass = env->GetObjectClass(snapshot);
    jfieldID fieldID = env->GetFieldID(snapshotClass, fieldName, "[D");
    jobject fieldData = env->GetObjectField(snapshot, fieldID);
    jdoubleArray *pointsDoubleArray =
            reinterpret_cast<jdoubleArray*>(&fieldData);

    double *points = env->GetDoubleArrayElements(*pointsDoubleArray, JNI_FALSE);

    int objectPointsCount = env->GetArrayLength(*pointsDoubleArray);
    std::vector<pcl::PointXYZ> pointVector(objectPointsCount + 1);

    jclass point3dClass = NULL;
    jmethodID getX = NULL, getY = NULL, getZ = NULL;
    for (int i = 0; i < objectPointsCount; i += 3) {
        double x = *(points + i);
        double y = *(points + i + 1);
        double z = *(points + i + 2);

        pcl::PointXYZ pointXYZ = pcl::PointXYZ(x, y, z);
        pointVector.push_back(pointXYZ);
    }

    env->ReleaseDoubleArrayElements(*pointsDoubleArray, points, 0);
//    delete (points); //TODO double free?

    return pointVector;
}

std::vector<PCD, Eigen::aligned_allocator<PCD> > getData(JNIEnv * env,
        jobjectArray snapshots) {
    int snapshotsCount = env->GetArrayLength(snapshots);
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;

    for (int i = 0; i < snapshotsCount; i++) {
        jobject snapshot = env->GetObjectArrayElement(snapshots, i);

        std::vector<pcl::PointXYZ> objectPoints = getPoints(env, snapshot,
                "objectPoints");
        std::vector<pcl::PointXYZ> backgroundPoints = getPoints(env, snapshot,
                "backgroundPoints");

        PCD pointCloud;
        for (int j = 0; j < objectPoints.size(); j++) {
            pointCloud.cloud->push_back(objectPoints[j]);
        }

        for (int j = 0; j < backgroundPoints.size(); j++) {
            pointCloud.cloud->push_back(backgroundPoints[j]); //TODO temporary for testing
        }

        char buffer[10];
        sprintf(buffer, "%d.pcd", i);
        pointCloud.fileName = buffer;
        data.push_back(pointCloud);
    }

    return data;
}

void calculateNormals(const PointCloud::Ptr firstCloud,
        const PointCloud::Ptr secondCloud,
        PointCloudWithNormals::Ptr firstWithNormals,
        PointCloudWithNormals::Ptr secondWithNormals) {

    pcl::NormalEstimation<PointT, PointNormalT> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setKSearch(30);

    normalEstimation.setInputCloud(firstCloud);
    normalEstimation.compute(*firstWithNormals);
    pcl::copyPointCloud(*firstCloud, *firstWithNormals);

    normalEstimation.setInputCloud(secondCloud);
    normalEstimation.compute(*secondWithNormals);
    pcl::copyPointCloud(*secondCloud, *secondWithNormals);
}

void initializeAlgorithm(
        pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> &alignAlgorithm,
        PointCloudWithNormals::Ptr firstWithNormals,
        PointCloudWithNormals::Ptr secondWithNormals) {
    double transformationEpsilon = 1e-6;
    float scaleValues[4] = { 1.0, 1.0, 1.0, 1.0 };
    double maxCorrespondenceDistance = 0.1; //TODO Adjust this according to size of scanned objects. Now error margin is set to 10cm.
    int maximumIterations = 2;

    MyPointRepresentation pointRepresentation;
    pointRepresentation.setRescaleValues(scaleValues);

    alignAlgorithm.setTransformationEpsilon(transformationEpsilon);
    alignAlgorithm.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    alignAlgorithm.setPointRepresentation(
            boost::make_shared<const MyPointRepresentation>(
                    pointRepresentation));
    alignAlgorithm.setInputSource(firstWithNormals);
    alignAlgorithm.setInputTarget(secondWithNormals);
    alignAlgorithm.setMaximumIterations(maximumIterations);
}

void transformMatrixes(Eigen::Matrix4f transformationMatrix, pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> alignAlgorithm, PointCloudWithNormals::Ptr firstWithNormals, int iterationCount) {
    Eigen::Matrix4f previous;
    PointCloudWithNormals::Ptr firstWithNormalsCopy = firstWithNormals;

    for (int i = 0; i < iterationCount; ++i) {
        firstWithNormals = firstWithNormalsCopy;

        alignAlgorithm.setInputSource(firstWithNormals);
        alignAlgorithm.align(*firstWithNormalsCopy);

        transformationMatrix = alignAlgorithm.getFinalTransformation()
                * transformationMatrix;

        double difference =
                fabs(
                        (alignAlgorithm.getLastIncrementalTransformation()
                                - previous).sum());
        if (difference < alignAlgorithm.getTransformationEpsilon()) {
            alignAlgorithm.setMaxCorrespondenceDistance(
                    alignAlgorithm.getMaxCorrespondenceDistance() - 0.001);
        }

        previous = alignAlgorithm.getLastIncrementalTransformation();
    }
}

void pairAlign(const PointCloud::Ptr firstCloud,
        const PointCloud::Ptr secondCloud, PointCloud::Ptr resultCloud,
        Eigen::Matrix4f &transformation) {
    PointCloudWithNormals::Ptr firstWithNormals(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr secondWithNormals(new PointCloudWithNormals);

    calculateNormals(firstCloud, secondCloud, firstWithNormals,
            secondWithNormals);

    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> alignAlgorithm;
    initializeAlgorithm(alignAlgorithm, firstWithNormals, secondWithNormals);

    Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity(),
            secondIntoFirst;

    int iterationCount = 30;
    transformMatrixes(transformationMatrix, alignAlgorithm, firstWithNormals, iterationCount);

    secondIntoFirst = transformationMatrix.inverse();
    pcl::transformPointCloud(*secondCloud, *resultCloud, secondIntoFirst);
    *resultCloud += *firstCloud;

    transformation = secondIntoFirst;
}

JNIEXPORT void JNICALL Java_pl_edu_agh_scanner_pcl_PCLWrapper_createJoinedPointCloud(
        JNIEnv * env, jobject thisObj, jobjectArray snapshots) {
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data = getData(env,
            snapshots);

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(),
            pairTransform;
    PointCloud::Ptr source, target;
    PointCloud::Ptr result = data[0].cloud;

    for (size_t i = 1; i < data.size(); i++) {
        source = result;
        target = data[i].cloud;
        result = *(new PointCloud::Ptr(new PointCloud()));

        PointCloud::Ptr temp(new PointCloud);

        pairAlign(source, target, temp, pairTransform);
        pcl::transformPointCloud(*temp, *result, GlobalTransform);
        GlobalTransform = GlobalTransform * pairTransform;
    }
    pcl::io::savePCDFile("result.pcd", *result, false);
}

