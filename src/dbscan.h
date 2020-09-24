#pragma once

#ifndef DBSCAN_H_MODIFIED_MARIUS
#define DBSCAN_H_MODIFIED_MARIUS

#include <vector>
#include <cmath>
#include <iostream>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3


typedef struct Pointdb_ {
    float x, y, z;  // X, Y, Z position
    int clusterID;  // clustered ID
} Pointdb;

class DBSCAN {
public:
    DBSCAN(unsigned int minPts, float eps, const std::vector<Pointdb>& points) {
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
        m_max_clusterID = -1;
    }

    ~DBSCAN() = default;

    int run();

    std::vector<int> calculateCluster(Pointdb point);

    int expandCluster(Pointdb point, int clusterID);

    inline double calculateDistance(Pointdb pointCore, Pointdb pointTarget);

    int getTotalPointdbSize() const { return m_pointSize; }

    int getMinimumClusterSize() const { return m_minPoints; }

    float getEpsilonSize() const { return m_epsilon; }

    int getMaxClusterID() const { return m_max_clusterID; }

    /// User implemented by Marius Lindegaard team 2020
    const std::vector<Pointdb>& getPoints() const { return m_points; }


private:
    std::vector<Pointdb> m_points;
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
    /// User implemented by Marius Lindegaard
    int m_max_clusterID;
};


#endif //DBSCAN_H_MODIFIED_MARIUS
