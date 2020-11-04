#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;
namespace dbscan {
    typedef struct Point_ {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
    } Point;

    struct PointSet{
    vector<Point> points;
    int clusterID;
    Point getCenter(){
        double x=0,y=0,z=0;
        int n = 0 ;
        for (auto pnt : points){
            x +=pnt.x;
            y +=pnt.y;
            z +=pnt.z;
            n++;
        }
        x/=n;
        y/=n;
        z/=n;

        Point pnt;
        pnt.x = x;
        pnt.y = y;
        pnt.z = z;
        return pnt;
    }
};


    class DBSCAN {
    public:
        DBSCAN(unsigned int minPts, float eps, vector<Point> points) {
            m_minPoints = minPts;
            m_epsilon = eps;
            m_points = points;
            m_pointSize = points.size();
        }

        ~DBSCAN() {}

        int run();

        vector<int> calculateCluster(Point point);

        int expandCluster(Point point, int clusterID);

        inline double calculateDistance(Point pointCore, Point pointTarget);

        int getTotalPointSize() { return m_pointSize; }

        int getMinimumClusterSize() { return m_minPoints; }

        int getEpsilonSize() { return m_epsilon; }

    public:
        vector<Point> m_points;
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;
    };
}
#endif // DBSCAN_H
