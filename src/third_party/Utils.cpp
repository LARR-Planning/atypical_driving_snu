//
// Created by jbs on 20. 5. 14..
//

#include <third_party/Utils.h>

using namespace Eigen; 
double interpolate( vector<double> &xData, vector<double> &yData, double x, bool extrapolate )
{
    int size = xData.size();

    int i = 0;                                                                  // find left end of interval for interpolation
    if ( x >= xData[size - 2] )                                                 // special case: beyond right end
    {
        i = size - 2;
    }
    else
    {
        while ( x > xData[i+1] ) i++;
    }
    double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
    if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
    {
        if ( x < xL ) yR = yL;
        if ( x > xR ) yL = yR;
    }

    double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

    return yL + dydx * ( x - xL );                                              // linear interpolation
}

Vector3d applyTransform(const SE3& Tab, const Vector3d & vb){
    Vector3d va = Tab*vb;
    return va;
}



bool intersect(Point i0, Point i1, Point j0, Point j1){
    int ab = ccw(i0, i1, j0)*ccw(i0, i1, j1);
    int cd = ccw(j0, j1, i0)*ccw(j0, j1, i1);
    if (ab == 0 && cd == 0) {
//        if (a > b)swap(a, b);
//        if (c > d)swap(c, d);
//        return c <= b && a <= d;
        return false;
    }
    return ab <= 0 && cd <= 0;
}

int ccw(Point a, Point b, Point c) {
    double op = a.x*b.y + b.x*c.y + c.x*a.y;
    op -= (a.y*b.x + b.y*c.x + c.y*a.x);
    if (op > 1e-9) return 1;
    else if (abs(op) < 1e-9) return 0;
    else return -1;
}

nav_msgs::Path getPath(const vector<Vector2d, aligned_allocator<Vector2d>>& point2dSeq, string frame_id ){


    nav_msgs::Path nodeNavPath;
    nodeNavPath.header.frame_id = frame_id.c_str();
    for (auto & lane : point2dSeq){
        geometry_msgs::PoseStamped aPoint;
        aPoint.pose.position.x = lane(0);
        aPoint.pose.position.y = lane(1);
        nodeNavPath.poses.push_back(aPoint);
    }
    return nodeNavPath;

}

double meanCurvature(const vector<Vector2d, aligned_allocator<Vector2d>> & point2dSeq){

    double curvatureSum = 0;
    if (point2dSeq.size() <= 2){
        cerr << "Cannot compute curvature when number of points <= 2. curvature will be big number" << endl;
        return 1e+6;
    }

    for (int i = 0 ; i < point2dSeq.size()-2 ; i++){
        Vector2d v1 = point2dSeq[i+1] - point2dSeq[i];
        Vector2d v2 = point2dSeq[i+2] - point2dSeq[i+1];

        double angleThres = max(min(v1.dot(v2) / (v1.norm()*v2.norm()),1.0),-1.0);
        double theta = acos(angleThres);

        curvatureSum += theta/v1.norm();
    }

    cout << curvatureSum<< endl;
    return curvatureSum/(point2dSeq.size()-2);
}