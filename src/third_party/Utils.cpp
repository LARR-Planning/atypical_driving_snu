//
// Created by jbs on 20. 5. 14..
//

#include <third_party/Utils.h>

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

    double maxCurve = -1;

    for (int i = 0 ; i < point2dSeq.size()-2 ; i++){
        Vector2d v1 = point2dSeq[i+1] - point2dSeq[i];
        Vector2d v2 = point2dSeq[i+2] - point2dSeq[i+1];
        if(v1.norm()<1e-3||v2.norm()<1e-3)
        {
            curvatureSum +=1e3; // paramterize?? //Yunwoo
        }
        else
        {
            double angleThres = max(min(v1.dot(v2) / (v1.norm()*v2.norm()),1.0),-1.0);
            double theta = acos(angleThres);
            curvatureSum += theta/v1.norm();
        }
    }
    //cout << curvatureSum<< endl;
    // cout << curvatureSum/(point2dSeq.size()-2)<< endl;
    return curvatureSum/(point2dSeq.size()-2);


}

void pixelTo3DPoint(const sensor_msgs::PointCloud2& pCloud, const int u, const int v,geometry_msgs::Point & p)
{
    // get width and height of 2D point cloud data
    int width = pCloud.width;
    int height = pCloud.height;

    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

    // compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Y has an offset of 4

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    // put data into the point p
    p.x = X;
    p.y = Y;
    p.z = Z;
}


