#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <math.h>

using namespace std;

class Tools
{
public:
    Tools(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
    ~Tools();

public:
    // For converting back and forth between radians and degrees.
    double pi() const { return M_PI; }
    double deg2rad(double x) const { return x * pi() / 180; }
    double rad2deg(double x) const { return x * 180 / pi(); }

    double distance(double x1, double y1, double x2, double y2) const;
    
    int ClosestWaypoint(double x, double y) const;
    int NextWaypoint(double x, double y, double theta) const;

    vector<double> getFrenet(double x, double y, double theta) const;
    vector<double> getXY(double s, double d) const;

private:
    const vector<double> &maps_s;
    const vector<double> &maps_x;
    const vector<double> &maps_y;
};

#endif
