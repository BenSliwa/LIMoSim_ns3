#ifndef LINKERNIGHAN_H
#define LINKERNIGHAN_H


#include <vector>

using namespace std;

namespace LIMoSim {
namespace mobility {
namespace routing {

typedef unsigned int uint;

class LinKernighan
{
public:
    int size;
    LinKernighan(vector<vector<double> > &_edgeDistances, vector<uint> &ids);
    vector<uint> getCurrentTour();
    double getCurrentTourDistance();
    void optimizeTour();
    void printTour();
    void printTourIds();


private:
    vector<uint> tour;
    vector<vector<int> > edgeFlags;
    vector<pair<double, double> > coords;
    vector<uint> ids;
    void joinLocations(int i, int j);
    vector<vector<double> > edgeDistances;
    void LKMove(int tourStart);
    void reverse(int start, int end);
    bool isTour();
};

} // namespace routing
} // namespace mobility
} // namespace LIMoSim

#endif // LINKERNIGHAN_H
