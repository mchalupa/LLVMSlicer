#include "../src/PointsTo/PointsTo.h"

namespace llvm {
namespace ptr {

// Class that can access PointsToGraph private attributes
// due to testing
class PTGTester
{
public:
    typedef PointsToGraph::Pointer Pointer;
    typedef PointsToGraph::Pointee Pointee;

    PTGTester(PointsToGraph *PTG)
        :PTG(PTG) {}

    const PointsToGraph& getPTG(void) const { return *PTG; }

    // give front-end for insert functions
    bool insert(Pointer p, Pointee location)
    {
        return PTG->insert(p, location);
    }

    bool insert(Pointer p, std::set<Pointee>& locations)
    {
        return PTG->insert(p, locations);
    }

    bool insertDerefPointee(Pointer p, Pointee location)
    {
        return PTG->insertDerefPointee(p, location);
    }

    bool insertDerefPointer(Pointer p, Pointee location)
    {
        return PTG->insertDerefPointer(p, location);
    }

    /*
    bool insertDerefPointee(Pointer p, Node *LocationNode);
    bool insertDerefPointee(Node *PointerNode, Node *LocationNode);
    bool insertDerefPointer(Node *PointerNode, Pointee location);
    bool insertDerefPointer(Node *PointerNode, Node *LocationNode);
    bool insertDerefBoth(Node *PointerNode, Node *LocationNode);
    */

private:
    PointsToGraph *PTG;
};

} // namespace ptr
} // namespace llvm
