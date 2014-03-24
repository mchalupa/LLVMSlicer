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


// helper functions
// ------------------------------
using namespace llvm::ptr;

void dumpPointsToSets(PointsToSets& PS);
bool comparePointsToSets(PointsToSets& a, PointsToSets& b);

// get pointer with given name. All such pointers are globals
// and should be used for testing PTG insert logic only.
// For other testing must be used working Module with real Values
PointsToGraph::Pointer getPointer(llvm::Module *M, const char *name);

enum deref {
    DEREF_NONE,
    DEREF_POINTEE,
    DEREF_POINTER
};

// insert new points-to pair into PTG
void addPointsTo(llvm::Module *M, PTGTester &PTG,
                 const char *a, const char *b,
                 enum deref derefFlag = DEREF_NONE);

// add points-to pair to control points-to set
void addPointsTo(llvm::Module *M, PointsToSets& PTSets,
                 const char *a, const char *b);

bool check(PTGTester &PTG, PointsToSets &S);

std::pair<int, int> getResults(void);
