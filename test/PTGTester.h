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

    bool insertDerefBoth(Pointer p, Pointee location)
    {
        PointsToGraph::Node *l, *r;

        if (!(l = PTG->findNode(p)))
            return false;
        if (!(r = PTG->findNode(location)))
            return false;

        return PTG->insertDerefBoth(l, r);
    }

    /*
    bool insertDerefPointee(Pointer p, Node *LocationNode);
    bool insertDerefPointee(Node *PointerNode, Node *LocationNode);
    bool insertDerefPointer(Node *PointerNode, Pointee location);
    bool insertDerefPointer(Node *PointerNode, Node *LocationNode);
    */

    // --------------------------------------------------------------------
    // applyRules functions -> convert ruleCodes into points-to-graph
    // copied from PointsTo.h
    // --------------------------------------------------------------------
    bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                   VARIABLE<const llvm::Value *> > const& E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(const llvm::DataLayout &DL, ASSIGNMENT<
                   VARIABLE<const llvm::Value *>,
                   GEP<VARIABLE<const llvm::Value *> > > const& E)
    {
        PTG->applyRule(DL, E);
    }

    bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                   REFERENCE<VARIABLE<const llvm::Value *> > > const& E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                   DEREFERENCE< VARIABLE<const llvm::Value *> >
                   > const& E, const int idx = -1)
    {
        PTG->applyRule(E, idx);
    }

    bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                   VARIABLE<const llvm::Value *> > const& E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                   REFERENCE<VARIABLE<const llvm::Value *> > > const &E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                   DEREFERENCE<VARIABLE<const llvm::Value *> > > const& E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                   ALLOC<const llvm::Value *> > const &E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                   NULLPTR<const llvm::Value *> > const &E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                   NULLPTR<const llvm::Value *> > const &E)
    {
        PTG->applyRule(E);
    }

    bool applyRule(DEALLOC<const llvm::Value *> &E)
    {
        PTG->applyRule(E);
    }

    bool applyRules(const RuleCode &RC, const llvm::DataLayout &DL)
    {
        PTG->applyRules(RC, DL);
    }

private:
    PointsToGraph *PTG;
};

class FixedCategories : public PointsToCategories
{
public:
    FixedCategories(const std::set<std::set<Pointer> >& Categories)
    {
	// it's not efficent, but it doesn't have to
        this->Categories = Categories;
    }

    virtual unsigned int getCategory(Pointer a) const;
private:
    std::set<std::set<Pointer> > Categories;
};

// Simulates Andersen's algorithm
class AllInSelfCategory : public PointsToCategories
{
public:
    virtual unsigned int getCategory(Pointer a) const;
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
PointsToGraph::Pointer getPointer(llvm::Module *M, const char *name, int64_t off = -1);

enum deref {
    DEREF_NONE,
    DEREF_POINTEE,
    DEREF_POINTER,
    DEREF_BOTH
};

// insert new points-to pair into PTG
void addPointsTo(llvm::Module *M, PTGTester &PTG,
                 const char *a, const char *b,
                 enum deref derefFlag = DEREF_NONE,
                 int64_t aoff = -1, int64_t boff = -1);

// add points-to pair to control points-to set
void addPointsTo(llvm::Module *M, PointsToSets& PTSets,
                 const char *a, const char *b,
                 int64_t aoff = -1, int64_t boff = -1);

bool check(PointsToSets &A, PointsToSets &B);
bool check(PTGTester &PTG, PointsToSets &S);

void notTested(const char *msg);

std::pair<int, int> getResults(void);
