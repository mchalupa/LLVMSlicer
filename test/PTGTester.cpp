#include "PTGTester.h"

using namespace llvm;

typedef PointsToGraph::Pointer Pointer;
typedef PointsToGraph::Pointer Pointee;

static int failed = 0;
static int total = 0;

static std::map<const char *, Pointer> valueMap;

void dumpPointsToSets(ptr::PointsToSets& PS)
{
    ptr::PointsToSets::const_iterator I, E;
    ptr::PointsToSets::PointsToSet::const_iterator II, EE;

    for (I = PS.begin(), E = PS.end(); I != E; ++I) {
        errs() << "PTR: ";
        I->first.first->dump();

        for (II = I->second.cbegin(), EE = I->second.cend(); II != EE; ++II) {
            errs() << "    --> ";
            II->first->dump();
        }
    }
}

bool comparePointsToSets(ptr::PointsToSets& a, ptr::PointsToSets& b)
{
    ptr::PointsToSets::const_iterator I1, E1, I2;
    ptr::PointsToSets::PointsToSet::const_iterator PI1, PE1, PI2;

    if (a.getContainer().size() != b.getContainer().size())
        return false;

    // the sets should have the same order, compare item by item
    for (I1 = a.begin(), E1 = a.end(), I2 = b.begin();
            I1 != E1; ++I1, ++I2) {

        const ptr::PointsToSets::PointsToSet& A = I1->second;
        const ptr::PointsToSets::PointsToSet& B = I2->second;

        if (A.size() != B.size())
            return false;

        for (PI1 = A.begin(), PI2 = B.begin(), PE1 = A.end();
                PI1 != PE1; ++PI1, ++PI2) {

            if (*PI1 != *PI2)
                return false;
        }

    }

    return true;
}

// get or create value
Pointer getPointer(Module *M, const char *name)
{
    std::map<const char *, Pointer>::iterator I = valueMap.find(name);

    if (I == valueMap.end()) {
        Value *va = new GlobalVariable(*M, IntegerType::get(M->getContext(), 32),
                                         false,
                                         GlobalValue::CommonLinkage, 0 , name);
        Pointer p(va, -1);
        valueMap.insert(std::make_pair(name, p));

        return p;
    }

    return I->second;
}

void addPointsTo(Module *M, PTGTester &PTG,
                 const char *a, const char *b,
                 enum deref derefFlag)
{
    Pointer pa, pb;

    pa = getPointer(M, a);
    pb = getPointer(M, b);

    switch (derefFlag) {
    case DEREF_NONE:
        PTG.insert(pa, pb); break;
    case DEREF_POINTEE:
        PTG.insertDerefPointee(pa, pb); break;
    case DEREF_POINTER:
        PTG.insertDerefPointer(pa, pb); break;
    default:
        assert(0);
    }
}

void addPointsTo(Module *M, ptr::PointsToSets& PTSets,
                 const char *a, const char *b)
{
    Pointer p = getPointer(M, a);
    Pointee l = getPointer(M, b);

    ptr::PointsToSets::PointsToSet& S = PTSets[p];
    S.insert(l);
}

bool check(PTGTester &PTG, ptr::PointsToSets &S)
{
    ptr::PointsToSets PTGSet;

    PTG.getPTG().toPointsToSets(PTGSet);

    ++total;

    if (!comparePointsToSets(S, PTGSet)) {
        ++failed;

        errs() << "Points-to graph:\n\n";
        PTG.getPTG().dump();
        errs() << "\nAssocaited points-to set:\n\n";
        dumpPointsToSets(PTGSet);
        errs() << "But should be\n";
        dumpPointsToSets(S);
        errs() << "\n";

        return false;
    }

    return true;
}

std::pair<int, int> getResults(void)
{
    return std::pair<int, int>(failed, total);
}
