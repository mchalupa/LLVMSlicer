#include <llvm/LLVMContext.h>
#include <llvm/Function.h>
#include <llvm/Module.h>
#include <llvm/Support/IRReader.h>
#include <llvm/Support/raw_ostream.h>
#include <cassert>

#include "../src/PointsTo/PointsTo.h"

using namespace llvm;

typedef ptr::PointsToGraph::Pointer Pointer;
typedef ptr::PointsToGraph::Pointee Pointee;

static ptr::ProgramStructure *PS;
static std::map<const char *, Pointer> valueMap;
static int retval = 0;

static void dumpPointsToSets(ptr::PointsToSets& PS)
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

enum deref {
    DEREF_NONE,
    DEREF_POINTEE,
    DEREF_POINTER
};

static void addPointsTo(Module &M, ptr::PointsToGraph &PTG,
                        const char *a, const char *b,
                        enum deref derefFlag = DEREF_NONE)
{
    std::map<const char *, Pointer>::iterator ai, bi;
    Pointer pa, pb;

    ai = valueMap.find(a);
    bi = valueMap.find(b);

    if (ai == valueMap.end()) {
        Value *va = new GlobalVariable(M, IntegerType::get(M.getContext(), 32),
                                         false,
                                         GlobalValue::CommonLinkage, 0 , a);
        pa = Pointer(va, -1);
        // use default copy constructor
        valueMap.insert(std::make_pair(a, pa));
    } else {
        pa = ai->second;
    }

    if (bi == valueMap.end()) {
        Value *vb = new GlobalVariable(M, IntegerType::get(M.getContext(), 32),
                                          false,
                                          GlobalValue::CommonLinkage, 0 , b);
        pb = Pointer(vb, -1);
        valueMap.insert(std::make_pair(b, pb));
    } else {
        pb = bi->second;
    }

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

static void addPointsTo(ptr::PointsToSets& PTSets,
                        const char *a, const char *b)
{
    // XXX this will create the item in map if it doesn't exists
    Pointer p = valueMap[a];
    Pointee l = valueMap[b];

    ptr::PointsToSets::PointsToSet& S = PTSets[p];
    S.insert(l);
}

static bool comparePointsToSets(ptr::PointsToSets& a, ptr::PointsToSets& b)
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

static void check(ptr::PointsToGraph &PTG, ptr::PointsToSets &S)
{
    ptr::PointsToSets PTGSet;

    PTG.toPointsToSets(PTGSet);

    if (!comparePointsToSets(S, PTGSet)) {
        retval = 1;

        errs() << "Points-to graph:\n\n";
        PTG.dump();
        errs() << "\nAssocaited points-to set:\n\n";
        dumpPointsToSets(PTGSet);
        errs() << "But should be\n";
        dumpPointsToSets(S);
        errs() << "\n";
    }
}

static void buildPointsToGraph(Module &M, bool (*seq)(Module&, ptr::PointsToGraph&),
                                ptr::PointsToCategories *categ = NULL)
{
    ptr::PointsToGraph PTG(PS,
        (categ == NULL) ? new ptr::AllInSelfCategory : categ);

    seq(M, PTG);
}

static void test_test(void)
{
    ptr::PointsToSets A;
    ptr::PointsToSets B;

    assert(comparePointsToSets(A, A));
    assert(comparePointsToSets(A, B));

    Pointee p1(NULL, -1);
    Pointee p2(NULL, 8);
    ptr::PointsToSets::PointsToSet& S1 = A[p1];
    ptr::PointsToSets::PointsToSet& S2 = B[p1];

    assert(comparePointsToSets(A, B));
    assert(comparePointsToSets(A, A));

    S1.insert(p1);
    S2.insert(p1);
    assert(comparePointsToSets(A, B));
    assert(comparePointsToSets(A, A));


    S1.insert(p2);
    assert(!comparePointsToSets(A, B));
    assert(comparePointsToSets(A, A));

    S2.insert(p2);
    assert(comparePointsToSets(A, B));
}

static bool figure1(Module& M, ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "b", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "d", "e");

    // points to set for control
    // steengaards
    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "d");
    addPointsTo(PTSets, "b", "c");
    addPointsTo(PTSets, "b", "e");
    addPointsTo(PTSets, "d", "c");
    addPointsTo(PTSets, "d", "e");

    check(PTG, PTSets);
}

static bool figure2(Module& M, ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "x", "v1");
    addPointsTo(M, PTG, "x", "v2");
    addPointsTo(M, PTG, "y", "v1");

    addPointsTo(PTSets, "x", "v1");
    addPointsTo(PTSets, "x", "v2");
    addPointsTo(PTSets, "y", "v1");
    addPointsTo(PTSets, "y", "v2");

    check(PTG, PTSets);
}

static bool figure3(Module& M, ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "c", "d");

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "a", "d");
    addPointsTo(PTSets, "b", "b");
    addPointsTo(PTSets, "b", "c");
    addPointsTo(PTSets, "b", "d");
    addPointsTo(PTSets, "c", "b");
    addPointsTo(PTSets, "c", "c");
    addPointsTo(PTSets, "c", "d");
    addPointsTo(PTSets, "d", "b");
    addPointsTo(PTSets, "d", "c");
    addPointsTo(PTSets, "d", "d");

    check(PTG, PTSets);
}

static bool derefPointer1(Module &M, ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d", DEREF_POINTER);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "b", "d");
    addPointsTo(PTSets, "c", "d");

    check(PTG, PTSets);
}

static bool derefPointee1(Module &M, ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "d", "a", DEREF_POINTEE);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "d", "b");
    addPointsTo(PTSets, "d", "a");

    check(PTG, PTSets);
}




int main(int argc, char **argv)
{
	LLVMContext context;
	SMDiagnostic SMD;

    // test testing functions first
    test_test();

	Module M("points-to-test", getGlobalContext());

    ptr::ProgramStructure ps(M);
    PS = &ps;

    // these are from Shapiro-Horowitz paper and simulate Steengaard's analysis
    buildPointsToGraph(M, figure1, new ptr::AllInOneCategory());
    buildPointsToGraph(M, figure2, new ptr::AllInOneCategory());
    buildPointsToGraph(M, figure3, new ptr::AllInOneCategory());

    buildPointsToGraph(M, derefPointer1);
	return retval;
}
