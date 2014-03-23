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

static Module *M;
static ptr::ProgramStructure *PS;
static std::map<const char *, Pointer> valueMap;

static int failed = 0;
static int total = 0;

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

// get or create value
static Pointer getPointer(const char *name)
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

enum deref {
    DEREF_NONE,
    DEREF_POINTEE,
    DEREF_POINTER
};

static void addPointsTo(ptr::PointsToGraph &PTG,
                        const char *a, const char *b,
                        enum deref derefFlag = DEREF_NONE)
{
    Pointer pa, pb;

    pa = getPointer(a);
    pb = getPointer(b);

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
    Pointer p = getPointer(a);
    Pointee l = getPointer(b);

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

static bool check(ptr::PointsToGraph &PTG, ptr::PointsToSets &S)
{
    ptr::PointsToSets PTGSet;

    PTG.toPointsToSets(PTGSet);

    ++total;

    if (!comparePointsToSets(S, PTGSet)) {
        ++failed;

        errs() << "Points-to graph:\n\n";
        PTG.dump();
        errs() << "\nAssocaited points-to set:\n\n";
        dumpPointsToSets(PTGSet);
        errs() << "But should be\n";
        dumpPointsToSets(S);
        errs() << "\n";

        return false;
    }

    return true;
}

static void buildPointsToGraph(bool (*seq)(ptr::PointsToGraph&),
                                ptr::PointsToCategories *categ = NULL)
{
    ptr::PointsToGraph PTG(PS,
        (categ == NULL) ? new ptr::AllInSelfCategory : categ);

    seq(PTG);
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

static bool figure1(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "b", "c");
    addPointsTo(PTG, "a", "d");
    addPointsTo(PTG, "d", "e");

    // points to set for control
    // steengaards
    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "d");
    addPointsTo(PTSets, "b", "c");
    addPointsTo(PTSets, "b", "e");
    addPointsTo(PTSets, "d", "c");
    addPointsTo(PTSets, "d", "e");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool figure2(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "x", "v1");
    addPointsTo(PTG, "x", "v2");
    addPointsTo(PTG, "y", "v1");

    addPointsTo(PTSets, "x", "v1");
    addPointsTo(PTSets, "x", "v2");
    addPointsTo(PTSets, "y", "v1");
    addPointsTo(PTSets, "y", "v2");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool figure3(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "a", "d");
    addPointsTo(PTG, "c", "d");

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

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointer1(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "a", "d", DEREF_POINTER);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "b", "d");
    addPointsTo(PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointer2(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "b", "a");
    addPointsTo(PTG, "b", "c");
    addPointsTo(PTG, "a", "d", DEREF_POINTER);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "b", "a");
    addPointsTo(PTSets, "b", "c");
    addPointsTo(PTSets, "b", "d");
    addPointsTo(PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for the 1st check of " << __func__ << "\n";

    addPointsTo(PTG, "b", "d", DEREF_POINTER);

    addPointsTo(PTSets, "a", "d");
    addPointsTo(PTSets, "d", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointer3(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "c", "d");
    addPointsTo(PTG, "a", "a", DEREF_POINTER);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "c", "d");
    addPointsTo(PTSets, "b", "a");

    if (!check(PTG, PTSets))
        errs() << "dump for the 1st part of " << __func__ << "\n";

    addPointsTo(PTG, "e", "d", DEREF_POINTER);

    if (!check(PTG, PTSets))
        errs() << "dump for the 2nd part of " << __func__ << "\n";
}

static bool derefPointee1(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "d", "a", DEREF_POINTEE);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "d", "b");
    addPointsTo(PTSets, "d", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointee2(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "d", "c", DEREF_POINTEE);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointee3(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "a", "a", DEREF_POINTEE);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for the 1st part of " << __func__ << "\n";

    addPointsTo(PTG, "a", "a");
    addPointsTo(PTG, "c", "a", DEREF_POINTEE);

    addPointsTo(PTSets, "a", "a");
    addPointsTo(PTSets, "c", "a");
    addPointsTo(PTSets, "c", "b");
    addPointsTo(PTSets, "c", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for the 2nd part of " << __func__ << "\n";

    addPointsTo(PTG, "c", "a", DEREF_POINTEE);

    if (!check(PTG, PTSets))
        errs() << "dump for the 3rd part of " << __func__ << "\n";
}

static bool derefPointeer1(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "c", "a");
    addPointsTo(PTG, "c", "d");
    addPointsTo(PTG, "a", "c", DEREF_POINTER);
    addPointsTo(PTG, "a", "c", DEREF_POINTER);
    addPointsTo(PTG, "a", "c", DEREF_POINTEE);

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "c", "a");
    addPointsTo(PTSets, "c", "d");
    addPointsTo(PTSets, "b", "c");
    addPointsTo(PTSets, "c", "c");
    addPointsTo(PTSets, "a", "a");
    addPointsTo(PTSets, "a", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories1(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer("a"));
    C.insert(getPointer("b"));

    Categories.insert(C);
    C.clear();

    C.insert(getPointer("c"));
    C.insert(getPointer("d"));
    Categories.insert(C);

    return Categories;
}

static bool fixedCateg1(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "a", "d");
    addPointsTo(PTG, "c", "d");

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "a", "d");
    addPointsTo(PTSets, "c", "d");
    addPointsTo(PTSets, "c", "c");
    addPointsTo(PTSets, "d", "c");
    addPointsTo(PTSets, "d", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories2(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer("a"));
    C.insert(getPointer("c"));

    Categories.insert(C);
    C.clear();

    C.insert(getPointer("b"));
    C.insert(getPointer("d"));
    Categories.insert(C);

    return Categories;
}

static bool fixedCateg2(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "a", "d");
    addPointsTo(PTG, "c", "d");

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "a", "d");
    addPointsTo(PTSets, "c", "b");
    addPointsTo(PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories3(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer("a"));
    C.insert(getPointer("b"));

    Categories.insert(C);
    C.clear();

    C.insert(getPointer("c"));
    Categories.insert(C);
    C.clear();

    C.insert(getPointer("d"));
    Categories.insert(C);

    return Categories;
}

static bool fixedCateg3(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "a", "d");
    addPointsTo(PTG, "c", "d");

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "a", "d");
    addPointsTo(PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories4(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer("b"));
    C.insert(getPointer("c"));

    Categories.insert(C);

    return Categories;
}

static bool fixedCateg4(ptr::PointsToGraph& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(PTG, "a", "b");
    addPointsTo(PTG, "a", "c");
    addPointsTo(PTG, "c", "d");

    addPointsTo(PTSets, "a", "b");
    addPointsTo(PTSets, "a", "c");
    addPointsTo(PTSets, "b", "d");
    addPointsTo(PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}


int main(int argc, char **argv)
{
	LLVMContext context;
	SMDiagnostic SMD;

    // test testing functions first
    test_test();

	Module m("points-to-test", getGlobalContext());
    M = &m;

    ptr::ProgramStructure ps(m);
    PS = &ps;

    // these are from Shapiro-Horowitz paper and simulate Steengaard's analysis
    buildPointsToGraph(figure1, new ptr::AllInOneCategory());
    buildPointsToGraph(figure2, new ptr::AllInOneCategory());
    buildPointsToGraph(figure3, new ptr::AllInOneCategory());

    // these use Andersen analysis
    buildPointsToGraph(derefPointer1);
    buildPointsToGraph(derefPointer2);
    buildPointsToGraph(derefPointer3);
    buildPointsToGraph(derefPointee1);
    buildPointsToGraph(derefPointee2);
    buildPointsToGraph(derefPointee3);
    buildPointsToGraph(derefPointeer1);

    // tests with fixed categories, the first three ones are also from the
    // Shapiro-Horwitz paper (Figure 3)
    buildPointsToGraph(fixedCateg1, new ptr::FixedCategories(categories1()));
    buildPointsToGraph(fixedCateg2, new ptr::FixedCategories(categories2()));
    buildPointsToGraph(fixedCateg3, new ptr::FixedCategories(categories3()));
    buildPointsToGraph(fixedCateg4, new ptr::FixedCategories(categories4()));

    if (failed)
        errs() << failed << " tests from " << total << " failed!\n";

	return failed != 0;
}
