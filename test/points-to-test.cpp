#include <llvm/LLVMContext.h>
#include <llvm/Function.h>
#include <llvm/Module.h>
#include <llvm/Support/IRReader.h>
#include <llvm/Support/raw_ostream.h>
#include <cassert>

#include "../src/PointsTo/PointsTo.h"
#include "PTGTester.h"

using namespace llvm;
using ptr::PTGTester;

typedef ptr::PointsToGraph::Pointer Pointer;
typedef ptr::PointsToGraph::Pointee Pointee;

static Module *M;
static ptr::ProgramStructure *PS;

static void buildPointsToGraph(bool (*seq)(PTGTester&),
                                ptr::PointsToCategories *categ = NULL)
{
    ptr::PointsToGraph PTG(PS,
        (categ == NULL) ? new ptr::AllInSelfCategory : categ);

    PTGTester T(&PTG);
    seq(T);
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

static bool figure1(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "b", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "d", "e");

    // points to set for control
    // steengaards
    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "b", "c");
    addPointsTo(M, PTSets, "b", "e");
    addPointsTo(M, PTSets, "d", "c");
    addPointsTo(M, PTSets, "d", "e");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool figure2(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "x", "v1");
    addPointsTo(M, PTG, "x", "v2");
    addPointsTo(M, PTG, "y", "v1");

    addPointsTo(M, PTSets, "x", "v1");
    addPointsTo(M, PTSets, "x", "v2");
    addPointsTo(M, PTSets, "y", "v1");
    addPointsTo(M, PTSets, "y", "v2");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool figure3(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "c", "d");

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "b", "b");
    addPointsTo(M, PTSets, "b", "c");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "c", "b");
    addPointsTo(M, PTSets, "c", "c");
    addPointsTo(M, PTSets, "c", "d");
    addPointsTo(M, PTSets, "d", "b");
    addPointsTo(M, PTSets, "d", "c");
    addPointsTo(M, PTSets, "d", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointer1(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d", DEREF_POINTER);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointer2(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "b", "a");
    addPointsTo(M, PTG, "b", "c");
    addPointsTo(M, PTG, "a", "d", DEREF_POINTER);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "b", "a");
    addPointsTo(M, PTSets, "b", "c");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for the 1st check of " << __func__ << "\n";

    addPointsTo(M, PTG, "b", "d", DEREF_POINTER);

    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "d", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointer3(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "c", "d");
    addPointsTo(M, PTG, "a", "a", DEREF_POINTER);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "c", "d");
    addPointsTo(M, PTSets, "b", "a");

    if (!check(PTG, PTSets))
        errs() << "dump for the 1st part of " << __func__ << "\n";

    addPointsTo(M, PTG, "e", "d", DEREF_POINTER);

    if (!check(PTG, PTSets))
        errs() << "dump for the 2nd part of " << __func__ << "\n";
}

static bool derefPointee1(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "d", "a", DEREF_POINTEE);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "d", "b");
    addPointsTo(M, PTSets, "d", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointee2(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "d", "c", DEREF_POINTEE);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefPointee3(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "a", DEREF_POINTEE);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for the 1st part of " << __func__ << "\n";

    addPointsTo(M, PTG, "a", "a");
    addPointsTo(M, PTG, "c", "a", DEREF_POINTEE);

    addPointsTo(M, PTSets, "a", "a");
    addPointsTo(M, PTSets, "c", "a");
    addPointsTo(M, PTSets, "c", "b");
    addPointsTo(M, PTSets, "c", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for the 2nd part of " << __func__ << "\n";

    addPointsTo(M, PTG, "c", "a", DEREF_POINTEE);

    if (!check(PTG, PTSets))
        errs() << "dump for the 3rd part of " << __func__ << "\n";
}

static bool derefPointeer1(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "a");
    addPointsTo(M, PTG, "c", "d");
    addPointsTo(M, PTG, "a", "c", DEREF_POINTER); // POINTER !!
    addPointsTo(M, PTG, "a", "c", DEREF_POINTER);
    addPointsTo(M, PTG, "a", "c", DEREF_POINTEE); // POINTEE !!

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "a");
    addPointsTo(M, PTSets, "c", "d");
    addPointsTo(M, PTSets, "b", "c");
    addPointsTo(M, PTSets, "c", "c");
    addPointsTo(M, PTSets, "a", "a");
    addPointsTo(M, PTSets, "a", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefBoth1(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "a");
    addPointsTo(M, PTG, "c", "d");
    addPointsTo(M, PTG, "a", "c", DEREF_BOTH);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "a");
    addPointsTo(M, PTSets, "c", "d");
    addPointsTo(M, PTSets, "b", "a");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "c", "a");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool derefBoth2(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "b", "b");
    addPointsTo(M, PTG, "b", "d");
    addPointsTo(M, PTG, "d", "e");
    addPointsTo(M, PTG, "a", "b", DEREF_BOTH);

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "b", "b");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "d", "e");

    addPointsTo(M, PTSets, "c", "b");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}


static std::set<std::set<Pointer> > categories1(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer(M, "a"));
    C.insert(getPointer(M, "b"));

    Categories.insert(C);
    C.clear();

    C.insert(getPointer(M, "c"));
    C.insert(getPointer(M, "d"));
    Categories.insert(C);

    return Categories;
}

static bool fixedCateg1(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "c", "d");

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "c", "d");
    addPointsTo(M, PTSets, "c", "c");
    addPointsTo(M, PTSets, "d", "c");
    addPointsTo(M, PTSets, "d", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories2(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer(M, "a"));
    C.insert(getPointer(M, "c"));

    Categories.insert(C);
    C.clear();

    C.insert(getPointer(M, "b"));
    C.insert(getPointer(M, "d"));
    Categories.insert(C);

    return Categories;
}

static bool fixedCateg2(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "c", "d");

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "c", "b");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories3(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer(M, "a"));
    C.insert(getPointer(M, "b"));

    Categories.insert(C);
    C.clear();

    C.insert(getPointer(M, "c"));
    Categories.insert(C);
    C.clear();

    C.insert(getPointer(M, "d"));
    Categories.insert(C);

    return Categories;
}

static bool fixedCateg3(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "c", "d");

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories4(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer(M, "b"));
    C.insert(getPointer(M, "c"));

    Categories.insert(C);

    return Categories;
}

static bool fixedCateg4(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "d");

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static std::set<std::set<Pointer> > categories5(void)
{
    std::set<std::set<Pointer> > Categories;
    std::set<Pointer> C;

    C.insert(getPointer(M, "a"));
    C.insert(getPointer(M, "c"));
    C.insert(getPointer(M, "e"));

    Categories.insert(C);

    C.clear();
    C.insert(getPointer(M, "d"));
    Categories.insert(C);

    C.clear();
    C.insert(getPointer(M, "b"));
    Categories.insert(C);

    return Categories;
}

static bool fixedCateg5(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;

    addPointsTo(M, PTG, "b", "a");
    addPointsTo(M, PTG, "b", "c");
    addPointsTo(M, PTG, "d", "e");
    addPointsTo(M, PTG, "d", "a");

    addPointsTo(M, PTSets, "b", "a");
    addPointsTo(M, PTSets, "b", "c");
    addPointsTo(M, PTSets, "b", "e");
    addPointsTo(M, PTSets, "d", "e");
    addPointsTo(M, PTSets, "d", "a");
    addPointsTo(M, PTSets, "d", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static bool applyVarAsgnAlloc(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "e");

    lval = getPointer(M, "d").first;
    rval = getPointer(M, "a").first;
    PTG.applyRule((ruleVar(lval) = ruleAllocSite(rval)).getSort());

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "e");
    // alloc should have offset 0
    addPointsTo(M, PTSets, "d", "a", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    lval = getPointer(M, "d").first;
    rval = getPointer(M, "c").first;
    PTG.applyRule((ruleVar(lval) = ruleAllocSite(rval)).getSort());

    addPointsTo(M, PTSets, "d", "c", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";

    PTG.applyRule((ruleVar(lval) = ruleAllocSite(rval)).getSort());
    if (!check(PTG, PTSets))
        errs() << "dump for 3rd in " << __func__ << "\n";
}

static bool applyVarAsgnRef(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "e");

    lval = getPointer(M, "d").first;
    rval = getPointer(M, "a").first;
    PTG.applyRule((ruleVar(lval) = &ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "d", "a", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    lval = getPointer(M, "d").first;
    rval = getPointer(M, "c").first;
    PTG.applyRule((ruleVar(lval) = &ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "d", "c", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";

    PTG.applyRule((ruleVar(lval) = ruleAllocSite(rval)).getSort());
    if (!check(PTG, PTSets))
        errs() << "dump for 3rd in " << __func__ << "\n";
}


static bool applyVarAsgnVar(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "e");

    lval = getPointer(M, "d").first;
    rval = getPointer(M, "a").first;
    PTG.applyRule((ruleVar(lval) = ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "d", "b");
    addPointsTo(M, PTSets, "d", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    // this should not change the sets
    rval = getPointer(M, "d").first;
    PTG.applyRule((ruleVar(lval) = ruleVar(rval)).getSort());

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";

    // this should also not change the sets, since e
    // has no points-to set
    lval = getPointer(M, "e").first;
    rval = lval;
    PTG.applyRule((ruleVar(lval) = ruleVar(rval)).getSort());

    if (!check(PTG, PTSets))
        errs() << "dump for 3rd in " << __func__ << "\n";
}

static bool applyVarAsgnDeref(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "e");
    addPointsTo(M, PTG, "e", "d");
    addPointsTo(M, PTG, "e", "f");

    lval = getPointer(M, "a").first;
    rval = getPointer(M, "c").first;
    PTG.applyRule((ruleVar(lval) = *ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "e", "d");
    addPointsTo(M, PTSets, "e", "f");

    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "a", "f");

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    // this should not change the sets
    // lval = "a"
    PTG.applyRule((ruleVar(lval) = *ruleVar(lval)).getSort());

    addPointsTo(M, PTSets, "a", "e");

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";

    lval = getPointer(M, "e").first;
    rval = lval;
    PTG.applyRule((ruleVar(lval) = ruleVar(rval)).getSort());

    if (!check(PTG, PTSets))
        errs() << "dump for 3rd in " << __func__ << "\n";
}

static bool applyDrefVarAsgnVar(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "e");
    addPointsTo(M, PTG, "e", "d");
    addPointsTo(M, PTG, "e", "f");

    lval = getPointer(M, "a").first;
    rval = getPointer(M, "e").first;
    PTG.applyRule((*ruleVar(lval) = ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "e", "d");
    addPointsTo(M, PTSets, "e", "f");

    addPointsTo(M, PTSets, "c", "d");
    addPointsTo(M, PTSets, "c", "f");

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    addPointsTo(M, PTG, "c", "a");
    addPointsTo(M, PTSets, "c", "a");

    rval = lval; // *a = a;
    PTG.applyRule((*ruleVar(lval) = ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "c", "c");

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";

    lval = getPointer(M, "c").first;
    rval = getPointer(M, "d").first;
    PTG.applyRule((*ruleVar(lval) = ruleVar(rval)).getSort());

    if (!check(PTG, PTSets))
        errs() << "dump for 3rd in " << __func__ << "\n";

    addPointsTo(M, PTG, "d", "e");
    addPointsTo(M, PTSets, "d", "e");

    // *c = e
    rval = getPointer(M, "e").first;
    PTG.applyRule((*ruleVar(lval) = ruleVar(rval)).getSort());
    addPointsTo(M, PTSets, "d", "f");
    addPointsTo(M, PTSets, "d", "d");
    addPointsTo(M, PTSets, "f", "f");
    addPointsTo(M, PTSets, "f", "d");
    addPointsTo(M, PTSets, "a", "d");
    addPointsTo(M, PTSets, "a", "f");

    if (!check(PTG, PTSets))
        errs() << "dump for 4rd in " << __func__ << "\n";
}

static bool applyDrefVarAsgnRef(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "g");
    addPointsTo(M, PTG, "c", "e");
    addPointsTo(M, PTG, "e", "d");
    addPointsTo(M, PTG, "e", "f");

    lval = getPointer(M, "a").first;
    rval = getPointer(M, "e").first;
    PTG.applyRule((*ruleVar(lval) = &ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "a", "g");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "e", "d");
    addPointsTo(M, PTSets, "e", "f");

    addPointsTo(M, PTSets, "c", "e", -1, 0);
    addPointsTo(M, PTSets, "g", "e", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    rval = lval; // lval = @a;
    lval = getPointer(M, "c").first;
    PTG.applyRule((*ruleVar(lval) = &ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "e", "a", -1, 0);
    addPointsTo(M, PTSets, "e", "a", 0, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";

    // should not change graph
    lval = getPointer(M, "f").first;
    rval = getPointer(M, "d").first;
    PTG.applyRule((*ruleVar(lval) = ruleVar(rval)).getSort());

    if (!check(PTG, PTSets))
        errs() << "dump for 3rd in " << __func__ << "\n";
}

static bool applyVarAsgnNull(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "c", "e");

    lval = getPointer(M, "d").first;
    rval = getPointer(M, "null").first;
    // have to add null value into getPointer
    PTG.applyRule((ruleVar(lval) = ruleNull(rval)).getSort());

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "d", "null", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    lval = getPointer(M, "e").first;
    PTG.applyRule((ruleVar(lval) = ruleNull(rval)).getSort());

    addPointsTo(M, PTSets, "e", "null", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";

    PTG.applyRule((ruleVar(lval) = ruleNull(rval)).getSort());
    if (!check(PTG, PTSets))
        errs() << "dump for 3rd in " << __func__ << "\n";
}

static bool applyDrefVarAsgnNull(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "g");
    addPointsTo(M, PTG, "c", "e");
    addPointsTo(M, PTG, "e", "d");
    addPointsTo(M, PTG, "e", "f");

    lval = getPointer(M, "a").first;
    rval = getPointer(M, "null").first;
    PTG.applyRule((*ruleVar(lval) = &ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "a", "g");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "e", "d");
    addPointsTo(M, PTSets, "e", "f");

    addPointsTo(M, PTSets, "c", "null", -1, 0);
    addPointsTo(M, PTSets, "g", "null", -1, 0);

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    // should not change graph
    lval = getPointer(M, "f").first;
    rval = getPointer(M, "null").first;
    PTG.applyRule((*ruleVar(lval) = ruleVar(rval)).getSort());

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";
}

static bool applyDrefVarAsgnDref(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "b", "c");
    addPointsTo(M, PTG, "c", "e");
    addPointsTo(M, PTG, "c", "g");
    addPointsTo(M, PTG, "e", "d");
    addPointsTo(M, PTG, "e", "f");

    lval = getPointer(M, "a").first;
    rval = getPointer(M, "b").first;
    PTG.applyRule((*ruleVar(lval) = *ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "b", "c");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "c", "g");
    addPointsTo(M, PTSets, "e", "d");
    addPointsTo(M, PTSets, "e", "f");

    addPointsTo(M, PTSets, "b", "e");
    addPointsTo(M, PTSets, "b", "g");

    if (!check(PTG, PTSets))
        errs() << "dump for 1st in " << __func__ << "\n";

    PTG.applyRule((*ruleVar(lval) = *ruleVar(rval)).getSort());
    addPointsTo(M, PTSets, "b", "f");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "c", "f");
    addPointsTo(M, PTSets, "c", "d");

    if (!check(PTG, PTSets))
        errs() << "dump for 2nd in " << __func__ << "\n";
}

static bool applyDrefVarAsgnDref2(PTGTester& PTG)
{
    ptr::PointsToSets PTSets;
    const llvm::Value *lval, *rval;

    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "b", "b");
    addPointsTo(M, PTG, "b", "d");
    addPointsTo(M, PTG, "d", "e");

    lval = getPointer(M, "a").first;
    rval = getPointer(M, "b").first;
    PTG.applyRule((*ruleVar(lval) = *ruleVar(rval)).getSort());

    addPointsTo(M, PTSets, "a", "b");
    addPointsTo(M, PTSets, "a", "c");
    addPointsTo(M, PTSets, "b", "b");
    addPointsTo(M, PTSets, "b", "d");
    addPointsTo(M, PTSets, "d", "e");

    addPointsTo(M, PTSets, "b", "e");
    addPointsTo(M, PTSets, "c", "e");
    addPointsTo(M, PTSets, "c", "b");
    addPointsTo(M, PTSets, "c", "d");


    if (!check(PTG, PTSets))
        errs() << "dump for " << __func__ << "\n";
}

static void toPointsToSets1(void)
{
    ptr::PointsToGraph PTG(PS, new ptr::AllInSelfCategory);
    ptr::PointsToSets A, B;

    PTGTester T(&PTG);

    // initialize graph with pts-to pairs
    addPointsTo(M, T, "a", "b");
    addPointsTo(M, T, "a", "c");
    addPointsTo(M, T, "b", "c");
    addPointsTo(M, T, "d", "d");
    addPointsTo(M, T, "e", "g");

    addPointsTo(M, A, "a", "b");
    addPointsTo(M, A, "a", "c");
    addPointsTo(M, A, "b", "c");
    addPointsTo(M, A, "d", "d");
    addPointsTo(M, A, "e", "g");

    if (!check(T, A))
        errs() << "prolly bug in addPointsTo or PTG.insert ...: "
               << __func__ << "\n";

    // build new pts-to sets
    PTG.toPointsToSets(B);

    if (!check(B, A))
        errs() << "pts-to sets should equal (new): " << __func__ << "\n";

    for (int I = 0; I < 10; ++I) {
        // create intersection with itself.. should do nothing
        PTG.toPointsToSets(B);

        if (!check(B, A))
            errs() << "pts-to sets should equal (inters. " << I + 1
                   << "): " << __func__ << "\n";
    }
}

static void toPointsToSets2(void)
{
    ptr::PointsToGraph PTG(PS, new ptr::AllInSelfCategory);
    ptr::PointsToSets A, B, C;

    PTGTester T(&PTG);

    // initialize graph with pts-to pairs
    addPointsTo(M, T, "a", "b");
    addPointsTo(M, T, "a", "c");
    addPointsTo(M, T, "b", "c");
    addPointsTo(M, T, "d", "d");
    addPointsTo(M, T, "e", "g");

    // build new pts-to sets
    PTG.toPointsToSets(B);

    addPointsTo(M, A, "a", "b");
    addPointsTo(M, A, "b", "c");
    addPointsTo(M, A, "a", "g");
    addPointsTo(M, A, "d", "e");
    addPointsTo(M, A, "e", "c");

    addPointsTo(M, C, "a", "b");
    addPointsTo(M, C, "b", "c");

    // create intersection of PTG with A. C should be a result
    PTG.toPointsToSets(A);

    if (!check(A, C))
        errs() << "pts-to sets: " << __func__ << "\n";
}

static void toPointsToSets3(void)
{
    ptr::PointsToGraph PTG(PS, new ptr::AllInOneCategory);
    ptr::PointsToSets A, B, C, D;

    PTGTester T(&PTG);

    // initialize graph with pts-to pairs
    addPointsTo(M, T, "a", "b");
    addPointsTo(M, T, "a", "c");
    addPointsTo(M, T, "b", "c");
    addPointsTo(M, T, "d", "c");
    addPointsTo(M, T, "e", "d");
    addPointsTo(M, T, "a", "d");
    addPointsTo(M, T, "f", "a");
    addPointsTo(M, T, "f", "e");

    addPointsTo(M, A, "f", "a");
    addPointsTo(M, A, "f", "e");
    addPointsTo(M, A, "a", "b");
    addPointsTo(M, A, "a", "c");
    addPointsTo(M, A, "a", "d");
    addPointsTo(M, A, "e", "b");
    addPointsTo(M, A, "e", "c");
    addPointsTo(M, A, "e", "d");
    addPointsTo(M, A, "b", "b");
    addPointsTo(M, A, "b", "c");
    addPointsTo(M, A, "b", "d");
    addPointsTo(M, A, "c", "b");
    addPointsTo(M, A, "c", "c");
    addPointsTo(M, A, "c", "d");
    addPointsTo(M, A, "d", "b");
    addPointsTo(M, A, "d", "c");
    addPointsTo(M, A, "d", "d");

    if (!check(T, A))
        errs() << "pts-to sets (1): " << __func__ << "\n";

    PTG.toPointsToSets(B);
    assert(!B.getContainer().empty());

    if (!check(A, B))
        errs() << "pts-to sets (2): " << __func__ << "\n";

    // this should not change the points-to sets
    PTG.toPointsToSets(B);
    assert(!B.getContainer().empty());

    PTG.toPointsToSets(B);
    PTG.toPointsToSets(B);
    PTG.toPointsToSets(B);

    if (!check(A, B))
        errs() << "pts-to sets (3): " << __func__ << "\n";
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

    // do this until we convert PointsToGraph to be a template
    assert(NODE_EDGES_NUM >= 32
	   && "Graph must be compiled with more edges to run"
	      "these tests");

    // these use Andersen analysis
    buildPointsToGraph(derefPointer1);
    buildPointsToGraph(derefPointer2);
    buildPointsToGraph(derefPointer3);
    buildPointsToGraph(derefPointee1);
    buildPointsToGraph(derefPointee2);
    buildPointsToGraph(derefPointee3);
    buildPointsToGraph(derefPointeer1);
    buildPointsToGraph(derefBoth1);
    buildPointsToGraph(derefBoth2);

    // tests with fixed categories, the first three ones are also from the
    // Shapiro-Horwitz paper (Figure 3)
    buildPointsToGraph(fixedCateg1, new ptr::FixedCategories(categories1()));
    buildPointsToGraph(fixedCateg2, new ptr::FixedCategories(categories2()));
    buildPointsToGraph(fixedCateg3, new ptr::FixedCategories(categories3()));
    buildPointsToGraph(fixedCateg4, new ptr::FixedCategories(categories4()));
    buildPointsToGraph(fixedCateg5, new ptr::FixedCategories(categories5()));

    // tests for applyRule's functions
    // use andersen-like categories
    buildPointsToGraph(applyVarAsgnAlloc);
    buildPointsToGraph(applyVarAsgnRef);
    buildPointsToGraph(applyVarAsgnVar);
    buildPointsToGraph(applyVarAsgnDeref);
    buildPointsToGraph(applyDrefVarAsgnVar);
    buildPointsToGraph(applyDrefVarAsgnRef);
    buildPointsToGraph(applyVarAsgnNull);
    buildPointsToGraph(applyDrefVarAsgnNull);
    buildPointsToGraph(applyDrefVarAsgnDref);
    buildPointsToGraph(applyDrefVarAsgnDref2);
    notTested("var asgn gep");

    // test computePointsToSets
    toPointsToSets1();
    toPointsToSets2();
    toPointsToSets3();

    std::pair<int, int>results = getResults();

    if (results.first)
        errs() << results.first << " tests from " << results.second << " failed!\n";
    else
        errs() << "All tests passed!\n";

	return results.first != 0;
}
