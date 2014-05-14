#include "PTGTester.h"
#include <llvm/ADT/MapVector.h>

using namespace llvm;

typedef PointsToGraph::Pointer Pointer;
typedef PointsToGraph::Pointer Pointee;
typedef std::pair<const char *, int64_t > TestPointer;

static int failed = 0;
static int total = 0;

static MapVector<TestPointer, Pointer> valueMap;
static std::map<const char *, Value *> llvmValues;

void dumpPointsToSets(ptr::PointsToSets& PS)
{
    ptr::PointsToSets::const_iterator I, E;
    ptr::PointsToSets::PointsToSet::const_iterator II, EE;

    for (I = PS.begin(), E = PS.end(); I != E; ++I) {
        errs() << "PTR: ";
        if (I->first.second >= 0)
            errs() << "+" << I->first.second << " ";
        I->first.first->dump();

        for (II = I->second.cbegin(), EE = I->second.cend(); II != EE; ++II) {
            errs() << "    --> ";
            if (II->second >= 0)
                errs() << "+" << II->second << " ";
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
Pointer getPointer(Module *M, const char *name, int64_t off)
{
    Value *va;

    MapVector<TestPointer, Pointer>::iterator E = valueMap.end();
    Pointer& p = valueMap[TestPointer(name, off)];

    // if the last operation increased the size of valueMap, then
    // the pointer does not exist
    if (E != valueMap.end()) {
        // use always the same llvm value
        std::map<const char *, Value *>::iterator VI = llvmValues.find(name);
        if (VI == llvmValues.end()) {
            if (strcmp(name, "null") == 0)
                va = ConstantPointerNull::get(llvm::Type::getInt32PtrTy(M->getContext()));
            else
                va = new GlobalVariable(*M, IntegerType::get(M->getContext(), 32),
                                        false,
                                        GlobalValue::CommonLinkage, 0 , name);
            llvmValues.insert(std::make_pair(name, va));
        } else {
            va = VI->second;
        }

	// copy it to map (p is reference)
        p = Pointer(va, off);
    }

    return p;
}

void addPointsTo(Module *M, PTGTester &PTG,
                 const char *a, const char *b,
                 enum deref derefFlag,
                 int64_t aoff, int64_t boff)
{
    Pointer pa, pb;

    pa = getPointer(M, a, aoff);
    pb = getPointer(M, b, boff);

    switch (derefFlag) {
    case DEREF_NONE:
        PTG.insert(pa, pb); break;
    case DEREF_POINTEE:
        PTG.insertDerefPointee(pa, pb); break;
    case DEREF_POINTER:
        PTG.insertDerefPointer(pa, pb); break;
    case DEREF_BOTH:
        PTG.insertDerefBoth(pa, pb); break;
    default:
        assert(0);
    }
}

void addPointsTo(Module *M, ptr::PointsToSets& PTSets,
                 const char *a, const char *b,
                 int64_t aoff, int64_t boff)
{
    Pointer p = getPointer(M, a, aoff);
    Pointee l = getPointer(M, b, boff);

    ptr::PointsToSets::PointsToSet& S = PTSets[p];
    S.insert(l);
}

bool check(ptr::PointsToSets &A, ptr::PointsToSets &B)
{

    ++total;

    if (!comparePointsToSets(A, B)) {
        ++failed;

        errs() << "\nPoints-to sets:\n\n";
        dumpPointsToSets(A);
        errs() << "should be\n";
        dumpPointsToSets(B);
        errs() << "\n";

        return false;
    }

    return true;
}


bool check(PTGTester &PTG, ptr::PointsToSets &S)
{
    ptr::PointsToSets PTGSet;

    PTG.getPTG().toPointsToSets(PTGSet);

    if (!check(PTGSet, S)) {
        errs() << "PTG:\n";
        PTG.getPTG().dump();
	return false;
    }

    return true;
}

std::pair<int, int> getResults(void)
{
    return std::pair<int, int>(failed, total);
}

void notTested(const char *msg)
{
    errs() << "NOT TESTED: "<< msg << "\n";
    ++total;
    ++failed;
}

namespace llvm {
namespace ptr {

unsigned int FixedCategories::getCategory(PointsToGraph::Pointer a) const
{
    std::set<std::set<PointsToGraph::Pointer> >::const_iterator I, E;

    unsigned int num = 0;
    for (I = Categories.cbegin(), E = Categories.cend(); I != E; ++I) {
        if (I->count(a))
	    return num;

	++num;
    }

    return num;
}

unsigned int AllInSelfCategory::getCategory(PointsToGraph::Pointer a) const
{
    MapVector<TestPointer, Pointer>::const_iterator I, E;
    unsigned int num = 0;

    for (I = valueMap.begin(), E = valueMap.end(); I != E; ++I) {
	if (I->second == a) {
		return num;
	}
	++num;
    }

    // if the pointer does not exist (in the case of GEP), create it
    assert(a.second >= 0);
    valueMap[TestPointer(a.first->getName().data(), a.second)] = a;

    return num;
}

} // namespace ptr
} // namespace llvm
