#include <llvm/LLVMContext.h>
#include <llvm/Function.h>
#include <llvm/Module.h>
#include <llvm/Support/IRReader.h>
#include <llvm/Support/raw_ostream.h>

#include "../src/PointsTo/PointsTo.h"

using namespace llvm;

typedef ptr::PointsToGraph::Pointer Pointer;
typedef ptr::PointsToGraph::Pointee Pointee;

static ptr::ProgramStructure *PS;

static void dumpPointsToSets(ptr::PointsToGraph &PTG)
{
    ptr::PointsToSets::const_iterator I, E;
    ptr::PointsToSets::PointsToSet::const_iterator II, EE;

    ptr::PointsToSets PS;
    PTG.toPointsToSets(PS);

    for (I = PS.begin(), E = PS.end(); I != E; ++I) {
        errs() << "PTR: ";
        I->first.first->dump();

        for (II = I->second.cbegin(), EE = I->second.cend(); II != EE; ++II) {
            errs() << "    --> ";
            II->first->dump();
        }
    }
}

static void addPointsTo(Module &M, ptr::PointsToGraph &PTG,
                        const char *a, const char *b)
{
    static std::map<const char *, Pointer> valueMap;
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

    PTG.insert(pa, pb);
}

static void buildPointsToGraph(Module &M, void (*seq)(Module&, ptr::PointsToGraph&))
{
    ptr::PointsToCategories *categ = new ptr::AllInOneCategory();
    ptr::PointsToGraph PTG(*PS, categ);

    seq(M, PTG);

    errs() << "Points-to graph:\n\n";
    PTG.dump();
    errs() << "\nAssocaited points-to set:\n\n";
    dumpPointsToSets(PTG);
    errs() << "\n";
}

void figure1(Module& M, ptr::PointsToGraph& PTG)
{
    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "b", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "d", "e");
}

void figure2(Module& M, ptr::PointsToGraph& PTG)
{
    addPointsTo(M, PTG, "x", "v1");
    addPointsTo(M, PTG, "x", "v2");
    addPointsTo(M, PTG, "y", "v1");
}

void figure3(Module& M, ptr::PointsToGraph& PTG)
{
    addPointsTo(M, PTG, "a", "b");
    addPointsTo(M, PTG, "a", "c");
    addPointsTo(M, PTG, "a", "d");
    addPointsTo(M, PTG, "c", "d");
}



int main(int argc, char **argv)
{
	LLVMContext context;
	SMDiagnostic SMD;
	Module M("points-to-test", getGlobalContext());

    ptr::ProgramStructure ps(M);
    PS = &ps;

    errs() << "--== figure1 ==-- \n";
    errs() << "===============================================\n";
    buildPointsToGraph(M, figure1);

    errs() << "--== figure2 ==-- \n";
    errs() << "===============================================\n";
    buildPointsToGraph(M, figure2);

    errs() << "--== figure3 ==-- \n";
    errs() << "===============================================\n";
    buildPointsToGraph(M, figure3);

	return 0;
}
