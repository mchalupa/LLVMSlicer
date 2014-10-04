#include <llvm/LLVMContext.h>
#include <llvm/Function.h>
#include <llvm/Module.h>
#include <llvm/Support/IRReader.h>
#include <llvm/Support/raw_ostream.h>

#include "../src/PointsTo/PointsTo.h"

using namespace llvm;

typedef ptr::PointsToSets::PointsToSet PTSet;

static void pointsTo(const PTSet &P)
{
	for (PTSet::const_iterator I = P.begin(), E = P.end(); I != E; ++I) {
		errs() << "\t OFF=" << I->offset << " of";
		I->location->dump();
	}
}

static void pointsTo(const Value &val, const ptr::PointsToSets &PS)
{
	pointsTo(ptr::getPointsToSet(&val, PS));
}

static void pointsTo(Module &M)
{
	ptr::PointsToSets PS;
	{
		ptr::ProgramStructure P(M);
		computePointsToSets(P, PS);
	}

	for (Module::const_iterator I = M.begin(), E = M.end(); I != E; ++I) {
		const Function &F = *I;
		if (F.isDeclaration())
			continue;
		errs() << "=========== " << F.getName() << "\n";
		for (const_inst_iterator II = inst_begin(F), EE = inst_end(F);
				II != EE; ++II) {
			const Instruction *ins = &*II;
			errs() << "PTSTO";

			if (const StoreInst *SI = dyn_cast<StoreInst>(ins)) {
				errs() << " STORE";
				SI->dump();
				pointsTo(*SI->getPointerOperand(), PS);
			} else {
				ins->dump();
				pointsTo(*ins, PS);
			}
		}
		for (ptr::PointsToSets::const_iterator II = PS.begin(),
				EE = PS.end(); II != EE; ++II) {
			const ptr::Pointer &Ptr = II->first;
			const PTSet &P = II->second;

			if (Ptr.offset == -1)
				continue;
			if (const Instruction *ins =
					dyn_cast<Instruction>(Ptr.location)) {
				if (ins->getParent()->getParent() != &F)
					continue;
				errs() << "PTSTO w/ OFF=" << Ptr.offset;
				Ptr.location->dump();
				pointsTo(P);
			}
		}
	}
}

int main(int argc, char **argv)
{
	LLVMContext context;
	SMDiagnostic SMD;
	Module *M;

	M = ParseIRFile(argv[1], SMD, context);
	if (!M) {
		SMD.print(argv[0], errs());
		return 1;
	}

	pointsTo(*M);

	delete M;

	return 0;
}
