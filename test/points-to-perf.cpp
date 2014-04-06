#include <llvm/LLVMContext.h>
#include <llvm/Function.h>
#include <llvm/Module.h>
#include <llvm/Support/IRReader.h>
#include <llvm/Support/raw_ostream.h>
#include <ctime>

#include "../src/PointsTo/PointsTo.h"

using namespace llvm;
using ptr::PointsToSets;

typedef PointsToSets::PointsToSet PTSet;

static void add(long long unsigned int& sum, long unsigned cur, int N)
{
    long long unsigned int prev_sum = sum;
    sum += cur / N;

    if (prev_sum > sum) {
        errs() << "Overflow!\n";
        exit(1);
    }
}

static long long unsigned pointsToPerf(Module &M, int N, unsigned K)
{
    ptr::ProgramStructure P(M);
    clock_t start, end, diff;
    long long unsigned int sum = 0;
    long unsigned cur = 0;

    for (int I = 0; I < N; ++I) {
        PointsToSets PS;

        // take measurement
        start = clock();
        computePointsToSets(P, PS, K);
        end = clock();

        diff = end - start;

        // (a1 + a2 + ... + an)    a1     a2           an
        // -------------------- = ---- + ---- + ... + ----
        //          n               n      n            n
        if ((1LL * cur) + diff >= LONG_MAX) {
            add(sum, cur, N);
            cur = 0;
        } else {
            cur += diff;
        }
    }

    // add last accumulation
    add(sum, cur, N);
    return sum;
}

static long int countPairs(const PointsToSets& PS)
{
    long int pairs = 0;

    PointsToSets::const_iterator I, E;

    for (I = PS.begin(), E = PS.end(); I != E; ++I)
        pairs += I->second.size();

    return pairs;
}

int main(int argc, char **argv)
{
    LLVMContext context;
    SMDiagnostic SMD;
    Module *M;
    long long int Measurement;
    int N = 1000, K = 0;

    if (argc == 1) {
        errs() << "Usage: program input.bc [-n runs_no] [-k K]\n";
        SMD.print(argv[0], errs());
        return 1;
    }

    // handle options
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "-k") == 0)
            if (i + 1 < argc)
                if (strcmp(argv[i + 1], "andersen") == 0)
                    K = UINT_MAX;
                else
                    K = atoi(argv[i + 1]);
            else
                errs() << "Wrong K\n";
        else if (strcmp(argv[i], "-n") == 0)
            if (i + 1 < argc)
                N = atoi(argv[i + 1]);
            else
                errs() << "Wrong N\n";
    }

    M = ParseIRFile(argv[1], SMD, context);
    if (!M) {
        SMD.print(argv[0], errs());
        return 1;
    }

    // compute performance
    Measurement = pointsToPerf(*M, N, K);
    errs() << (double) Measurement / CLOCKS_PER_SEC;

    // compute number of points-to pairs
    PointsToSets PS;
    {
        ptr::ProgramStructure P(*M);
        computePointsToSets(P, PS, K);
    }

    errs() << " " << countPairs(PS) << "\n";

    delete M;

    return 0;
}
