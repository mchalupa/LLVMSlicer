// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.

#include <map>
#include <unordered_map>

#include "llvm/BasicBlock.h"
#include "llvm/DataLayout.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Instruction.h"
#include "llvm/Instructions.h"
#include "llvm/Module.h"

#include "PointsTo.h"
#include "RuleExpressions.h"

#include "../Languages/LLVM.h"

namespace llvm { namespace ptr { namespace detail {

class CallMaps {
private:
  /* return type -> function */
  typedef std::multimap<const Type *, const Function *> FunctionsMap;
  typedef std::multimap<const Type *, const CallInst *> CallsMap;

public:
  CallMaps(const Module &M) {
    buildCallMaps(M);
  }

  template <typename OutIterator>
  void collectCallRuleCodes(const CallInst *c, const Function *f,
      OutIterator out);

  template <typename OutIterator>
  void collectCallRuleCodes(const CallInst *c, OutIterator out);

  template <typename OutIterator>
  void collectReturnRuleCodes(const ReturnInst *r, OutIterator out);

private:
  FunctionsMap FM;
  CallsMap CM;

  static bool compatibleTypes(const Type *t1, const Type *t2);
  static bool compatibleFunTypes(const FunctionType *f1,
      const FunctionType *f2);
  static RuleCode argPassRuleCode(const Value *l, const Value *r);
  void buildCallMaps(const Module &M);
};

RuleCode CallMaps::argPassRuleCode(const Value *l, const Value *r)
{
    if (isa<ConstantPointerNull const>(r))
	return ruleCode(ruleVar(l) = ruleNull(r));
    if (hasExtraReference(l))
	if (hasExtraReference(r))
	    return ruleCode(ruleVar(l) = ruleVar(r));
	else
	    return ruleCode(ruleVar(l) = *ruleVar(r));
    else
	if (hasExtraReference(r))
	    return ruleCode(ruleVar(l) = &ruleVar(r));
	else
	    return ruleCode(ruleVar(l) = ruleVar(r));
}

template <typename OutIterator>
void CallMaps::collectCallRuleCodes(const CallInst *c, const Function *f,
    OutIterator out) {
  assert(!isInlineAssembly(c) && "Inline assembly is not supported!");

  if (memoryManStuff(f) && !isMemoryAllocation(f))
    return;

  if (isMemoryAllocation(f)) {
    const Value *V = c;
    *out++ = ruleCode(ruleVar(V) = ruleAllocSite(V));
  } else {
    static unsigned warned = 0;
    Function::const_arg_iterator fit = f->arg_begin();
    unsigned callNumOperands = c->getNumArgOperands();
    size_t i = 0;

    for (; fit != f->arg_end() && i < callNumOperands; ++fit, ++i)
      if (isPointerValue(&*fit))
	*out++ = argPassRuleCode(&*fit, elimConstExpr(c->getOperand(i)));

    if (i < callNumOperands && warned++ < 3) {
      errs() << __func__ << ": skipped some vararg arguments in '" <<
	f->getName() << "(" << i << ", " << callNumOperands << ")'\n";
    }
  }
}

bool CallMaps::compatibleTypes(const Type *t1, const Type *t2) {

  /*
   * Casting sucks, we can call (int *) with (char *) parameters.
   * Let's over-approximate.
   */
  if (t1->isPointerTy() && t2->isPointerTy())
    return true;

  return t1 == t2;
}

bool CallMaps::compatibleFunTypes(const FunctionType *f1,
		const FunctionType *f2) {

  unsigned params1 = f1->getNumParams();
  unsigned params2 = f2->getNumParams();

  if (!f1->isVarArg() && !f2->isVarArg() && params1 != params2)
    return false;

  if (!compatibleTypes(f1->getReturnType(), f2->getReturnType()))
    return false;

  for (int i = 0; i < params1 && i < params2; i++)
    if (!compatibleTypes(f1->getParamType(i), f2->getParamType(i)))
      return false;

  return true;
}

template<typename OutIterator>
void CallMaps::collectCallRuleCodes(const CallInst *c, OutIterator out) {

    if (const Function *f = c->getCalledFunction()) {
      collectCallRuleCodes(c, f, out);
      return;
    }

    const FunctionType *funTy = getCalleePrototype(c);
    const Type *retTy = funTy->getReturnType();

    for (FunctionsMap::const_iterator I = FM.lower_bound(retTy),
	E = FM.upper_bound(retTy); I != E; ++I) {
      const Function *fun = I->second;

      if (compatibleFunTypes(funTy, fun->getFunctionType()))
	collectCallRuleCodes(c, fun, out);
    }
}

template<typename OutIterator>
void CallMaps::collectReturnRuleCodes(const ReturnInst *r, OutIterator out) {
  const Value *retVal = r->getReturnValue();

  if (!retVal || !isPointerValue(retVal))
    return;

  const Function *f = r->getParent()->getParent();
  const FunctionType *funTy = f->getFunctionType();
  const Type *retTy = funTy->getReturnType();

  for (CallsMap::const_iterator b = CM.lower_bound(retTy),
      e = CM.upper_bound(retTy); b != e; ++b) {
    const CallInst *CI = b->second;

    if (const Function *g = CI->getCalledFunction()) {
      if (f == g)
	*out++ = argPassRuleCode(CI, retVal);
    } else if (compatibleFunTypes(funTy, getCalleePrototype(CI)))
	*out++ = argPassRuleCode(CI, retVal);
  }
}

void CallMaps::buildCallMaps(const Module &M) {
    for (Module::const_iterator f = M.begin(); f != M.end(); ++f) {
	if (!f->isDeclaration()) {
	    const FunctionType *funTy = f->getFunctionType();

	    FM.insert(std::make_pair(funTy->getReturnType(), &*f));
	}

	for (const_inst_iterator i = inst_begin(f), E = inst_end(f);
		i != E; ++i) {
	    if (const CallInst *CI = dyn_cast<CallInst>(&*i)) {
		if (!isInlineAssembly(CI) && !callToMemoryManStuff(CI)) {
		    const FunctionType *funTy = getCalleePrototype(CI);

		    CM.insert(std::make_pair(funTy->getReturnType(), CI));
		}
	    } else if (const StoreInst *SI = dyn_cast<StoreInst>(&*i)) {
		const Value *r = SI->getValueOperand();

		if (hasExtraReference(r) && memoryManStuff(r)) {
		    const Function *fn = dyn_cast<Function>(r);
		    const FunctionType *funTy = fn->getFunctionType();

		    FM.insert(std::make_pair(funTy->getReturnType(), fn));
		}
	    }
	}
    }
}

}}}

namespace llvm {
namespace ptr {

PointsToGraph::~PointsToGraph()
{
    std::unordered_map<Pointer, Node *>::iterator I, E;

    for (I = Nodes.begin(), E = Nodes.end(); I != E; ++I) {
        if (I->second) {
            replaceNode(I->second, NULL);
            delete I->second;
        }
    }

    // PointsToGraph adopts the category, since it must be
    // allocated on heap because of virtual functions
    delete PTC;
}

static void printPtrName(const PointsToGraph::Pointee p)
{
    const llvm::LoadInst *LInst;
    const llvm::Value *val = p.first;

    if (isa<CastInst>(p.first)) {
        errs() << "BT: ";
        val = val->stripPointerCasts();
    } else if ((LInst = dyn_cast<LoadInst>(val))) {
        errs() << "LD: ";
        val = LInst->getPointerOperand();
    }

	if (isa<GlobalValue>(val))
		errs() << "@";
    else
		errs() << "%";



    if (val->hasName())
	    errs() << val->getName().data();
    else
        errs() << val->getValueID();

    if (p.second >= 0)
        errs() << " + " << p.second;
}

void PointsToGraph::Node::dump(void) const
{
    std::set<Pointee>::const_iterator Begin;
    std::set<Pointee>::const_iterator I, E;

    Begin = I = Elements.cbegin();

    errs() << "[";

    for (E = Elements.cend(); I != E; ++I) {
        if (I != Begin)
            errs() << ", ";

        printPtrName(*I);
    }

    errs() << "]\n";
}

void PointsToGraph::dump(void) const
{
    std::unordered_map<Pointer, Node *>::const_iterator I, E;
    std::set<PointsToGraph::Node *>::const_iterator II, EE;

    if (Nodes.empty()) {
        errs() << "PointsToGraph is empty\n";
        return;
    }

    for(I = Nodes.begin(), E = Nodes.end(); I != E; ++I) {
        if (!I->second)
            continue;

        I->second->dump();

        for (II = (I->second)->getEdges().begin(),
             EE = (I->second)->getEdges().end();
                II != EE; ++II) {
            errs() << "    --> ";
            (*II)->dump();
        }
    }
}

void PointsToGraph::replaceNode(PointsToGraph::Node *a,
                                PointsToGraph::Node *b)
{
    std::set<Pointee>::iterator I, E;
    std::set<Pointee>& Elements = a->getElements();

    // we must zero out all occurences in the map
    for (I = Elements.begin(), E = Elements.end(); I != E; ++I) {
        Node *&n = Nodes[*I];
        /*
        if (n != a) {
            errs() << "Pointer ";
            printPtrName(*I);
            errs() << " has set wrong node. Should be ";
            a->dump();
            errs() << "but is ";
            if (n)
                n->dump();
            else
                errs() << "NULL\n";
        }
        */
        n = b;
    }
}

inline PointsToGraph::Node *PointsToGraph::findNode(Pointee p) const
{
    std::unordered_map<Pointer, Node *>::const_iterator I;
    I = Nodes.find(p);

    if (I == Nodes.end())
        return NULL;
    else
        return I->second;
}

// take nodes outgoing from root and check if pointee p
// should be added into one of them
PointsToGraph::Node *
PointsToGraph::shouldAddTo(PointsToGraph::Node *root, Pointee p)
{
    std::set<Node *>::const_iterator I, E;
    I = root->getEdges().cbegin();
    E = root->getEdges().cend();

    for (; I != E; ++I)
        // since node can contain only elements from the same category
        // it's sufficent to check only one element from each node
        if (PTC->areInSameCategory(*((*I)->getElements().cbegin()), p))
            return *I;

    return NULL;
}

inline PointsToGraph::Node *PointsToGraph::addNode(Pointee p)
{
    PointsToGraph::Node *n = new PointsToGraph::Node(p);
    Nodes[p] = n;

    return n;
}

// get or create new node
// it accesses Nodes only once contrary to findNode() + addNode()
inline PointsToGraph::Node *PointsToGraph::getNode(Pointee P)
{
    // hmm, is this defined? If Node(P) does not exit,
    // it could return some garbage.. Anyway, it looks like
    // working
    Node *&n = Nodes[P];

    if (!n)
        n = new Node(P);

    return n;
}

bool PointsToGraph::insert(Pointer p, Pointee location)
{
    bool changed = false;

    // find node that contains pointer p. From this node will
    // be created new outgoing edge (if needed)
    PointsToGraph::Node *From = NULL, *To = NULL;

    From = getNode(p);
    To = shouldAddTo(From, location);

    if (To) {
        ///
        // insert location into existing node if it's appropriated
        ///

        changed = To->insert(location);
        Nodes[location] = To;
    } else {
        To = getNode(location);
        changed = From->addNeighbour(To);
    }

    return changed;
}

bool PointsToGraph::insert(Pointer p, std::set<Pointee>& locations)
{
    std::set<Pointee>::iterator I, E;
    bool changed = false;

    for (I = locations.begin(), E = locations.end(); I != E; ++I)
        changed |= insert(p, *I);

    return changed;
}

bool PointsToGraph::insertDerefPointee(Node *PointerNode, Node *LocationNode)
{
    bool changed = false;

    std::set<PointsToGraph::Node *>::iterator I, E;
    std::set<PointsToGraph::Node *>& Edges = LocationNode->getEdges();

    for (I = Edges.begin(), E = Edges.end(); I != E; ++I)
       changed |= PointerNode->addNeighbour(*I);

    return changed;
}

bool PointsToGraph::insertDerefPointee(Pointer p, Node *LocationNode)
{
    if (!LocationNode->hasNeighbours())
        return false;

    return insertDerefPointee(getNode(p), LocationNode);
}

bool PointsToGraph::insertDerefPointee(Pointer p, Pointee location)
{
    PointsToGraph::Node *LocationNode;
    bool changed = false;

    LocationNode = findNode(location);

    if (!LocationNode) {
        // if location do not have a node yet, then it do not have
        // neighbours which should be inserted.
        // Do NOT add p->location into graph, because this functions
        // should add p->*location, which is something different.
        return false;
    }

    return insertDerefPointee(p, LocationNode);
}

bool PointsToGraph::insertDerefPointer(Node *PointerNode, Node *LocationNode)
{
    bool changed = false;

    std::set<PointsToGraph::Node *>::iterator I, E;
    std::set<PointsToGraph::Node *>& Edges = PointerNode->getEdges();

    for (I = Edges.begin(), E = Edges.end(); I != E; ++I)
        changed |= (*I)->addNeighbour(LocationNode);

    return changed;
}

bool PointsToGraph::insertDerefPointer(Node *PointerNode, Pointee location)
{
    if (!PointerNode->hasNeighbours())
        return false;

    return insertDerefPointer(PointerNode, getNode(location));
}


bool PointsToGraph::insertDerefPointer(Pointer p, Pointee location)
{
    PointsToGraph::Node *PointerNode;
    bool changed = false;

    PointerNode = findNode(p);

    if (!PointerNode)
        return false;

    return insertDerefPointer(PointerNode, location);
}

bool PointsToGraph::insertDerefBoth(Node *PointerNode, Node *LocationNode)
{
    bool changed = false;

    std::set<PointsToGraph::Node *>::iterator I, E;
    std::set<PointsToGraph::Node *>& Edges = PointerNode->getEdges();

    for (I = Edges.begin(), E = Edges.end(); I != E; ++I)
        changed |= insertDerefPointee(*I, LocationNode);

    return changed;
}

void PointsToGraph::Node::convertToPointsToSets(PointsToSets& PS,
                                                bool intersect) const
{
    typedef PointsToSets::PointsToSet PTSet;
    typedef PointsToSets::Pointer Ptr;

    std::set<Pointee>::const_iterator ElemI, ElemE;
    std::set<Node *>::const_iterator EdgesI, EdgesE;

    for (ElemI = Elements.cbegin(), ElemE = Elements.cend();
            ElemI != ElemE; ++ElemI) {

        PTSet& S = PS[*ElemI];
        PTSet TmpPTSet;

        for (EdgesI = Edges.cbegin(), EdgesE = Edges.cend();
             EdgesI != EdgesE; ++EdgesI) {
            const PTSet& Ptees = (*EdgesI)->getElements();

            if (intersect) {
                // when creating intersection, add the intersection into
                // TmpPTSet (if there are more outgoing edges from a node,
                // then I need to accumulate intersections from all these nodes
                std::set_intersection(S.begin(), S.end(),
                                      Ptees.begin(), Ptees.end(),
                                      std::inserter(TmpPTSet, TmpPTSet.end()));

            } else {
                std::copy(Ptees.begin(), Ptees.end(),
                          std::inserter(S, S.end()));
            }
        }

        if (intersect) {
            // store accumulated intersection in original pt set
            S.swap(TmpPTSet);

            // clean empty nodes. Empty nodes can be created only by
            // intersection
            if (S.empty())
                PS.getContainer().erase(PS.find(*ElemI));
        }
    }
}

PointsToSets& PointsToGraph::toPointsToSets(PointsToSets& PS) const
{
    std::unordered_map<Pointer, Node *>::const_iterator I, E;
    bool intersect = !PS.getContainer().empty();

    for (I = Nodes.cbegin(), E = Nodes.cend(); I != E; ++I)
        if (I->second && (I->second)->hasNeighbours())
            (I->second)->convertToPointsToSets(PS, intersect);

    return PS;
}

} // namespace ptr
} // namespace llvm

namespace llvm { namespace ptr {

typedef PointsToSets::PointsToSet PTSet;
typedef PointsToSets::Pointer Ptr;

bool PointsToGraph::applyRule(ASSIGNMENT<
                                VARIABLE<const llvm::Value *>,
                                VARIABLE<const llvm::Value *>
                              > const& E)
{
    const llvm::Value *lval = E.getArgument1().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument();

    return insertDerefPointee(Ptr(lval, -1), Ptr(rval, -1));
}

static int64_t accumulateConstantOffset(const GetElementPtrInst *gep,
	const DataLayout &DL, bool &isArray) {
    int64_t off = 0;

    for (gep_type_iterator GTI = gep_type_begin(gep), GTE = gep_type_end(gep);
	    GTI != GTE; ++GTI) {
	ConstantInt *OpC = dyn_cast<ConstantInt>(GTI.getOperand());
	if (!OpC) /* skip non-const array indices */
	    continue;
	if (OpC->isZero())
	    continue;

	int64_t ElementIdx = OpC->getSExtValue();

	// Handle a struct index, which adds its field offset to the pointer.
	if (StructType *STy = dyn_cast<StructType>(*GTI)) {
	    const StructLayout *SL = DL.getStructLayout(STy);
	    off += SL->getElementOffset(ElementIdx);
	    continue;
	} else if (SequentialType *STy = dyn_cast<SequentialType>(*GTI)) {
	    off += ElementIdx * DL.getTypeStoreSize(GTI.getIndexedType());
	    isArray = true;
	    continue;
	}
#ifdef FIELD_DEBUG
	errs() << "skipping " << OpC->getValue() << " in ";
	gep->dump();
#endif
    }

    return off;
}

static bool checkOffset(const DataLayout &DL, const Value *Rval, uint64_t sum) {
  if (const GlobalVariable *GV = dyn_cast<GlobalVariable>(Rval)) {
    if (GV->hasInitializer() &&
	sum >= DL.getTypeAllocSize(GV->getInitializer()->getType()))
      return false;
  } else if (const AllocaInst *AI = dyn_cast<AllocaInst>(Rval)) {
    if (!AI->isArrayAllocation() &&
	sum >= DL.getTypeAllocSize(AI->getAllocatedType()))
      return false;
  }

  return true;
}

bool PointsToGraph::applyRule(const llvm::DataLayout &DL,
                              ASSIGNMENT<
                                VARIABLE<const llvm::Value *>,
                                GEP<VARIABLE<const llvm::Value *> >
                              > const& E)
{
    const llvm::Value *lval = E.getArgument1().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument().getArgument();
    bool changed = false;

    const GetElementPtrInst *gep = dyn_cast<GetElementPtrInst>(rval);
    const llvm::Value *op = elimConstExpr(gep->getPointerOperand());
    bool isArray = false;
    int64_t off = accumulateConstantOffset(gep, DL, isArray);
    Ptr L = Ptr(lval, -1);

    if (hasExtraReference(op)) {
        changed = insert(L, Ptr(op, off)); /* VAR = REF */
    } else { /* VAR = VAR */
        Ptr R(op, -1);
        Node *n = findNode(R);

        if (!n)
            return false;

        if (!n->hasNeighbours())
            return false;

        std::set<Node *>& Edges = n->getEdges();
        std::set<Node *>::const_iterator NI, NE;
        std::set<Pointee>::const_iterator PI, PE;
        for (NI = Edges.cbegin(), NE = Edges.cend(); NI != NE; ++NI) {
            std::set<Pointee>& Elems = (*NI)->getElements();
            for (PI = Elems.cbegin(), PE = Elems.cend(); PI != PE; ++PI) {
                assert(PI->second >= 0);

                const Value *val = PI->first;

                if (off && (isa<Function>(val) || isa<ConstantPointerNull>(val)))
                    continue;

                int64_t sum = PI->second + off;

                if (!checkOffset(DL, val, sum))
                    continue;

                // XXX crop > 64
                if (sum < 0) {
                    assert (PI->second >= 0);
                    sum = 0;
                }

                /* an unsoudness :) */
                if (isArray && sum > 64)
                    sum = 64;

                changed |= insert(L, Ptr(val, sum));
            }
        }
    }

    return changed;
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                VARIABLE<const llvm::Value *>,
                                REFERENCE<VARIABLE<const llvm::Value *> >
                              > const& E)
{
    const llvm::Value *lval = E.getArgument1().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument().getArgument();

    return insert(Ptr(lval, -1), Ptr(rval, 0));
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                VARIABLE<const llvm::Value *>,
                                DEREFERENCE< VARIABLE<const llvm::Value *> >
                              > const& E, const int idx)
{
    const llvm::Value *lval = E.getArgument1().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument().getArgument();
    bool change = false;

    Ptr L(lval, idx);

    Node *r = findNode(Ptr(rval, -1));
    if (!r)
        return false;

    std::set<Node *>::const_iterator II, EE;
    std::set<Node *>& Edges = r->getEdges();
    for (II = Edges.cbegin(), EE = Edges.cend(); II != EE; ++II)
        // must process nodes *two* steps away
        change |= insertDerefPointee(L, *II);

    return change;
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                DEREFERENCE<VARIABLE<const llvm::Value *> >,
                                VARIABLE<const llvm::Value *>
                              > const& E)
{
    const llvm::Value *lval = E.getArgument1().getArgument().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument();

    Node *l = findNode(Ptr(lval, -1));
    if (!l)
        return false;

    Node *r = findNode(Ptr(rval, -1));
    if (!r)
         return false;

    return insertDerefBoth(l, r);
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                DEREFERENCE<VARIABLE<const llvm::Value *> >,
                                REFERENCE<VARIABLE<const llvm::Value *> >
                              > const &E)
{
    const llvm::Value *lval = E.getArgument1().getArgument().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument().getArgument();

    Node *l = findNode(Ptr(lval, -1));
    if (!l)
        return false;

    return insertDerefPointer(l, Ptr(rval, 0));
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                DEREFERENCE<VARIABLE<const llvm::Value *> >,
                                DEREFERENCE<VARIABLE<const llvm::Value *> >
                              > const& E)
{
    const llvm::Value *lval = E.getArgument1().getArgument().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument().getArgument();
    bool change = false;

    Node *l = findNode(Ptr(lval, -1));
    if (!l)
        return false;

    Node *r = findNode(Ptr(rval, -1));
    if (!r)
        return false;

    std::set<Node *>::const_iterator II, EE;
    // copy current edges, because we can change these edges, but
    // we want to work only with these edges
    // XXX don't we need copying even when dereferencing only one side??
    std::set<Node *> Edges = r->getEdges();
    for (II = Edges.cbegin(), EE = Edges.cend(); II != EE; ++II)
        change |= insertDerefBoth(l, *II);

    return change;
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                VARIABLE<const llvm::Value *>,
                                ALLOC<const llvm::Value *>
                              > const &E)
{
    const llvm::Value *lval = E.getArgument1().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument();

    return insert(Ptr(lval, -1), Ptr(rval, 0));
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                VARIABLE<const llvm::Value *>,
                                NULLPTR<const llvm::Value *>
                              > const &E)
{
    const llvm::Value *lval = E.getArgument1().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument();

    assert(isa<ConstantPointerNull>(rval) && "Not a NULL");

    return insert(Ptr(lval, -1), Ptr(rval, 0));
}

bool PointsToGraph::applyRule(ASSIGNMENT<
                                DEREFERENCE<VARIABLE<const llvm::Value *> >,
                                NULLPTR<const llvm::Value *>
                              > const &E)
{
    const llvm::Value *lval = E.getArgument1().getArgument().getArgument();
    const llvm::Value *rval = E.getArgument2().getArgument();

    assert(isa<ConstantPointerNull>(rval) && "Not a NULL");

    Node *l = findNode(Ptr(lval, -1));
    if (!l)
        return false;

    return insertDerefPointer(l, Ptr(rval, 0));
}

bool PointsToGraph::applyRule(DEALLOC<const llvm::Value *>) {
    return false;
}

bool PointsToGraph::applyRules(const RuleCode &RC, const llvm::DataLayout &DL)
{
    const llvm::Value *lval = RC.getLvalue();
    const llvm::Value *rval = RC.getRvalue();

    switch (RC.getType()) {
    case RCT_VAR_ASGN_ALLOC:
        return applyRule((ruleVar(lval) = ruleAllocSite(rval)).getSort());
    case RCT_VAR_ASGN_NULL:
        return applyRule((ruleVar(lval) = ruleNull(rval)).getSort());
    case RCT_VAR_ASGN_VAR:
        return applyRule((ruleVar(lval) = ruleVar(rval)).getSort());
    case RCT_VAR_ASGN_GEP:
        return applyRule(DL,
                        (ruleVar(lval) = ruleVar(rval).gep()).getSort());
    case RCT_VAR_ASGN_REF_VAR:
        return applyRule((ruleVar(lval) = &ruleVar(rval)).getSort());
    case RCT_VAR_ASGN_DREF_VAR:
        return applyRule((ruleVar(lval) = *ruleVar(rval)).getSort());
    case RCT_DREF_VAR_ASGN_NULL:
        return applyRule((*ruleVar(lval) = ruleNull(rval)).getSort());
    case RCT_DREF_VAR_ASGN_VAR:
        return applyRule((*ruleVar(lval) = ruleVar(rval)).getSort());
    case RCT_DREF_VAR_ASGN_REF_VAR:
        return applyRule((*ruleVar(lval) = &ruleVar(rval)).getSort());
    case RCT_DREF_VAR_ASGN_DREF_VAR:
        return applyRule((*ruleVar(lval) = *ruleVar(rval)).getSort());
    case RCT_DEALLOC:
        return applyRule(ruleDeallocSite(RC.getValue()).getSort());
    default:
        assert(0 && "Unknown rule code");
    }
}

/*
 * It does not really work -- it prunes too much. Like it does not take into
 * account bitcast instructions in the code.
 */
static PointsToSets &pruneByType(PointsToSets &S) {
  typedef PointsToSets::mapped_type PTSet;
  for (PointsToSets::iterator s = S.begin(); s != S.end(); ) {
      const llvm::Value *first = s->first.first;
      if (llvm::isa<llvm::Function>(first)) {
	const PointsToSets::iterator tmp = s++;
	S.getContainer().erase(tmp);
      } else {
#if 0
	if (isPointerValue(first)) {
	  const llvm::Type *firstTy;
	  if (const llvm::BitCastInst *BC =
		      llvm::dyn_cast<llvm::BitCastInst>(first))
	    firstTy = getPointedType(BC->getSrcTy());
	  else
	    firstTy = getPointedType(first);

	  for (typename PTSet::const_iterator v = s->second.begin();
	       v != s->second.end(); ) {
	    const llvm::Value *second = *v;
	    const llvm::Type *secondTy = second->getType();

	    if (hasExtraReference(second))
		    secondTy = llvm::cast<llvm::PointerType>(secondTy)->
			    getElementType();
	    if (const llvm::ArrayType *AT =
			    llvm::dyn_cast<llvm::ArrayType>(secondTy))
		    secondTy = AT->getElementType();

	    if (firstTy != secondTy) {
	      typename PTSet::iterator const tmp = v++;
	      s->second.erase(tmp);
	    } else
	      ++v;
	  }
	}
#endif
	++s;
      }
  }
  return S;
}

const PointsToGraph& PointsToGraph::fixpoint(void)
{
  bool change;

  DataLayout DL(&PS->getModule());

  do {
    change = false;

    for (ProgramStructure::const_iterator I = PS->begin(); I != PS->end(); ++I) {
      change |= applyRules(*I, DL);
    }
  } while (change);

  return *this;
}

PointsToSets &computePointsToSets(const ProgramStructure &P, PointsToSets &S,
                                  unsigned int K)
{
    PointsToSets TmpPTS;
    unsigned int Runs, I;

    if (K) {
        Runs = (unsigned int) (log(K) / log(2)); // transfer to base of 2
        if (!Runs)
            Runs = 1;

#ifdef PS_DEBUG
        errs() << "[Points-to]: Running algorithm " << Runs << " times\n";
#endif // PS_DEBUG

        for (I = 0; I < Runs; ++I) {
            PointsToGraph PTG(&P, new IDBitsCategory(I));
            PTG.toPointsToSets(S);
        }
    // if K is not given, compute number of runs from first run
    // XXX or use program structure?
    } else {
        // Steengaard's analysis is the fastest but least accurate.
        // However, points-to sets computed by steengaard's analysis
        // gives us upper bound. They can be now only
        // reduced. Deduce next steps from this first run
        PointsToGraph PTG(&P, new AllInOneCategory());
        PTG.toPointsToSets(S);

        K = S.getContainer().size();
        // use log2(n) runs of the algorithm
        Runs = (unsigned int) (log(K) / log(2));
        // setting Runs to 1 here has no effect but writing out
        // correct debug message
        if (!Runs)
            Runs = 1;

#ifdef PS_DEBUG
        errs() << "[Points-to]: Guessing number of runs\n";
        errs() << "[Points-to]: Running algorithm " << Runs << " times\n";
#endif // PS_DEBUG

        // I = 1 because we have already done one run
        for (I = 1; I < Runs; ++I) {
            PointsToGraph PTG(&P, new IDBitsCategory(I));
            PTG.toPointsToSets(S);
        }
    }

    return pruneByType(S);
}

const PTSet &
getPointsToSet(const llvm::Value *const &memLoc, const PointsToSets &S,
		const int idx) {
  const PointsToSets::const_iterator it = S.find(Ptr(memLoc, idx));
  if (it == S.end()) {
    static const PTSet emptySet;
    errs() << "WARNING[PointsTo]: No points-to set has been found: ";
    memLoc->print(errs());
    errs() << '\n';
    return emptySet;
  }
  return it->second;
}

ProgramStructure::ProgramStructure(Module &M) : M(M) {
    for (Module::const_global_iterator g = M.global_begin(), E = M.global_end();
	    g != E; ++g)
      if (isGlobalPointerInitialization(&*g))
	detail::toRuleCode(&*g,std::back_inserter(this->getContainer()));

    detail::CallMaps CM(M);

    for (Module::const_iterator f = M.begin(); f != M.end(); ++f) {
	for (const_inst_iterator i = inst_begin(f), E = inst_end(f);
		i != E; ++i) {
	    if (isPointerManipulation(&*i))
		detail::toRuleCode(&*i,
			    std::back_inserter(this->getContainer()));
	    else if (const CallInst *c = dyn_cast<CallInst>(&*i)) {
		if (!isInlineAssembly(c))
		    CM.collectCallRuleCodes(c,
			std::back_inserter(this->getContainer()));
	    } else if (const ReturnInst *r = dyn_cast<ReturnInst>(&*i)) {
		CM.collectReturnRuleCodes(r,
			std::back_inserter(this->getContainer()));
	    }
	}
    }
#ifdef PS_DEBUG
    errs() << "==PS START\n";
    for (const_iterator I = getContainer().begin(), E = getContainer().end();
	    I != E; ++I) {
	const RuleCode &rc = *I;
	errs() << "\tTYPE=" << rc.getType() << "\n\tL=";
	rc.getLvalue()->dump();
	errs() << "\tR=";
	rc.getRvalue()->dump();
    }
    errs() << "==PS END\n";
#endif
}

}}
