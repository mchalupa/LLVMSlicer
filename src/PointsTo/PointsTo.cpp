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


// This is an implementation of Shapiro-Horwitz analysis
//
// See details at:
// https://is.muni.cz/auth/th/396236/fi_b/?fakulta=1433;obdobi=5984;studium=576656;lang=cs;sorter=tema;balik=1275
// http://www.eecs.umich.edu/acal/swerve/docs/54-1.pdf
///
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
    ElementsTy::const_iterator Begin = Elements.begin();

    errs() << "[";

    for (ElementsTy::const_iterator I = Begin, E = Elements.end();
         I != E; ++I) {
        if (I != Begin)
            errs() << ", ";

        printPtrName(*I);
    }

    errs() << "]\n";
}

void PointsToGraph::dump(void) const
{
    std::unordered_map<Pointer, Node *>::const_iterator I, E;

    if (Nodes.empty()) {
        errs() << "PointsToGraph is empty\n";
        return;
    }

    for(I = Nodes.begin(), E = Nodes.end(); I != E; ++I) {
        if (!I->second)
            continue;

        I->second->dump();

        Node::EdgesTy Edges = I->second->getEdges();
        for (unsigned int I = 0; I < Node::EDGES_NUM; ++I) {
            if (!Edges[I])
                continue;

            errs() << "    --> ";
            Edges[I]->dump();
        }
    }
}

void PointsToGraph::replaceNode(PointsToGraph::Node *a,
                                PointsToGraph::Node *b)
{
    assert(a && "a must not be NULL");

    Node::ElementsTy& Elements = a->getElements();

    for (Node::ElementsTy::iterator I = Elements.begin(), E = Elements.end();
         I != E; ++I) {
        Node *&n = Nodes[*I];
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

inline PointsToGraph::Node *PointsToGraph::addNode(Pointee p)
{
    Node *n = new Node(p, this);
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
        n = new Node(P, this);

    return n;
}

// XXX do in Node's member function?
void PointsToGraph::replaceEdges(Node *a, Node *b)
{
    assert(a && "a must not be NULL");
    assert(b && "b must not be NULL");

    Node::ReferencesTy& References = b->getReferences();
    Node::EdgesTy Edges = b->getEdges();

    // change edges that points to b so that they will point to a
    for (Node::ReferencesTy::iterator I = References.begin(),
         E = References.end(); I != E; ++I) {

         // delete edges that points to b
         (*I)->removeNeighbour(b->getCategory());
         // and add edges that points to a instead
         (*I)->addNeighbour(a);
    }

    // change references of outgoing edges
    for (unsigned int I = 0; I < Node::EDGES_NUM; ++I)
        if (Edges[I]) {
            Edges[I]->getReferences().erase(b);
            a->addNeighbour(Edges[I]);
        }
}

void PointsToGraph::mergeNodes(Node *a, Node *b)
{
    Node::ElementsTy& ElementsB = b->getElements();

    // copy elements from b to a
    for (Node::ElementsTy::iterator I = ElementsB.begin(), E = ElementsB.end();
         I != E; ++I) {
        a->insert(*I);

        // set new location node for the element
        Nodes[*I] = a;
    }

    // make all nodes that point to b point to a
    replaceEdges(a, b);

    delete b;

    // if there were some edges with the same category, then these should
    // be merged in addNeighbour called from replaceEdges, so we're done here
}

bool PointsToGraph::insert(Pointer p, Pointee location)
{
    bool changed = false;

    // find node that contains pointer p. From this node will
    // be created new outgoing edge (if needed)
    PointsToGraph::Node *From, *To;

    From = getNode(p);
    To = findNode(location);

    if (To) {
        // addNeighbour should merge the nodes if necessary
        changed = From->addNeighbour(To);
    } else {
        // pointer is not in any node. Check if From node
        // has neighbour with the same category. If so, just
        // add the pointer there
        Node *n = From->getEdges()[PTC->getCategory(location)]; 

        if (n) {
            n->insert(location);
            Nodes[location] = n;
            changed = true;
        } else {
            To = getNode(location);
            changed = From->addNeighbour(To);
        }
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

    Node::EdgesTy Edges = LocationNode->getEdges();

    for (unsigned int I = 0; I < Node::EDGES_NUM; ++I)
        if (Edges[I])
            changed |= PointerNode->addNeighbour(Edges[I]);

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
        // If location do not have a node yet, then it do not have
        // neighbours which should be inserted.
        // XXX maybe we should write out a message about unsoundness,
        // since we're dereferencing pointer pointing to unknown location
        return false;
    }

    return insertDerefPointee(p, LocationNode);
}

bool PointsToGraph::insertDerefPointer(Node *PointerNode, Node *LocationNode)
{
    bool changed = false;

    Node::EdgesTy Edges = PointerNode->getEdges();

    for (unsigned int I = 0; I < Node::EDGES_NUM; ++I)
        if (Edges[I])
            changed |= Edges[I]->addNeighbour(LocationNode);

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

    Node::EdgesTy Edges = PointerNode->getEdges();

    for (unsigned int I = 0; I < Node::EDGES_NUM; ++I)
        if (Edges[I])
            changed |= insertDerefPointee(Edges[I], LocationNode);

    return changed;
}

void PointsToGraph::Node::convertToPointsToSets(PointsToSets& PS,
                                                bool intersect) const
{
    typedef PointsToSets::PointsToSet PTSet;
    typedef PointsToSets::Pointer Ptr;

    for (ElementsTy::const_iterator ElemI = Elements.begin(),
         ElemE = Elements.end();
         ElemI != ElemE; ++ElemI) {

        PTSet& S = PS[*ElemI];
        PTSet TmpPTSet;

        for (unsigned int I = 0; I < EDGES_NUM; ++I) {
            if (!Edges[I])
                continue;

            const ElementsTy& Ptees = Edges[I]->getElements();

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
    Ptr L(lval, -1);

    if (hasExtraReference(op)) {
        changed = insert(L, Ptr(op, off)); /* VAR = REF */
    } else { /* VAR = VAR */

        Ptr R(op, -1);
        Node *n = findNode(R);

        if (!n || !n->hasNeighbours())
            return false;

        Node::EdgesTy Edges = n->getEdges();
        for (unsigned int I = 0; I < Node::EDGES_NUM; ++I) {
            if (!Edges[I])
                continue;

            // if it's an array, go backward and find last offset
            // (set is sorted). It can introduce some unsoundness,
            // but for most cases it's working pretty well
            Node::ElementsTy& Elems = Edges[I]->getElements();
            Node::ElementsTy::reverse_iterator PI, PE;
            for (PI = Elems.rbegin(), PE = Elems.rend(); PI != PE; ++PI) {

                 // offset with variable has no meaning
                 if (PI->second == -1)
                    continue;

                const Value *val = PI->first;

                // there's no point to have offset with these values
                if (isa<Function>(val) || isa<ConstantPointerNull>(val))
                    continue;

                int64_t sum = PI->second + off;

                if (isArray) {
                    if (sum < 0)
                        sum = 0;
                    /* unsoundnes :-) */
                    else if (sum > 64)
                        sum = 64;
                }

                if (!checkOffset(DL, val, sum))
                    continue;

                changed |= insert(L, Ptr(val, sum));
                break; // we're done!
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

    Node::EdgesTy Edges = r->getEdges();
    for (unsigned int I = 0; I < Node::EDGES_NUM; ++I)
        if (Edges[I])
            // must process nodes *two* steps away
            change |= insertDerefPointee(L, Edges[I]);

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

    // create a copy of current edges and iterate over this copy,
    // because this operation can change the edges,
    // but we need to iterate only over these (old) edges
    // XXX don't we need copying even when dereferencing only one side??
    Node *Edges[NODE_EDGES_NUM];
    memcpy(&Edges, r->getEdges(), sizeof Edges);
    for (unsigned int I = 0; I < Node::EDGES_NUM; ++I)
        if (Edges[I])
            change |= insertDerefBoth(l, Edges[I]);

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

const PointsToGraph& PointsToGraph::build(void)
{
    DataLayout DL(&PS->getModule());

    for (ProgramStructure::const_iterator I = PS->begin(); I != PS->end(); ++I)
        applyRules(*I, DL);

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
