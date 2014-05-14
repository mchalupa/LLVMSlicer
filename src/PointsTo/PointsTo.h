// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.

#ifndef POINTSTO_POINTSTO_H
#define POINTSTO_POINTSTO_H

#include <map>
#include <unordered_map>
#include <set>
#include <vector>

#include "llvm/Value.h"
#include "llvm/DataLayout.h"

#include "RuleExpressions.h"

namespace llvm { namespace ptr {

  class PointsToSets {
  public:
    typedef const llvm::Value *MemoryLocation;
    /*
     * pointer is a pair <location, offset> such that the location is:
     * a) variable, offset is -1
     * b) alloc,    offset is <0,infty) -- structure members can point too
     *
     * Note that in LLVM, both a variable and an alloc (CallInst to malloc)
     * are llvm::Value.
     */
    typedef std::pair<MemoryLocation, int> Pointer;
    /*
     * Points-to set contains against pairs <location, offset>, where location
     * can be only an alloc companied by an offset (we can point to the
     * middle).
     */
    typedef std::pair<MemoryLocation, int> Pointee;
    typedef std::set<Pointee> PointsToSet;

    typedef std::map<Pointer, PointsToSet> Container;
    typedef Container::key_type key_type;
    typedef Container::mapped_type mapped_type;
    typedef Container::value_type value_type;
    typedef Container::iterator iterator;
    typedef Container::const_iterator const_iterator;
    typedef std::pair<iterator, bool> insert_retval;

    virtual ~PointsToSets() {}

    insert_retval insert(value_type const& val) { return C.insert(val); }
    PointsToSet& operator[](key_type const& key) { return C[key]; }
    const_iterator find(key_type const& key) const { return C.find(key); }
    iterator find(key_type const& key) { return C.find(key); }
    const_iterator begin() const { return C.begin(); }
    iterator begin() { return C.begin(); }
    const_iterator end() const { return C.end(); }
    iterator end() { return C.end(); }
    Container const& getContainer() const { return C; }
    Container& getContainer() { return C; }
  private:
    Container C;
  };

}}

namespace llvm { namespace ptr {

    struct ProgramStructure
    {
        typedef RuleCode Command;
        typedef std::vector<Command> Container;
        typedef Container::value_type value_type;
        typedef Container::iterator iterator;
        typedef Container::const_iterator const_iterator;

        explicit ProgramStructure(Module &M);

        llvm::Module &getModule() const { return M; }

        void insert(iterator it, value_type const& val) { C.insert(it,val); }
        void push_back(value_type const& val) { return C.push_back(val); }
        const_iterator begin() const { return C.begin(); }
        iterator begin() { return C.begin(); }
        const_iterator end() const { return C.end(); }
        iterator end() { return C.end(); }
        Container const& getContainer() const { return C; }
        Container& getContainer() { return C; }
    private:
        Container C;
        llvm::Module &M;
    };

}}

namespace std {

typedef pair<const llvm::Value *, int> Ptr;

template<>
struct hash<Ptr> {
    size_t operator()(const Ptr& val) const
    {
        return (_hash(val.first) ^ val.second);
    }

    hash<const llvm::Value *> _hash;
};

} // namespace std

namespace llvm {
namespace ptr {

    ///
    // This class represents catagories in Shapiro-Horwitz points-to analysis
    ///
    class PointsToCategories
    {
    public:
        typedef PointsToSets::Pointer Pointer;

        virtual ~PointsToCategories() {}
        virtual unsigned int getCategory(Pointer a) const = 0;
    };

    // implies Steengaard's analysis
    class AllInOneCategory : public PointsToCategories
    {
    public:
        virtual unsigned int getCategory(Pointer a) const
            { return 0; }
    };

    class IDBitsCategory : public PointsToCategories
    {
    public:
        IDBitsCategory(unsigned int K) { this->K = K; }

        virtual unsigned int getCategory(Pointer a) const
            { return (((a.first->getValueID() ^ a.second) >> K) & 0x7); }
    private:
        // use Kth bit of ID
        unsigned int K;
    };

    ///
    // This class represents Storage Shape Graph as described
    // in Shapiro-Horwitz analysis [1].
    // See details at:
    // https://is.muni.cz/auth/th/396236/fi_b/?fakulta=1433;obdobi=5984;studium=576656;lang=cs;sorter=tema;balik=1275
    //
    // [1] Marc Shapiro and Susan Horwitz: Fast and Accurate Flow-Insensitive Points-to Analysis
    //     http://www.eecs.umich.edu/acal/swerve/docs/54-1.pdf
    ///
    class PointsToGraph
    {
    public:
        // will build the points-to graph right from the constructor
        PointsToGraph(const ProgramStructure *PS, PointsToCategories *PTC)
        :PS(PS), PTC(PTC)
        {
            // estimate number of pointers
            Nodes.reserve(3 * PS->getContainer().size() / 2);
            build();
        }

        virtual ~PointsToGraph();

        typedef PointsToSets::Pointer Pointer;
        typedef PointsToSets::Pointee Pointee;

        PointsToSets& toPointsToSets(PointsToSets& PS) const;
        void dump(void) const;

    private:
        // this class represents one node in the graph
        class Node
        {
        public:
            typedef std::set<Pointee> ElementsTy;
            typedef llvm::SmallPtrSet<Node *, 16> ReferencesTy;

            typedef Node** EdgesTy;
            #define NODE_EDGES_NUM 8
            static const unsigned int EDGES_NUM = NODE_EDGES_NUM;

            Node() {};
            Node(Pointee p, PointsToGraph *PTG)
                :origin(p), PTG(PTG)
                { insert(p); Category = PTG->getCategories()->getCategory(p); }

            inline void insert(Pointee p)
            {
                Elements.insert(p);
            }

            ElementsTy& getElements(void) { return Elements; }
            const ElementsTy& getElements(void) const { return Elements; }

            EdgesTy getEdges(void) { return Edges; }
            const Node * const * getEdges(void) const { return Edges; }
            ReferencesTy& getReferences(void) { return References; }
            const ReferencesTy& getReferences(void) const { return References; }

            Pointee getOrigin() { return origin; }
            const Pointee& getOrigin() const { return origin; }

            unsigned int getCategory() const
                { return Category; }

            // Add edge from this node to node n.
            // Merge nodes with same category if necessary.
            bool addNeighbour(Node *n)
            {

                unsigned c = n->getCategory();
                assert(c < NODE_EDGES_NUM);

                if (Edges[c]) {
                    if (Edges[c] == n)
                        return false;
                    // if there already exists a node with the same
                    // category, merge them
                    PTG->mergeNodes(Edges[c], n);
                } else {
                    Edges[c] = n;
                    n->References.insert(this);
                    ++EdgesNo;
                }

                return true;
            }

            void removeNeighbour(Node *n)
            {
                unsigned c = n->getCategory();
                assert(Edges[c] == n);
                Edges[c] = NULL; --EdgesNo;
            }

            void removeNeighbour(unsigned int c)
            {
                assert(Edges[c]);

                --EdgesNo;
                Edges[c] = NULL;
            }

            inline bool hasNeighbours(void) const
            {
                return EdgesNo > 0;
            }

            void convertToPointsToSets(PointsToSets& PS,
                                       bool intersect = false) const;

            void dump(void) const;

        private:
            ElementsTy Elements;      // items in node
            ReferencesTy References;  // what nodes points to this one?
            Node *Edges[NODE_EDGES_NUM] = {0};
            unsigned int EdgesNo = 0; // number of outgoing edges

            Pointee origin;
            PointsToGraph *PTG;
            unsigned int Category;
        };

        const PointsToCategories *getCategories(void) const {  return PTC; }

        // insert that p points to location
        bool insert(Pointer p, Pointee location);
        // insert that p points to all Pointees from locations
        bool insert(Pointer p, std::set<Pointee>& locations);
        // insert all pairs a->b where b is every pointer
        // the Pointee can points to
        bool insertDerefPointee(Pointer p, Pointee location);
        bool insertDerefPointee(Pointer p, Node *LocationNode);
        bool insertDerefPointee(Node *PointerNode, Node *LocationNode);
        // insert all pairs a->b where a is every pointer the p can point to
        bool insertDerefPointer(Pointer p, Pointee location);
        bool insertDerefPointer(Node *PointerNode, Pointee location);
        bool insertDerefPointer(Node *PointerNode, Node *LocationNode);

        bool insertDerefBoth(Node *PointerNode, Node *LocationNode);

        // add new node
        Node *addNode(Pointee p);

        // get or insert new node, accesses Nodes only once
        Node *getNode(Pointee P);

        Node *findNode(Pointee p) const;

        // replace node in map
        void replaceNode(Node *, Node *);
        // make all nodes that points to the first node,
        // point to the second node instead
        void replaceEdges(Node *, Node *);

        void mergeNodes(Node *, Node *);
        // hash table Pointer->Node
        std::unordered_map<Pointer, Node *> Nodes;
        const ProgramStructure *PS;
        PointsToCategories *PTC;

        // --------------------------------------------------------------------
        // applyRules functions -> convert ruleCodes into points-to-graph
        // --------------------------------------------------------------------
        bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                       VARIABLE<const llvm::Value *> > const& E);

        bool applyRule(const llvm::DataLayout &DL, ASSIGNMENT<
                       VARIABLE<const llvm::Value *>,
                       GEP<VARIABLE<const llvm::Value *> > > const& E);

        bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                       REFERENCE<VARIABLE<const llvm::Value *> > > const& E);

        bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                       DEREFERENCE< VARIABLE<const llvm::Value *> >
                       > const& E, const int idx = -1);

        bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                       VARIABLE<const llvm::Value *> > const& E);

        bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                       REFERENCE<VARIABLE<const llvm::Value *> > > const &E);

        bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                       DEREFERENCE<VARIABLE<const llvm::Value *> > > const& E);

        bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                       ALLOC<const llvm::Value *> > const &E);

        bool applyRule(ASSIGNMENT<VARIABLE<const llvm::Value *>,
                       NULLPTR<const llvm::Value *> > const &E);

        bool applyRule(ASSIGNMENT<DEREFERENCE<VARIABLE<const llvm::Value *> >,
                       NULLPTR<const llvm::Value *> > const &E);

        bool applyRule(DEALLOC<const llvm::Value *>);

        // apply the right applyRule() for RuleCode
        bool applyRules(const RuleCode &RC, const llvm::DataLayout &DL);

        // apply rules until you can
        const PointsToGraph& build(void);

        // tester class, must be able to access private attributes
        friend class PTGTester;
    };

} // namespace ptr
} // namespace llvm

namespace llvm { namespace ptr {

  const PointsToSets::PointsToSet &
  getPointsToSet(const llvm::Value *const &memLoc, const PointsToSets &S,
		  const int offset = -1);

  PointsToSets &computePointsToSets(const ProgramStructure &P, PointsToSets &S,
                                    unsigned int K = 0);

}}

#endif
