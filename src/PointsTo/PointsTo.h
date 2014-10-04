// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.

#ifndef POINTSTO_POINTSTO_H
#define POINTSTO_POINTSTO_H

#include <map>
#include <set>
#include <vector>

#include "llvm/Value.h"

#include "RuleExpressions.h"

namespace llvm {
namespace ptr {

  /* the most significant bit is not used and is used
   * as a marker for undefined */
  enum {
    RANGE_TO_UNDEF    = 1,
    RANGE_FROM_UNDEF  = 1 << 1,
  };

  /* we could do it as a template, it would be nice but useless... */
  struct PtrRange {
    typedef int32_t Type;

    Type from;
    Type to;

    /*  we could use different special values of from and to and
     *  do masking and bit shifting.. but today when the memory
     *  is so cheap... */
    uint8_t flags;

    PtrRange() :flags(RANGE_TO_UNDEF | RANGE_FROM_UNDEF) {};
    PtrRange(const Type v) : from(v), to(v), flags(0) {};
    PtrRange(const Type f, const Type t)
      :from(f), to(t), flags(0) {}

    bool isFromUndefined(void) const { return flags & RANGE_FROM_UNDEF; }
    bool isToUndefined(void) const { return flags & RANGE_TO_UNDEF; }
    bool isUndefined(void) const { return isToUndefined() && isFromUndefined();}

    void operator=(const PtrRange& r)
      { from = r.from; to = r.to; flags = r.flags; }
    void operator=(const Type val) { setTo(val);  setFrom(val); }
    bool operator==(const PtrRange& r) const
      { return from == r.from && to == r.to && flags == r.flags; }
    bool operator!=(const PtrRange& r) const { return !operator==(r); }

    void operator+(const Type val);
    void operator-(const Type val);
    void operator*(const Type val);
    void operator/(const Type val);

    void operator+(const PtrRange& r);
    void operator-(const PtrRange& r);
    void operator*(const PtrRange& r);
    void operator/(const PtrRange& r);

    void setFrom(const Type v) { fromDefined(); from = v; }
    void setTo(const Type v) { toDefined(); to = v; }

    void print(void) const;

  private:
    void fromDefined(void) { flags &= (~RANGE_FROM_UNDEF); }
    void toDefined(void) { flags &= (~RANGE_TO_UNDEF); }
    void fromUndefined(void) { flags &= RANGE_FROM_UNDEF; }
    void toUndefined(void) { flags &= RANGE_TO_UNDEF; }

    void swapValues(void) { Type tmp = from; from = to; to = tmp; }
  };

  /*
   * pointer is a pair <location, offset> such that the location is:
   * a) variable, offset is -1
   * b) alloc,    offset is <0,infty) -- structure members can point too
   *
   * Note that in LLVM, both a variable and an alloc (CallInst to malloc)
   * are llvm::Value.
   */
  struct Pointer {
    typedef const llvm::Value *MemoryLocation;

    MemoryLocation location;
    int32_t offset;

    bool operator<(const Pointer& ptr) const;
    bool operator==(const Pointer& ptr) const
      { return location == ptr.location && offset == ptr.offset; }

    Pointer(MemoryLocation loc, int32_t off)
      : location(loc), offset(off) {}
  };
} // llvm
} // ptr

namespace llvm { namespace ptr {

  class PointsToSets {
  public:
    /*
     * Points-to set contains again pairs <location, offset>, where location
     * can be only an alloc companied by an offset (we can point to the
     * middle).
     */
    typedef Pointer Pointee;
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

namespace llvm { namespace ptr {

  const PointsToSets::PointsToSet &
  getPointsToSet(const llvm::Value *const &memLoc, const PointsToSets &S,
		  const int offset = -1);

  PointsToSets &computePointsToSets(const ProgramStructure &P, PointsToSets &S);

}}

#endif
