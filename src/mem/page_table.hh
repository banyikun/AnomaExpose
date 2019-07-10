/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
 * Copyright (c) 2003 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Steve Reinhardt
 */

/**
 * @file
 * Declarations of a non-full system Page Table.
 */

#ifndef __MEM_PAGE_TABLE_HH__
#define __MEM_PAGE_TABLE_HH__

#include <string>
#include <unordered_map>

#include "base/intmath.hh"
#include "base/types.hh"
#include "mem/request.hh"
#include "sim/serialize.hh"
#include "debug/DiffAnchorDis.hh"
#include "debug/SameAnchorDis.hh"
#include "debug/BigDiffAnchorDis.hh"
#include "debug/BigSameAnchorDis.hh"
#include "base/trace.hh"
#include "base/statistics.hh"
class ThreadContext;

class EmulationPageTable : public Serializable
{
  public:
    struct Entry
    {
        Addr paddr;
        uint64_t flags;


        // ban
        bool isAnchor;
        uint64_t contiguity;
        int anchorDistance;
        Addr vaddr;
        Addr req_paddr; //directly pass paddr to cpu when pass an anchorEntry to TLb


        Entry(Addr paddr, uint64_t flags) : paddr(paddr), flags(flags),isAnchor(false), contiguity(0), vaddr(0), req_paddr(0) {}
        Entry() {}
    };
   





  protected:
    typedef std::unordered_map<Addr, Entry> PTable;
    typedef PTable::iterator PTableItr;
    PTable pTable;

    const Addr pageSize;
    const Addr offsetMask;

    const uint64_t _pid;
    const std::string _name;

  public:
    //const Addr pageSize;
    int anchorDistance1 = 4;
    int anchorDistance2 = 32;

    EmulationPageTable(
            const std::string &__name, uint64_t _pid, Addr _pageSize) :
            pageSize(_pageSize), offsetMask(mask(floorLog2(_pageSize))),
            _pid(_pid), _name(__name)
    {
        assert(isPowerOf2(pageSize));
        using namespace Debug;
        if(SameAnchorDis){
            DPRINTF(SameAnchorDis, "set same Anchor Distance 4 done!\n");
            anchorDistance1 = 4;
            anchorDistance2 = 4;
         }
        else if(DiffAnchorDis){
            DPRINTF(DiffAnchorDis, "set different Anchor Distance 4 32  done!\n");
            anchorDistance1 = 4;
            anchorDistance2 = 32;
        }
        else if(BigDiffAnchorDis){
            DPRINTF(BigDiffAnchorDis, "set different Anchor Distance 16 128  done!\n");
            anchorDistance1 = 16;
            anchorDistance2 = 128;
        }
        else if(BigSameAnchorDis){
            DPRINTF(BigSameAnchorDis, "set different Anchor Distance 16 done!\n");
            anchorDistance1 = 16;
            anchorDistance2 = 16;
        }
        else {
            anchorDistance1 = 4;
            anchorDistance2 = 16;
            std::cout<< "set default anchor distance 4 16"<<std::endl;
        }
        
    }

    uint64_t pid() const { return _pid; };

    virtual ~EmulationPageTable() {};

    /* generic page table mapping flags
     *              unset | set
     * bit 0 - no-clobber | clobber
     * bit 2 - cacheable  | uncacheable
     * bit 3 - read-write | read-only
     */
    enum MappingFlags : uint32_t {
        Clobber     = 1,
        Uncacheable = 4,
        ReadOnly    = 8,
    };


    //ban
     void caculateContiguity(Entry * entry, int _anchorDistance, Addr vaddr, Addr paddr);




    virtual void initState(ThreadContext* tc) {};
    
    // for DPRINTF compatibility
    const std::string name() const { return _name; }

    Addr pageAlign(Addr a)  { return (a & ~offsetMask); }
    Addr pageOffset(Addr a) { return (a &  offsetMask); }

    /**
     * Maps a virtual memory region to a physical memory region.
     * @param vaddr The starting virtual address of the region.
     * @param paddr The starting physical address where the region is mapped.
     * @param size The length of the region.
     * @param flags Generic mapping flags that can be set by or-ing values
     *              from MappingFlags enum.
     */
    virtual void map(Addr vaddr, Addr paddr, int64_t size, uint64_t flags = 0);
    virtual void remap(Addr vaddr, int64_t size, Addr new_vaddr);
    virtual void unmap(Addr vaddr, int64_t size);
    void updateAnchorDistance();
    /**
     * Check if any pages in a region are already allocated
     * @param vaddr The starting virtual address of the region.
     * @param size The length of the region.
     * @return True if no pages in the region are mapped.
     */
    virtual bool isUnmapped(Addr vaddr, int64_t size);

    /**
     * Lookup function
     * @param vaddr The virtual address.
     * @return The page table entry corresponding to vaddr.
     */
    const Entry *lookup(Addr vaddr);

    // added by ban 
    // lookup function for TLB

    const Entry *lookup_tlb(Addr vaddr);




    /**
     * Translate function
     * @param vaddr The virtual address.
     * @param paddr Physical address from translation.
     * @return True if translation exists
     */
    bool translate(Addr vaddr, Addr &paddr);

    /**
     * Simplified translate function (just check for translation)
     * @param vaddr The virtual address.
     * @return True if translation exists
     */
    bool translate(Addr vaddr) { Addr dummy; return translate(vaddr, dummy); }

    /**
     * Perform a translation on the memory request, fills in paddr
     * field of req.
     * @param req The memory request.
     */
    Fault translate(RequestPtr req);

    void getMappings(std::vector<std::pair<Addr, Addr>> *addr_mappings);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;


    // Statistics
    Stats::Scalar smallContiBlock;
    Stats::Scalar mediumContiBlock;
    Stats::Scalar bigContiBlock;

    void regStats() ;

};

#endif // __MEM_PAGE_TABLE_HH__
