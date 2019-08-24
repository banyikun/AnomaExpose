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
 *          Ron Dreslinski
 *          Ali Saidi
 */

/**
 * @file
 * Definitions of functional page table.
 */
#include "mem/page_table.hh"

#include <string>
#include "math.h"
#include <cmath>

#include "base/compiler.hh"
#include "base/trace.hh"
#include "debug/MMU.hh"
#include "sim/faults.hh"
#include "sim/serialize.hh"


#include "debug/Process_BAN.hh"
#include "debug/TLB_BAN.hh"
#include "debug/TLB.hh"
#include "debug/TLB_ANCHOR.hh"
#include "debug/PageConti.hh"

using namespace std;

void
EmulationPageTable::map(Addr vaddr, Addr paddr, int64_t size, uint64_t flags)
{
    bool clobber = flags & Clobber;
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0); 

    int nump = size/pageSize;
    if (nump < 30){
        smallContiBlock ++;
    }else if (nump < 100){
        mediumContiBlock ++;
    }else{
        bigContiBlock ++;
    }
    DPRINTF(PageConti, "contiguity is %d \n", size/pageSize);
 


    DPRINTF(MMU, "Allocating Page: %#x-%#x\n", vaddr, vaddr + size);
    // DPRINTF(Process_BAN, "size is %d. PageSize is %d\n",size, pageSize);
    while (size > 0) {
        auto it = pTable.find(vaddr);
        if (it != pTable.end()) {
            // already mapped
            panic_if(!clobber,
                     "EmulationPageTable::allocate: addr %#x already mapped",
                     vaddr);
            it->second = Entry(paddr, flags);
        } else {
            Entry entry =  Entry(paddr, flags);
       /**
            if ((vaddr / pageSize % anchorDistance1) == 0){
                // set it as an anchorEntry
               entry.isAnchor = 1;
               caculateContiguity(&entry, anchorDistance1, vaddr, paddr);
              
               //DPRINTF(Process_BAN, "This is an anchorEntry. Update %d, contiguity is %d.\n", vaddr/pageSize, entry.contiguity);
            } 
            
            if ((vaddr / pageSize % anchorDistance2) == 0){
                entry.isAnchor = 1;
                caculateContiguity(&entry, anchorDistance2, vaddr, paddr);
            }
                     
                   //set it as a regular entry and calcuate the contiguity of its corresponding anchorEntry
                   // entry.isAnchor = 0;
                  //  DPRINTF(Process_BAN, "This is a regular entry.\n");
        **/
            
            pTable.emplace(vaddr, entry);
        }

        size -= pageSize;
        vaddr += pageSize;
        paddr += pageSize;
    }
    DPRINTF(MMU, "PageSize = %#x\n", pageSize);

}







void
EmulationPageTable::remap(Addr vaddr, int64_t size, Addr new_vaddr)
{
    assert(pageOffset(vaddr) == 0);
    assert(pageOffset(new_vaddr) == 0);

    DPRINTF(MMU, "moving pages from vaddr %08p to %08p, size = %d\n", vaddr,
            new_vaddr, size);

    while (size > 0) {
        auto new_it M5_VAR_USED = pTable.find(new_vaddr);
        auto old_it = pTable.find(vaddr);
        assert(old_it != pTable.end() && new_it == pTable.end());

        pTable.emplace(new_vaddr, old_it->second);
        pTable.erase(old_it);
        size -= pageSize;
        vaddr += pageSize;
        new_vaddr += pageSize;
    }
}

void
EmulationPageTable::getMappings(std::vector<std::pair<Addr, Addr>> *addr_maps)
{
    int i = 0;
    for (auto &iter : pTable)
        addr_maps->push_back(make_pair(iter.first, iter.second.paddr));
        i++;
    DPRINTF(MMU,"%d page entries totally ",i);

}

void
EmulationPageTable::unmap(Addr vaddr, int64_t size)
{
    assert(pageOffset(vaddr) == 0);

    DPRINTF(MMU, "Unmapping page: %#x-%#x\n", vaddr, vaddr + size);

    while (size > 0) {
        auto it = pTable.find(vaddr);
        assert(it != pTable.end());
        pTable.erase(it);
        size -= pageSize;
        vaddr += pageSize;
    }
}

bool
EmulationPageTable::isUnmapped(Addr vaddr, int64_t size)
{
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0);

    for (int64_t offset = 0; offset < size; offset += pageSize)
        if (pTable.find(vaddr + offset) != pTable.end())
            return false;

    return true;
}

const EmulationPageTable::Entry *
EmulationPageTable::lookup(Addr vaddr)
{
    Addr page_addr = pageAlign(vaddr);
    DPRINTF(TLB, "vaddr %#x : page_addr %#x, offsetMask is %d, pageSize is %d.\n", vaddr, page_addr, offsetMask, pageSize);
    DPRINTF(TLB, "pTable address is %s\n", &pTable);
    PTableItr iter = pTable.find(page_addr);
    if (iter == pTable.end()){
        DPRINTF(TLB, "RETURN NULL\n");
        return nullptr;}
    Entry * entry = &(iter->second);
    entry->vaddr = page_addr;
    return entry;
}


// BAN
const EmulationPageTable::Entry *
EmulationPageTable::lookup_tlb(Addr vaddr)
{
    Addr page_addr = pageAlign(vaddr);
    DPRINTF(TLB, "here vaddr %#x : page_addr %#x\n", vaddr/pageSize, page_addr/pageSize);
    DPRINTF(TLB, "pTable: %s\n", &pTable);
    PTableItr iter = pTable.find(page_addr);
    Entry * entry = &(iter->second);
    bool anchor1 = false;
    if (iter == pTable.end()){
        DPRINTF(TLB, "lookup null \n");
        return nullptr;}
    //Entry * entry = &(iter->second);
    entry->vaddr = page_addr;

    
    if (vaddr%anchorDistance1 == 0) {
        entry->req_paddr = entry->paddr;
        DPRINTF(TLB_ANCHOR, "return an anchor\n");
        caculateContiguity(entry, anchorDistance1, page_addr, entry->paddr);
        entry->isAnchor = true;
        return entry;
    } else {
                // AnchorDistance 1
        int rem = anchorDistance1 - (page_addr/pageSize)%anchorDistance1;
        //DPRINTF(TLB, "anchorDistance is %d\n", rem);
        Addr anchor_addr =  (((page_addr/pageSize)&~(anchorDistance1-1))+anchorDistance1)*pageSize;
        DPRINTF(TLB, "anchor_vaddr is %d\n", anchor_addr/pageSize);
        PTableItr new_iter  = pTable.find(anchor_addr);
        Entry * anchorEntry1;
        if (new_iter != pTable.end()){
            anchorEntry1 = &(new_iter->second);
            caculateContiguity(anchorEntry1, anchorDistance1, anchor_addr, anchorEntry1->paddr);
             
            if (anchorEntry1->contiguity >= rem){
                anchorEntry1->vaddr = anchor_addr;
                anchorEntry1->req_paddr = entry->paddr;
                anchorEntry1->anchorDistance = anchorDistance1;
                anchorEntry1->isAnchor = true;
                //DPRINTF(TLB_ANCHOR, "return an anchor but not a regular\n");
                anchor1 = true;
            }
        }

                
        if (anchor1){
             //DPRINTF(TLB_ANCHOR, "return an anchor1\n");
            return anchorEntry1;
        } 
        
        return entry;     
    }   
}

void EmulationPageTable::caculateContiguity (Entry * entry, int _anchorDistance, Addr vaddr, Addr paddr){

    Addr vaddrCopy = vaddr;
    Addr paddrCopy = paddr;
    Addr new_vaddr;
    Addr new_paddr;
    int rem = _anchorDistance;
    int contiguity = 0;
    while ( rem > 1) {
        rem--;
        new_vaddr = vaddrCopy - pageSize;
        auto new_it = pTable.find(new_vaddr);
        if (new_it != pTable.end()) {
            Entry * new_entry = &(new_it->second);
            new_paddr = new_entry->paddr;
            if ((paddrCopy - new_paddr) == pageSize) {
                contiguity ++;
                paddrCopy = new_paddr;
                vaddrCopy = new_vaddr;
            } else {
                // new_paddr is not continuous with paddar
                break; }
        } else {
            // new_vaddr hasn't been mapped
            break; }
    }
    entry->contiguity = contiguity;
    DPRINTF(Process_BAN, "anchorDistance is %d,contiguity is %d,  EntryContiguity is %d\n",_anchorDistance, contiguity, entry->contiguity);
 
}



bool
EmulationPageTable::translate(Addr vaddr, Addr &paddr)
{
    const Entry *entry = lookup(vaddr);
    if (!entry) {
        DPRINTF(MMU, "Couldn't Translate: %#x\n", vaddr);
        return false;
    }
    paddr = pageOffset(vaddr) + entry->paddr;
   // DPRINTF(MMU, "Translating: %#x->%#x\n", vaddr, paddr);
    return true;
}

Fault
EmulationPageTable::translate(RequestPtr req)
{
    Addr paddr;
    assert(pageAlign(req->getVaddr() + req->getSize() - 1) ==
           pageAlign(req->getVaddr()));
    if (!translate(req->getVaddr(), paddr))
        return Fault(new GenericPageTableFault(req->getVaddr()));
    req->setPaddr(paddr);
    if ((paddr & (pageSize - 1)) + req->getSize() > pageSize) {
        panic("Request spans page boundaries!\n");
        return NoFault;
    }
    return NoFault;
}

void
EmulationPageTable::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "ptable.size", pTable.size());

    PTable::size_type count = 0;
    for (auto &pte : pTable) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", count++));

        paramOut(cp, "vaddr", pte.first);
        paramOut(cp, "paddr", pte.second.paddr);
        paramOut(cp, "flags", pte.second.flags);
    }
    assert(count == pTable.size());
}

void
EmulationPageTable::unserialize(CheckpointIn &cp)
{
    int count;
    paramIn(cp, "ptable.size", count);

    for (int i = 0; i < count; ++i) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", i));

        Addr vaddr;
        UNSERIALIZE_SCALAR(vaddr);
        Addr paddr;
        uint64_t flags;
        UNSERIALIZE_SCALAR(paddr);
        UNSERIALIZE_SCALAR(flags);

        pTable.emplace(vaddr, Entry(paddr, flags));
    }
}



void
EmulationPageTable::regStats()
{
    using namespace Stats;

    smallContiBlock
        .name(name() + ".smallContiBlock")
        .desc("small contiguity block");
    mediumContiBlock
        .name(name() + ".mediumContiBlock")
        .desc("medium contiguity block");
    bigContiBlock
        .name(name() + ".bigContiBlock")
        .desc("big contiguity block");



}
