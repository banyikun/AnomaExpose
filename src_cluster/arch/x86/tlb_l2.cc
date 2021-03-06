/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Gabe Black
 */

#include "arch/x86/tlb_l2.hh"

#include <cstring>
#include <memory>

#include "arch/generic/mmapped_ipr.hh"
#include "arch/x86/faults.hh"
#include "arch/x86/insts/microldstop.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/msr.hh"
#include "arch/x86/x86_traits.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

// added by ban

#include "debug/TLB_BAN.hh"
#include "debug/MMU.hh"
#include "debug/TLB_ANCHOR.hh"

// end

namespace X86ISA {

TLB_L2::TLB_L2(const Params *p)
    : BaseTLB(p), configAddress(0), size(1024),
      tlb(size), lruSeq(0)
{
    if (!size)
        fatal("TLBs must have a non-zero size.\n");

    for (int x = 0; x < size; x++) {
        tlb[x].trieHandle = NULL;
        freeList.push_back(&tlb[x]);
    }

    //walker = p->walker;
    //walker->setTLB(this);
}

void
TLB_L2::evictLRU()
{
    // Find the entry with the lowest (and hence least recently updated)
    // sequence number.

    unsigned lru = 0;
    for (unsigned i = 1; i < size; i++) {
        if (tlb[i].lruSeq < tlb[lru].lruSeq)
            lru = i;
    }

    assert(tlb[lru].trieHandle);
    trie.remove(tlb[lru].trieHandle);
    //DPRINTF(TLB_BAN, "Evict an entry, %#x. \n", tlb[lru].trieHandle);
    tlb[lru].trieHandle = NULL;
    freeList.push_back(&tlb[lru]);
}

TlbEntry *
TLB_L2::insert(Addr vpn, const TlbEntry &entry)
{
    // If somebody beat us to it, just use that existing entry.
    TlbEntry *newEntry = trie.lookup(vpn);
    if (newEntry) {
        assert(newEntry->vaddr == vpn);
        return newEntry;
    }

    if (freeList.empty())
        evictLRU();

    newEntry = freeList.front();
    freeList.pop_front(); 
    //DPRINTF(TLB_BAN, "Insert an entry.\n");

    *newEntry = entry;
    newEntry->lruSeq = nextSeq();
    newEntry->vaddr = vpn;
    newEntry->trieHandle =
    trie.insert(vpn, TlbEntryTrie::MaxBits - entry.logBytes, newEntry);
    //DPRINTF(TLB_BAN, "Insert an entry. And vpn is %#x .\n",vpn);
    return newEntry;
}

TlbEntry *
TLB_L2::lookup(Addr va, bool update_lru)
{
    TlbEntry *entry = trie.lookup(va);
    if (entry && update_lru)
        entry->lruSeq = nextSeq();
    return entry;
}

void
TLB_L2::flushAll()
{
    DPRINTF(TLB, "Invalidating all entries.\n");
    for (unsigned i = 0; i < size; i++) {
        if (tlb[i].trieHandle) {
            trie.remove(tlb[i].trieHandle);
            tlb[i].trieHandle = NULL;
            freeList.push_back(&tlb[i]);
        }
    }
}

void
TLB_L2::setConfigAddress(uint32_t addr)
{
    configAddress = addr;
}

void
TLB_L2::flushNonGlobal()
{
    DPRINTF(TLB, "Invalidating all non global entries.\n");
    for (unsigned i = 0; i < size; i++) {
        if (tlb[i].trieHandle && !tlb[i].global) {
            trie.remove(tlb[i].trieHandle);
            tlb[i].trieHandle = NULL;
            freeList.push_back(&tlb[i]);
        }
    }
}

void
TLB_L2::demapPage(Addr va, uint64_t asn)
{
    TlbEntry *entry = trie.lookup(va);
    if (entry) {
        trie.remove(entry->trieHandle);
        entry->trieHandle = NULL;
        freeList.push_back(entry);
    }
}

Fault
TLB_L2::translateInt(RequestPtr req, ThreadContext *tc)
{
    DPRINTF(TLB, "Addresses references internal memory.\n");
    Addr vaddr = req->getVaddr();
    Addr prefix = (vaddr >> 3) & IntAddrPrefixMask;
    if (prefix == IntAddrPrefixCPUID) {
        panic("CPUID memory space not yet implemented!\n");
    } else if (prefix == IntAddrPrefixMSR) {
        vaddr = (vaddr >> 3) & ~IntAddrPrefixMask;
        req->setFlags(Request::MMAPPED_IPR);

        MiscRegIndex regNum;
        if (!msrAddrToIndex(regNum, vaddr))
            return std::make_shared<GeneralProtection>(0);

        //The index is multiplied by the size of a MiscReg so that
        //any memory dependence calculations will not see these as
        //overlapping.
        req->setPaddr((Addr)regNum * sizeof(MiscReg));
        return NoFault;
    } else if (prefix == IntAddrPrefixIO) {
        // TODO If CPL > IOPL or in virtual mode, check the I/O permission
        // bitmap in the TSS.

        Addr IOPort = vaddr & ~IntAddrPrefixMask;
        // Make sure the address fits in the expected 16 bit IO address
        // space.
        assert(!(IOPort & ~0xFFFF));
        if (IOPort == 0xCF8 && req->getSize() == 4) {
            req->setFlags(Request::MMAPPED_IPR);
            req->setPaddr(MISCREG_PCI_CONFIG_ADDRESS * sizeof(MiscReg));
        } else if ((IOPort & ~mask(2)) == 0xCFC) {
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            Addr configAddress =
                tc->readMiscRegNoEffect(MISCREG_PCI_CONFIG_ADDRESS);
            if (bits(configAddress, 31, 31)) {
                req->setPaddr(PhysAddrPrefixPciConfig |
                        mbits(configAddress, 30, 2) |
                        (IOPort & mask(2)));
            } else {
                req->setPaddr(PhysAddrPrefixIO | IOPort);
            }
        } else {
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            req->setPaddr(PhysAddrPrefixIO | IOPort);
        }
        return NoFault;
    } else {
        panic("Access to unrecognized internal address space %#x.\n",
                prefix);
    }
}

Fault
TLB_L2::finalizePhysical(RequestPtr req, ThreadContext *tc, Mode mode) const
{
    Addr paddr = req->getPaddr();

    AddrRange m5opRange(0xFFFF0000, 0xFFFFFFFF);

    if (m5opRange.contains(paddr)) {
        req->setFlags(Request::MMAPPED_IPR | Request::GENERIC_IPR |
                      Request::STRICT_ORDER);
        req->setPaddr(GenericISA::iprAddressPseudoInst((paddr >> 8) & 0xFF,
                                                       paddr & 0xFF));
    } else if (FullSystem) {
        // Check for an access to the local APIC
        LocalApicBase localApicBase =
            tc->readMiscRegNoEffect(MISCREG_APIC_BASE);
        AddrRange apicRange(localApicBase.base * PageBytes,
                            (localApicBase.base + 1) * PageBytes - 1);

        if (apicRange.contains(paddr)) {
            // The Intel developer's manuals say the below restrictions apply,
            // but the linux kernel, because of a compiler optimization, breaks
            // them.
            /*
            // Check alignment
            if (paddr & ((32/8) - 1))
                return new GeneralProtection(0);
            // Check access size
            if (req->getSize() != (32/8))
                return new GeneralProtection(0);
            */
            // Force the access to be uncacheable.
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            req->setPaddr(x86LocalAPICAddress(tc->contextId(),
                                              paddr - apicRange.start()));
        }
    }

    return NoFault;
}

Fault
TLB_L2::translate(RequestPtr req, ThreadContext *tc, Translation *translation,
        Mode mode, bool &delayedResponse, bool timing)
{
    Request::Flags flags = req->getFlags();
    int seg = flags & SegmentFlagMask;
    bool storeCheck = flags & (StoreCheck << FlagShift);

    delayedResponse = false;

    // If this is true, we're dealing with a request to a non-memory address
    // space.
    if (seg == SEGMENT_REG_MS) {
        return translateInt(req, tc);
    }

    Addr vaddr = req->getVaddr();
    DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);

    HandyM5Reg m5Reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);

    // If protected mode has been enabled...
    if (m5Reg.prot) {
        DPRINTF(TLB, "In protected mode.\n");
        // If we're not in 64-bit mode, do protection/limit checks
        if (m5Reg.mode != LongMode) {
            DPRINTF(TLB, "Not in long mode. Checking segment protection.\n");
            // Check for a NULL segment selector.
            if (!(seg == SEGMENT_REG_TSG || seg == SYS_SEGMENT_REG_IDTR ||
                        seg == SEGMENT_REG_HS || seg == SEGMENT_REG_LS)
                    && !tc->readMiscRegNoEffect(MISCREG_SEG_SEL(seg)))
                return std::make_shared<GeneralProtection>(0);
            bool expandDown = false;
            SegAttr attr = tc->readMiscRegNoEffect(MISCREG_SEG_ATTR(seg));
            if (seg >= SEGMENT_REG_ES && seg <= SEGMENT_REG_HS) {
                if (!attr.writable && (mode == Write || storeCheck))
                    return std::make_shared<GeneralProtection>(0);
                if (!attr.readable && mode == Read)
                    return std::make_shared<GeneralProtection>(0);
                expandDown = attr.expandDown;

            }
            Addr base = tc->readMiscRegNoEffect(MISCREG_SEG_BASE(seg));
            Addr limit = tc->readMiscRegNoEffect(MISCREG_SEG_LIMIT(seg));
            bool sizeOverride = (flags & (AddrSizeFlagBit << FlagShift));
            unsigned logSize = sizeOverride ? (unsigned)m5Reg.altAddr
                                            : (unsigned)m5Reg.defAddr;
            int size = (1 << logSize) * 8;
            Addr offset = bits(vaddr - base, size - 1, 0);
            Addr endOffset = offset + req->getSize() - 1;
            if (expandDown) {
                DPRINTF(TLB, "Checking an expand down segment.\n");
                warn_once("Expand down segments are untested.\n");
                if (offset <= limit || endOffset <= limit)
                    return std::make_shared<GeneralProtection>(0);
            } else {
                if (offset > limit || endOffset > limit)
                    return std::make_shared<GeneralProtection>(0);
            }
        }
        if (m5Reg.submode != SixtyFourBitMode ||
                (flags & (AddrSizeFlagBit << FlagShift)))
            vaddr &= mask(32);
        // If paging is enabled, do the translation.
        if (m5Reg.paging) {
            DPRINTF(TLB, "Paging enabled.\n");
            // The vaddr already has the segment base applied.
            TlbEntry *entry = lookup(vaddr);
            if (mode == Read) {
                rdAccesses_l2++;
            } else {
                wrAccesses_l2++;
            }
            Addr raw_paddr = 0;
            bool anchorHit = false;
            if (!entry) {
                DPRINTF(TLB, "Handling a TLB miss for "
                        "address %#x at pc %#x.\n",
                        vaddr, tc->instAddr());
                //DPRINTF(TLB_BAN, "Handling a TLB miss for address %#x at pc %#x.\n", vaddr, tc->instAddr());
                // advance in AnchorEntry lookup
                Process *p = tc->getProcessPtr();
                anchorDistance1 = p->pTable->anchorDistance1;
                pageSize = 4096; 
                Addr alignedVaddr = p->pTable->pageAlign(vaddr);
                int rem;
                if ((alignedVaddr/pageSize)%anchorDistance1 !=0){
                    Addr anchor_addr =  ((alignedVaddr/pageSize)&~(anchorDistance1-1))*pageSize;
                    //DPRINTF(TLB_ANCHOR, "Check anchor! vaddr is %d,  anchv is %d, anchorDis is %d\n", alignedVaddr/pageSize, anchor_addr/pageSize, anchorDistance1);
                    TlbEntry * anchorEntry = lookup(anchor_addr);
                    //DPRINTF(TLB_ANCHOR, "Handle a miss\n");    
                    if (anchorEntry) {
                      //  DPRINTF(TLB_ANCHOR, "find the anchor entry,and its contiguity is %d\n", anchorEntry->contiguity);
                        rem = (alignedVaddr/pageSize)%anchorDistance1;
                        if (anchorEntry->validBit[rem]){
                            // anchor hit
                            raw_paddr = (anchorEntry->base_paddr + anchorEntry->subEntry[rem])*pageSize;
                            anchorHit = true;
                            anHits++;
                          

                           // DPRINTF(TLB_ANCHOR, "anchor hit!\n");
                            // check

                            if(anchorHit){               
                                const EmulationPageTable::Entry *pte2 = p->pTable->lookup(alignedVaddr);
                                if (pte2){
                                    if (raw_paddr == pte2->paddr){
                                        DPRINTF(TLB_ANCHOR, "anwser right!\n"); 
                                        } else{
                                        DPRINTF(TLB_ANCHOR, "anwser wrong!, raw is %d, pad is %d\n", raw_paddr, pte2->paddr);
                                        raw_paddr = pte2->paddr;
                                        //anchorEntry->contiguity = 0;
                                        }
                                } else{
                                        DPRINTF(TLB_ANCHOR, "check fault\n");} 
                             }

                            TlbEntry entry1 = TlbEntry(p->pTable->pid(), vaddr, raw_paddr,
                                             anchorEntry->uncacheable,
                                             !anchorEntry->writable, false, 0, 0, 0, NULL,NULL ) ;
                            entry = &entry1;
                            

                                                                    
                        } 
                    }
                }
               
                           



                if (!anchorHit){
                    if (mode == Read) {
                        rdMisses_l2++;
                    } else {
                        wrMisses_l2++;}
                    L2MISS++;
                    
                    if (FullSystem) {
                       // DPRINTF(TLB_BAN,"fullsystem mode for handling the miss\n");
                        Fault fault = walker->start(tc, translation, req, mode);
                        if (timing || fault != NoFault) {
                            // This gets ignored in atomic mode.
                            delayedResponse = true;
                            return fault;
                        }
                        entry = lookup(vaddr);
                        raw_paddr = entry->paddr;
                        assert(entry);
                    } else {
                        DPRINTF(TLB,"non-full-system for hanling the miss\n");
                        //Process *p = tc->getProcessPtr();
                        const EmulationPageTable::Entry * pte = p->pTable->lookup_tlb(vaddr);
                        
                        //DPRINTF(TLB_BAN,"pagewalk start------\n");
                        if (!pte && mode != Execute) {
                            // Check if we just need to grow the stack.
                            if (p->fixupStackFault(vaddr)) {
                                // If we did, lookup the entry for the new page.
                                pte = p->pTable->lookup_tlb(vaddr);
                            }
                        }
                        if (!pte) {
                            return std::make_shared<PageFault>(vaddr, true, mode,
                                                               true, false);
                        } else {
                            //Addr alignedVaddr = p->pTable->pageAlign(vaddr);
                            //DPRINTF(TLB_BAN,"vaddr is %d, alignedVaddr is %d. \n",vaddr, alignedVaddr);
                           
                            if (pte->isAnchor){
                            //DPRINTF(TLB_ANCHOR, "THIS IS AN ANCHOR, anchorDistance = %d, contiguity = %d  \n",pte->anchorDistance, pte->contiguity);
                            }
                            entry = insert(pte->vaddr, TlbEntry(
                                    p->pTable->pid(), pte->vaddr, pte->paddr,
                                    pte->flags & EmulationPageTable::Uncacheable,
                                    pte->flags & EmulationPageTable::ReadOnly, pte->isAnchor, pte->contiguity, pte->anchorDistance, pte->base_paddr, pte->validBit, pte->subEntry));
                        }
                        DPRINTF(TLB, "Miss was serviced.\n");
                        if (pte->isAnchor){
                            raw_paddr = pte->req_paddr;
                            //raw_paddr = pte->paddr;
                            /**
                            const EmulationPageTable::Entry *pte1 = p->pTable->lookup(vaddr);
                            if (raw_paddr == pte1->paddr){
                                DPRINTF(TLB, "anwser right!\n"); } else{
                                //DPRINTF(TLB_ANCHOR, "here anwser wrong!__________________________________\n");
                                }
                            **/
                        } else{
                            raw_paddr = pte->paddr;
                        }
                    }
                    
                }    
            } else {
                // DPRINTF(MMU,"TLB hit! for the vaddr %#x. And its mapping is %#x \n",vaddr,entry->paddr);
                raw_paddr = entry->paddr;
            }

           // DPRINTF(TLB, "Entry found with paddr %#x,doing protection checks.\n", entry->paddr);
            // Do paging protection checks.
            bool inUser = (m5Reg.cpl == 3 &&
                    !(flags & (CPL0FlagBit << FlagShift)));
            CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);
            bool badWrite = (!entry->writable && (inUser || cr0.wp));
            if ((inUser && !entry->user) || (mode == Write && badWrite)) {
                // The page must have been present to get into the TLB in
                // the first place. We'll assume the reserved bits are
                // fine even though we're not checking them.
                return std::make_shared<PageFault>(vaddr, true, mode, inUser,
                                                   false);
            }
            if (storeCheck && badWrite) {
                // This would fault if this were a write, so return a page
                // fault that reflects that happening.
                return std::make_shared<PageFault>(vaddr, true, Write, inUser,
                                                   false);
            }

            //Addr paddr = raw_paddr | (vaddr & mask(12));
            Addr paddr = raw_paddr | (vaddr & mask(entry->logBytes));
            //DPRINTF(TLB_ANCHOR, "padd is %#x, raw_paddr is %#x\n", paddr, raw_paddr);
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
            req->setPaddr(paddr);
            if (entry->uncacheable)
                req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
        } else {
            //Use the address which already has segmentation applied.
            DPRINTF(TLB, "Paging disabled.\n");
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
            req->setPaddr(vaddr);
        }
    } else {
        // Real mode
        DPRINTF(TLB, "In real mode.\n");
        DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
        req->setPaddr(vaddr);
    }

    return finalizePhysical(req, tc, mode);
}

Fault
TLB_L2::translateAtomic(RequestPtr req, ThreadContext *tc, Mode mode)
{
    bool delayedResponse;
    return TLB_L2::translate(req, tc, NULL, mode, delayedResponse, false);
}

void
TLB_L2::translateTiming(RequestPtr req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    bool delayedResponse;
    assert(translation);
    Fault fault =
        TLB_L2::translate(req, tc, translation, mode, delayedResponse, true) ;
    if (!delayedResponse)
        translation->finish(fault, req, tc, mode);
}

Walker *
TLB_L2::getWalker()
{
    return walker;
}

void
TLB_L2::regStats()
{
    using namespace Stats;

    rdAccesses_l2
        .name(name() + ".rdAccesses_l2")
        .desc("L2 TLB accesses on read requests");

    wrAccesses_l2
        .name(name() + ".wrAccesses_l2")
        .desc("L2 TLB accesses on write requests");

    rdMisses_l2
        .name(name() + ".rdMisses_l2")
        .desc("L2 TLB misses on read requests");

    wrMisses_l2
        .name(name() + ".wrMisses_l2")
        .desc("L2 TLB misses on write requests");
    anHits
        .name(name() + ".anHits")
        .desc("Anchor TLB entry hit ");

}

void
TLB_L2::serialize(CheckpointOut &cp) const
{
    // Only store the entries in use.
    uint32_t _size = size - freeList.size();
    SERIALIZE_SCALAR(_size);
    SERIALIZE_SCALAR(lruSeq);

    uint32_t _count = 0;
    for (uint32_t x = 0; x < size; x++) {
        if (tlb[x].trieHandle != NULL)
            tlb[x].serializeSection(cp, csprintf("Entry%d", _count++));
    }
}

void
TLB_L2::unserialize(CheckpointIn &cp)
{
    // Do not allow to restore with a smaller tlb.
    uint32_t _size;
    UNSERIALIZE_SCALAR(_size);
    if (_size > size) {
        fatal("TLB size less than the one in checkpoint!");
    }

    UNSERIALIZE_SCALAR(lruSeq);

    for (uint32_t x = 0; x < _size; x++) {
        TlbEntry *newEntry = freeList.front();
        freeList.pop_front();

        newEntry->unserializeSection(cp, csprintf("Entry%d", x));
        newEntry->trieHandle = trie.insert(newEntry->vaddr,
            TlbEntryTrie::MaxBits - newEntry->logBytes, newEntry);
    }
}

BaseMasterPort *
TLB_L2::getMasterPort()
{
    return &walker->getMasterPort("port");
}

} // namespace X86ISA
/**
X86ISA::TLB_L2 *
X86TLBParams::create()
{
    return new X86ISA::TLB_L2(this);
}
**/
