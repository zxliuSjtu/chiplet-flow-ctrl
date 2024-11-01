/*
 * Copyright (c) 2020 Inria
 * Copyright (c) 2016 Georgia Institute of Technology
 * Copyright (c) 2008 Princeton University
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
 */

#include "mem/ruby/network/garnet/InputUnit.hh"

#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/Credit.hh"
#include "mem/ruby/network/garnet/NetworkInterface.hh"
#include "mem/ruby/network/garnet/Router.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

InputUnit::InputUnit(int id, PortDirection direction, Router *router)
  : Consumer(router), m_router(router), m_id(id), m_direction(direction),
    m_vc_per_vnet(m_router->get_vc_per_vnet())
{
    const int m_num_vcs = m_router->get_num_vcs();
    m_num_buffer_reads.resize(m_num_vcs/m_vc_per_vnet);
    m_num_buffer_writes.resize(m_num_vcs/m_vc_per_vnet);
    for (int i = 0; i < m_num_buffer_reads.size(); i++) {
        m_num_buffer_reads[i] = 0;
        m_num_buffer_writes[i] = 0;
    }

    // Instantiating the virtual channels
    virtualChannels.reserve(m_num_vcs);
    for (int i=0; i < m_num_vcs; i++) {
        virtualChannels.emplace_back();
    }
}

/*
 * The InputUnit wakeup function reads the input flit from its input link.
 * Each flit arrives with an input VC.
 * For HEAD/HEAD_TAIL flits, performs route computation,
 * and updates route in the input VC.
 * The flit is buffered for (m_latency - 1) cycles in the input VC
 * and marked as valid for SwitchAllocation starting that cycle.
 *
 */

void
InputUnit::wakeup()
{
    flit *t_flit;
    std::cout<<"InputUnit::wakeup()"<<std::endl;
    if (m_in_link->isReady(curTick())) {
        std::cout<<"get a flit from link"<<std::endl;
        t_flit = m_in_link->consumeLink();
        DPRINTF(RubyNetwork, "Router[%d] Consuming:%s Width: %d Flit:%s\n",
        m_router->get_id(), m_in_link->name(),
        m_router->getBitWidth(), *t_flit);
        assert(t_flit->m_width == m_router->getBitWidth());
        int vc = t_flit->get_vc();
        t_flit->increment_hops(); // for stats

        if ((t_flit->get_type() == HEAD_) ||
            (t_flit->get_type() == HEAD_TAIL_)) {

            assert(virtualChannels[vc].get_state() == IDLE_);
            set_vc_active(vc, curTick());

            // Route computation for this vc
            int outport = m_router->route_compute(t_flit->get_route(),
                m_id, m_direction);

            // Update output port in VC
            // All flits in this packet will use this output port
            // The output port field in the flit is updated after it wins SA
            grant_outport(vc, outport);

        } else {
            assert(virtualChannels[vc].get_state() == ACTIVE_);
        }


        // Buffer the flit
        virtualChannels[vc].insertFlit(t_flit);

        int vnet = vc/m_vc_per_vnet;
        // number of writes same as reads
        // any flit that is written will be read only once
        m_num_buffer_writes[vnet]++;
        m_num_buffer_reads[vnet]++;

        Cycles pipe_stages = m_router->get_pipe_stages();
        if (pipe_stages == 1) {
            // 1-cycle router
            // Flit goes for SA directly
            t_flit->advance_stage(SA_, curTick());
        } else {
            assert(pipe_stages > 1);
            // Router delay is modeled by making flit wait in buffer for
            // (pipe_stages cycles - 1) cycles before going for SA

            Cycles wait_time = pipe_stages - Cycles(1);
            t_flit->advance_stage(SA_, m_router->clockEdge(wait_time));

            // Wakeup the router in that cycle to perform SA
            m_router->schedule_wakeup(Cycles(wait_time));
        }

        if (m_in_link->isReady(curTick())) {
            m_router->schedule_wakeup(Cycles(1));
        }
    }
}

// Send a credit back to upstream router for this VC.
// Called by SwitchAllocator when the flit in this VC wins the Switch.
void
InputUnit::increment_credit(int in_vc, bool free_signal, Tick curTime)
{
    DPRINTF(RubyNetwork, "Router[%d]: Sending a credit vc:%d free:%d to %s\n",
    m_router->get_id(), in_vc, free_signal, m_credit_link->name());
    Credit *t_credit = new Credit(in_vc, free_signal, curTime);
    creditQueue.insert(t_credit);
    m_credit_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
}

bool
InputUnit::functionalRead(Packet *pkt, WriteMask &mask)
{
    bool read = false;
    for (auto& virtual_channel : virtualChannels) {
        if (virtual_channel.functionalRead(pkt, mask))
            read = true;
    }

    return read;
}

uint32_t
InputUnit::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (auto& virtual_channel : virtualChannels) {
        num_functional_writes += virtual_channel.functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
InputUnit::resetStats()
{
    for (int j = 0; j < m_num_buffer_reads.size(); j++) {
        m_num_buffer_reads[j] = 0;
        m_num_buffer_writes[j] = 0;
    }
}

bool InputUnit::MakeFastTransmission(int vc)
{
    if (m_router->get_net_ptr()->m_cfc == 1)
    {
        flit *t_flit;

        if (virtualChannels[vc].isEmpty())
        {
            std::cout<<"fail because empty"<<std::endl;
            return false;
        }
        // check the des router is ready to accept the packe
        int desRouterId = (virtualChannels[vc].peekTopFlit())
                            ->get_route().dest_router;
        //std::cout<<"desRouterId: "<<desRouterId<<std::endl;
        int numRows = m_router->get_net_ptr()->getNumRows();
        // get des router region
        int desRouterRegion = \
            m_router->RegionNumber(numRows, numRows, desRouterId);
        //std::cout<<"desRouterRegion: "<<desRouterRegion<<std::endl;
        // check des router region is if des region
        int desRegionType = m_router->timeSlotType(desRouterRegion);
        //std::cout<<"desRegionType: "<<desRegionType<<std::endl;
        // if des region is not ready, return false

        // return false if src/des are not on same chiplet
        /* if (desRouterId/(numRows * numRows) !=
             m_router->get_id()/(numRows * numRows))*/
        if (false)
        {
                /*std::cout<<"fastTransmission fail because des/src
                are on different chiplet"<<std::endl;*/
                return false;
        }
        else if (desRegionType == -1)
        {
            std::cout<<"fastTransmission fail because desRegionType:"
                     <<desRegionType<<std::endl;
            return false;
        }
        else {
            // todo: found a bug! fix this
            // even if fasttransmission failed
            // flit still will be deleted.
            //std::cout<<"getTopFlit() from vc: "<<vcBase<<std::endl;
            t_flit = virtualChannels[vc].getTopFlit();
        }

        DPRINTF(RubyNetwork, "[InputUnit::make_pkt_bufferless()] "\
                "Inport direction for which we are ejecting packet: %s\n",\
                 m_direction);
        DPRINTF(RubyNetwork,"%s \n", *t_flit);

        int chipletLinkLatency = 1;
        int interposerLinkLatency = 1;
        int routerLatency = 0;

        int latency = LatencyCompute(t_flit, \
                                     chipletLinkLatency, \
                                     routerLatency, \
                                     interposerLinkLatency);

        // std::cout << "latencycompute over at vnet: "<<vnet<<std::endl;

        int dest_ni = t_flit->get_route().dest_ni;
        std::vector<NetworkInterface *> NIs=m_router->get_net_ptr()->getNIs();
        // if (t_flit->get_type() == TAIL_) {std::cout << "TAIL_";}
        // if (t_flit->get_type() == HEAD_TAIL_) {std::cout << "HEAD_TAIL_";}
        // if (t_flit->get_type() == BODY_) {std::cout << "BODY_";}
        // std::cout << std::endl;


        // m_cfcPacketBuffer is just a temp buffer to save t_flit
        // just compute hops, so make input as (t_flit,0,1,0)
        for (int hops = 0; hops <= LatencyCompute(t_flit,0,1,0); hops++){
            t_flit->increment_hops();
        }
        NIs[dest_ni] -> m_cfcPacketBuffer -> insert(t_flit);
        //NIs[dest_ni] -> m_cfcPacketBuffer -> print(std::cout);
        NIs[dest_ni] -> ConsumeCfcPacket(latency);
        //NIs[dest_ni] -> m_cfcPacketBuffer -> print(std::cout);

        m_router->add_num_cfcpkt();
        // enqueue this pkt in dest_ni's buffer and set a dequeue time
        // std::cout << "Mk a fast transmission at vnet: "<< vnet << std::endl;

        increment_credit(vc, true, m_router->curCycle());
        set_vc_idle(vc, m_router->curCycle());

        return true;

    }
    return false;
}

bool InputUnit::MakeFastPass(int vnet)
{
    if (m_router->get_net_ptr()->m_fastpass == 1)
    {
        int vcBase = vnet * m_vc_per_vnet;
        flit *t_flit;

        if (virtualChannels[vcBase].isEmpty())
        {
            return false;
        }
        // check the des router is ready to accept the packe
        int desRouterId = (virtualChannels[vcBase].peekTopFlit())
                            ->get_route().dest_router;
        std::cout<<"desRouterId: "<<desRouterId<<std::endl;

        int numRows = m_router->get_net_ptr()->getNumRows();
        // get des router region
        int desRouterRegion = desRouterId % numRows;
        std::cout<<"desRouterRegion: "<<desRouterRegion<<std::endl;
        // check des router region is if des region
        int srcRouterRegion = (m_router->get_id()) % numRows;
        // if des region is not ready, return false
        int slotLength = m_router->get_net_ptr()->get_slotlength();
        int slotNum = m_router->curCycle()/slotLength;
        // see the gem5::ruby::garnet::Router::FastpassTurn()
        int allowedDesRegion = \
        (srcRouterRegion + (slotNum % numRows)) % numRows;
        std::cout<<"allowed desregion: "<<allowedDesRegion<<std::endl;

        if (desRouterRegion != allowedDesRegion)
        {
            return false;
        }
        else {
            t_flit = virtualChannels[vcBase].getTopFlit();
        }

        int chipletLinkLatency = 1;
        int interposerLinkLatency = 1;
        int routerLatency = 1;

        int latency = LatencyCompute(t_flit, \
                                     chipletLinkLatency, \
                                     routerLatency, \
                                     interposerLinkLatency);

        int dest_ni = t_flit->get_route().dest_ni;
        std::vector<NetworkInterface *> NIs=m_router->get_net_ptr()->getNIs();

        for (int hops = 0; hops <= LatencyCompute(t_flit,0,1,0); hops++){
            t_flit->increment_hops();
        }
        NIs[dest_ni] -> m_cfcPacketBuffer -> insert(t_flit);
        NIs[dest_ni] -> ConsumeCfcPacket(latency);
        m_router->add_num_cfcpkt();

        increment_credit(vcBase, true, m_router->curCycle());
        set_vc_idle(vcBase, m_router->curCycle());

        return true;

    }
    return false;
}


int InputUnit::LatencyCompute \
(flit* pFlit, int linkLatency, int routerLatency, int interposerLinkLatency)
{
    int resultHops = -1;
    int resultLatency = -1;

    int numRows = m_router->get_net_ptr()->getNumRows();
    assert(numRows < 10); //deprecate it when need

    int myRouterId = m_router->get_id();
    int myOnchipRouterId = myRouterId % (numRows * numRows);
    int myChipletId = myRouterId / (numRows * numRows);
    int myX = myOnchipRouterId % numRows;
    int myY = myOnchipRouterId / numRows;

    int destRouterId = pFlit->get_route().dest_router;
    int destOnchipRouterId = destRouterId % (numRows * numRows);
    int destChipletId = destRouterId / (numRows * numRows);
    // int destNI = pFlit->get_route().dest_ni;
    int destX = destOnchipRouterId % numRows;
    int destY = destOnchipRouterId / numRows;

    // if src and dest are on the same chiplet
    if (destChipletId == myChipletId)
    {
        int xHops = abs(destX - myX);
        int yHops = abs(destY - myY);
        resultHops = xHops + yHops;

        if (myOnchipRouterId == destOnchipRouterId)
        {
            resultHops = 1;
        }
        assert(resultHops > 0);
        resultLatency = (linkLatency + routerLatency) * resultHops + 1;
        std::cout<<"myRouterId: "<<myRouterId<<std::endl;
        std::cout<<"destRouterId: "<<destRouterId<<std::endl;
        std::cout<<"resultHops: "<<resultHops<<std::endl;
        std::cout<<"resultLatency: "<<resultLatency<<std::endl;
        assert(resultLatency > 0);
        return resultLatency;
    }
    else
    {   // src and dest are not on the same chiplet
        // find the two boundary router used
        int srcBoundaryRouterId = \
            m_router->GetBoundaryRouter(myRouterId);
        int destBoundaryRouterId = \
            m_router->GetBoundaryRouter(destRouterId);

        // find the corresponding interposer router
        int srcInterposerRouterId = \
            m_router->BoundaryToInterposer(srcBoundaryRouterId);
        int destInterposerRouterId = \
            m_router->BoundaryToInterposer(destBoundaryRouterId);

        int srcBrX = (srcBoundaryRouterId % (numRows * numRows))\
            % numRows;
        int srcBrY = (srcBoundaryRouterId % (numRows * numRows))\
            / numRows;

        int destBrX = (destBoundaryRouterId % (numRows * numRows))\
            % numRows;
        int destBrY = (destBoundaryRouterId % (numRows * numRows))\
            / numRows;

        int srcIrX = (srcInterposerRouterId % (numRows * numRows))\
            % numRows;
        int srcIrY = (srcInterposerRouterId % (numRows * numRows))\
            / numRows;

        int destIrX = (destInterposerRouterId % (numRows * numRows))\
            % numRows;
        int destIrY = (destInterposerRouterId % (numRows * numRows))\
            / numRows;

        int chipletXHops = abs(srcBrX - myX) + \
                           abs(destX - destBrX);

        int chipletYHops = abs(srcBrY - myY) + \
                           abs(destY - destBrY);

        int interposerXHops = abs(srcIrX - destIrX);
        int interposerYHops = abs(srcIrY - destIrY);

        int chipletLatency = (linkLatency + routerLatency) * \
                             (chipletXHops + chipletYHops);
        int interposerLatency= (interposerLinkLatency + routerLatency) * \
                               (interposerYHops + interposerXHops) + \
                               2 * interposerLinkLatency; //vertical link
        resultLatency = chipletLatency + interposerLatency;
        assert(resultLatency > 0);
        return resultLatency;
    }

}

} // namespace garnet
} // namespace ruby
} // namespace gem5
