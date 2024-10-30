/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
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


#include "mem/ruby/network/garnet/Router.hh"

#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/CreditLink.hh"
#include "mem/ruby/network/garnet/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/InputUnit.hh"
#include "mem/ruby/network/garnet/NetworkLink.hh"
#include "mem/ruby/network/garnet/OutputUnit.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

Router::Router(const Params &p)
  : BasicRouter(p), Consumer(this), m_latency(p.latency),
    m_virtual_networks(p.virt_nets), m_vc_per_vnet(p.vcs_per_vnet),
    m_num_vcs(m_virtual_networks * m_vc_per_vnet), m_bit_width(p.width),
    m_network_ptr(nullptr), routingUnit(this), switchAllocator(this),
    crossbarSwitch(this)
{
    m_input_unit.clear();
    m_output_unit.clear();
    m_cfc_vnet_ptr = 0;//update in rr-fashion
    m_cur_num_cfcpkt = 0;
    m_vTimeSlot = {
            {0, 1, 3, 6},
            {1, 2, 4, 7},
            {3, 4, 5, 8},
            {6, 7, 8, 9}
        };
    // m_vTimeSlot = {
    //     {0, 1, 3, 6, 10, 15, 21, 28},
    //     {1, 2, 4, 7, 11, 16, 22, 29},
    //     {3, 4, 5, 8, 12, 17, 23, 30},
    //     {6, 7, 8, 9, 13, 18, 24, 31},
    //     {10, 11, 12, 13, 14, 19, 25, 32},
    //     {15, 16, 17, 18, 19, 20, 26, 33},
    //     {21, 22, 23, 24, 25, 26, 27, 34},
    //     {28, 29, 30, 31, 32, 33, 34, 35}
    // };
}

void
Router::init()
{
    BasicRouter::init();

    switchAllocator.init();
    crossbarSwitch.init();
}

int
Router::get_numFreeVC(PortDirection dirn_) {
    // Caution: This 'dirn_' is the direction of inport
    // of downstream router...
    assert(dirn_ != "Local");
    int inport_id = routingUnit.get_map_d2i(dirn_); // ahh.. this is wrong!
    return (m_input_unit[inport_id]->get_numFreeVC(dirn_));
}

void
Router::wakeup()
{
    //std::cout<<"enter 0"<<std::endl;
    DPRINTF(RubyNetwork, "Router %d woke up\n", m_id);

    // check for incoming flits
    for (int inport = 0; inport < m_input_unit.size(); inport++) {
        m_input_unit[inport]->wakeup();
        std::cout<<"vc state: "<<\
        m_input_unit[inport]->isReady(0,curTick())<<\
        m_input_unit[inport]->isReady(1,curTick())<<\
        m_input_unit[inport]->isReady(2,curTick())<<\
        m_input_unit[inport]->isReady(3,curTick())<<\
        std::endl;
    }
    std::cout<<std::endl;
    assert(clockEdge() == curTick());
    bool isCfcTurn = CfcTurn();

    int sendCfc = 0;
    bool sendFastpass = false;
    if (isCfcTurn) {
        std::cout<<"router "\
        <<m_id<< "wakeup as cfc, at cycle: "\
        <<curCycle()<<std::endl;
        std::cout<<"m_input_unit.size(): "\
        <<m_input_unit.size()<<std::endl;
        for (int inport = m_input_unit.size()-1; inport >= 0; inport--) {
            //std::cout<<"check inport: "<<inport<<std::endl;
            // loop all inports and all vcs
            // find a ready cfc pkt
            // if find one then break
            // give a prioirty to "in" direction of br
            if (m_cur_num_cfcpkt <= get_net_ptr()->m_max_num_cfcpkt){
                //std::cout<<"enter 2"<<std::endl;
                //std::cout<<"m_cfc_vnet_ptr"<<m_cfc_vnet_ptr<<std::endl;
                //std::cout<<"inport"<<inport<<std::endl;
                //use vnet ptr as a vc ptr
                while (m_cfc_vnet_ptr < get_vc_per_vnet()){
                    std::cout<<"m_virtual_channels: "\
                    <<m_virtual_networks<<std::endl;
                    std::cout<<"check vc: "<<m_cfc_vnet_ptr<<std::endl;

                    if (m_input_unit[inport]->\
                        MakeFastTransmission(m_cfc_vnet_ptr)){
                        //std::cout<<"enter 3"<<std::endl;
                        //return false if vc is empty
                        //return false if des router is not des region
                        //return false if src/des are not on same chiplet
                        std::cout<<"CFC SEND"<<std::endl;
                        std::cout<<"number of vn: "\
                        <<m_virtual_networks<<std::endl;
                        sendCfc++;
                        std::cout<<"At cycle: "<<curCycle();
                        std::cout<<" Router "<<
                        m_id<<" make CFC"<<std::endl;
                        std::cout<<"at vc:"<<m_cfc_vnet_ptr<<std::endl;
                        std::cout<<"at inport:"<<inport<<std::endl;
                        break;
                    }
                    else{
                        m_cfc_vnet_ptr++;
                    }
                }
                m_cfc_vnet_ptr = 0;
                //std::cout<<"out 3"<<std::endl;
            }
            //std::cout<<"out 2"<<std::endl;

            if (sendCfc > 4){
                break;
            }

        }
        //std::cout<<"out 1"<<std::endl;
        //m_cfc_vnet_ptr++;
        //std::cout<<"m_cfc_vnet_ptr"<<m_cfc_vnet_ptr<<std::endl;
        // if (m_cfc_vnet_ptr == m_virtual_networks){
        //     m_cfc_vnet_ptr = 0;
        // }
    }

    if (FastpassTurn()){
        std::cout<<"router "\
        <<m_id<< "wakeup as fastpass, at cycle: "\
        <<curCycle()<<std::endl;
        std::cout<<"FASTPASS WAKEUP"<<std::endl;
        for (int inport = m_input_unit.size()-1; inport >= 0; inport--) {
            //we can reuse the cfc_vnet_ptr
            while (m_cfc_vnet_ptr < m_virtual_networks){
                if (m_input_unit[inport]->\
                    MakeFastPass(m_cfc_vnet_ptr)){
                    std::cout<<"FASTPASS SEND"<<std::endl;
                    std::cout<<"At cycle: "<<curCycle();
                    std::cout<<" Router "<<
                    m_id<<" make FASTPASSS"<<std::endl;
                    std::cout<<"at vnet:"<<m_cfc_vnet_ptr<<std::endl;
                    std::cout<<"at inport:"<<inport<<std::endl;
                    sendFastpass = true;
                    break;
                }
                else{
                    m_cfc_vnet_ptr++;
                }
            }
                m_cfc_vnet_ptr = 0;

            if (sendFastpass){
                break;
            }
        }
    }


    // check for incoming credits
    // Note: the credit update is happening before SA
    // buffer turnaround time =
    //     credit traversal (1-cycle) + SA (1-cycle) + Link Traversal (1-cycle)
    // if we want the credit update to take place after SA, this loop should
    // be moved after the SA request
    for (int outport = 0; outport < m_output_unit.size(); outport++) {
        m_output_unit[outport]->wakeup();
    }

    // Switch Allocation
    switchAllocator.wakeup();

    // Switch Traversal
    crossbarSwitch.wakeup();

    if (isCfcTurn) {
        m_cur_num_cfcpkt = 0;
    }
}

void
Router::addInPort(PortDirection inport_dirn,
                  NetworkLink *in_link, CreditLink *credit_link)
{
    fatal_if(in_link->bitWidth != m_bit_width, "Widths of link %s(%d)does"
            " not match that of Router%d(%d). Consider inserting SerDes "
            "Units.", in_link->name(), in_link->bitWidth, m_id, m_bit_width);

    int port_num = m_input_unit.size();
    InputUnit *input_unit = new InputUnit(port_num, inport_dirn, this);

    input_unit->set_in_link(in_link);
    input_unit->set_credit_link(credit_link);
    in_link->setLinkConsumer(this);
    in_link->setVcsPerVnet(get_vc_per_vnet());
    credit_link->setSourceQueue(input_unit->getCreditQueue(), this);
    credit_link->setVcsPerVnet(get_vc_per_vnet());

    m_input_unit.push_back(std::shared_ptr<InputUnit>(input_unit));

    routingUnit.addInDirection(inport_dirn, port_num);
}

void
Router::addOutPort(PortDirection outport_dirn,
                   NetworkLink *out_link,
                   std::vector<NetDest>& routing_table_entry, int link_weight,
                   CreditLink *credit_link, uint32_t consumerVcs)
{
    fatal_if(out_link->bitWidth != m_bit_width, "Widths of units do not match."
            " Consider inserting SerDes Units");

    int port_num = m_output_unit.size();
    OutputUnit *output_unit = new OutputUnit(port_num, outport_dirn, this,
                                             consumerVcs);

    output_unit->set_out_link(out_link);
    output_unit->set_credit_link(credit_link);
    credit_link->setLinkConsumer(this);
    credit_link->setVcsPerVnet(consumerVcs);
    out_link->setSourceQueue(output_unit->getOutQueue(), this);
    out_link->setVcsPerVnet(consumerVcs);

    m_output_unit.push_back(std::shared_ptr<OutputUnit>(output_unit));

    routingUnit.addRoute(routing_table_entry);
    routingUnit.addWeight(link_weight);
    routingUnit.addOutDirection(outport_dirn, port_num);
}

PortDirection
Router::getOutportDirection(int outport)
{
    return m_output_unit[outport]->get_direction();
}

PortDirection
Router::getInportDirection(int inport)
{
    return m_input_unit[inport]->get_direction();
}

int
Router::route_compute(RouteInfo route, int inport, PortDirection inport_dirn)
{
    return routingUnit.outportCompute(route, inport, inport_dirn);
}

void
Router::grant_switch(int inport, flit *t_flit)
{
    crossbarSwitch.update_sw_winner(inport, t_flit);
}

void
Router::schedule_wakeup(Cycles time)
{
    // wake up after time cycles
    scheduleEvent(time);
}

std::string
Router::getPortDirectionName(PortDirection direction)
{
    // PortDirection is actually a string
    // If not, then this function should add a switch
    // statement to convert direction to a string
    // that can be printed out
    return direction;
}

void
Router::regStats()
{
    BasicRouter::regStats();

    m_buffer_reads
        .name(name() + ".buffer_reads")
        .flags(statistics::nozero)
    ;

    m_buffer_writes
        .name(name() + ".buffer_writes")
        .flags(statistics::nozero)
    ;

    m_crossbar_activity
        .name(name() + ".crossbar_activity")
        .flags(statistics::nozero)
    ;

    m_sw_input_arbiter_activity
        .name(name() + ".sw_input_arbiter_activity")
        .flags(statistics::nozero)
    ;

    m_sw_output_arbiter_activity
        .name(name() + ".sw_output_arbiter_activity")
        .flags(statistics::nozero)
    ;
}

void
Router::collateStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_buffer_reads += m_input_unit[i]->get_buf_read_activity(j);
            m_buffer_writes += m_input_unit[i]->get_buf_write_activity(j);
        }
    }

    m_sw_input_arbiter_activity = switchAllocator.get_input_arbiter_activity();
    m_sw_output_arbiter_activity =
        switchAllocator.get_output_arbiter_activity();
    m_crossbar_activity = crossbarSwitch.get_crossbar_activity();
}

void
Router::resetStats()
{
    for (int i = 0; i < m_input_unit.size(); i++) {
            m_input_unit[i]->resetStats();
    }

    crossbarSwitch.resetStats();
    switchAllocator.resetStats();
}

void
Router::printFaultVector(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    int num_fault_types = m_network_ptr->fault_model->number_of_fault_types;
    float fault_vector[num_fault_types];
    get_fault_vector(temperature_celcius, fault_vector);
    out << "Router-" << m_id << " fault vector: " << std::endl;
    for (int fault_type_index = 0; fault_type_index < num_fault_types;
         fault_type_index++) {
        out << " - probability of (";
        out <<
        m_network_ptr->fault_model->fault_type_to_string(fault_type_index);
        out << ") = ";
        out << fault_vector[fault_type_index] << std::endl;
    }
}

void
Router::printAggregateFaultProbability(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    float aggregate_fault_prob;
    get_aggregate_fault_probability(temperature_celcius,
                                    &aggregate_fault_prob);
    out << "Router-" << m_id << " fault probability: ";
    out << aggregate_fault_prob << std::endl;
}

bool
Router::functionalRead(Packet *pkt, WriteMask &mask)
{
    bool read = false;
    if (crossbarSwitch.functionalRead(pkt, mask))
        read = true;

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        if (m_input_unit[i]->functionalRead(pkt, mask))
            read = true;
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        if (m_output_unit[i]->functionalRead(pkt, mask))
            read = true;
    }

    return read;
}

uint32_t
Router::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    num_functional_writes += crossbarSwitch.functionalWrite(pkt);

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        num_functional_writes += m_input_unit[i]->functionalWrite(pkt);
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        num_functional_writes += m_output_unit[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

/**
 * Determines whether the router should turn on
 * as the CFC (Chiplet Flow Controller).
 * Only chiplet routers with IDs less than 64 should wake up as CFC.
 * The global ID is converted into an on-chip ID.
 * The router checks if it is in the valid time slot based
 * on the current cycle and region number.
 *
 * @return true if the router should turn on as CFC, false otherwise.
 */
bool
Router::CfcTurn()
{
    bool result = false;
    int numRows = get_net_ptr()->getNumRows();
    int numCols = numRows;
    assert(numRows == numCols);
    int myId = get_id();
    //only chiplet router should wake up as cfc
    if (myId >= 64)
    {
        return false;
    }
    //convert global ID into on chip ID
    int myOnchipId = myId % (numRows * numRows);

    // total region number is same as number of rows
    // int totalRegionNum = numRows;
    //std::cout<<"myID: "<<myId<<std::endl;
    int myRegionNum = RegionNumber(numRows, numRows, myOnchipId);
    //std::cout<<"myID and myRegionNum: "<<myId<<" / "<<myRegionNum<<std::endl;

    if (get_net_ptr()->m_cfc == 1){
        //
        // int timeSlotNum = curCycle() % totalRegionNum;
        // just check if this router is in the valid time slot
        int mySlotType = timeSlotType(myRegionNum);
        if (mySlotType != -1){
            result = true;
            //std::cout<<"Router::CfcTurn()-> return: "<<result<<std::endl;
            //std::cout<<std::endl;
        }
    }
    //std::cout<<"at cycle: "<<curCycle();
    //std::cout<<" router "<<myId<<" is checking cfcturn. Result: ";
    //std::cout<<result<<std::endl;
    return result;
}

bool
Router::FastpassTurn(){
    if (get_net_ptr()->m_fastpass == 0){
        return false;
    }
    bool result = false;
    int numRows = get_net_ptr()->getNumRows();
    int numCols = numRows;
    assert(numRows == numCols);
    int myId = get_id();
        if (myId >= 64)
    {
        return false;
    }
    //std::cout<<std::endl;
    //std::cout<<"myID: "<<myId<<std::endl;
    int myOnchipId = myId % (numRows * numRows);
    // each row is a region in fastpass
    int myRegionNum = myOnchipId % numRows;
    //std::cout<<"myRegionNum: "<<myRegionNum<<std::endl;
    // slotLength is the K in fastpass paper
    int slotLength = get_net_ptr()->get_slotlength();
    int slotNum = curCycle()/slotLength;
    if (curCycle() % slotLength != 0){
        return false;
    }
    // details in fastpass paper figure 1
    int routerBias = slotNum / numRows;
    int fpRouterRow = myRegionNum;
    // 0->1, 1->2, 2->3, 3->0
    int fpRouterCol = (routerBias + myRegionNum) % numRows;
    //std::cout<<"routerBias: "<<routerBias<<std::endl;

    if (get_net_ptr()->m_fastpass == 1){
        if (myId == fpRouterRow + fpRouterCol * numRows){
            //std::cout<<"At slot: "<<slotNum<<" ,router "<<myId;
            //std::cout<<"is fastpass turn"<<std::endl;
            result = true;
        }
    }
    return result;
}

bool
Router::UppTurn(){
    return false;
}

/**
 * Calculates the region number for a given router in a mesh network.
 * @author zxliu
 * @param meshRows The number of rows in the mesh network.
 * @param meshCols The number of columns in the mesh network.
 * @param routerId The ID of the router.
 * @return The region number of the router.
 */
int
Router::RegionNumber(int meshRows, int meshCols, int routerId)
{
    if (routerId > meshRows * meshCols)
    {
        routerId = routerId % (meshRows * meshCols);
    }
    //std::cout<<" myonchipID: "<<routerId;

    int resultRegionNum = 0;
    assert(meshRows == meshCols); //only support nxn mesh

    int onChipX = routerId % meshRows;
    int onChipY = routerId / meshRows;

    // use difference of x and y to divide region
    int differXY = onChipX - onChipY;
    if (differXY >=0)
    {
        resultRegionNum = differXY;
    }
    else
    {
        resultRegionNum = differXY + meshRows;
    }
    //std::cout<<" belongs to Region:"<<resultRegionNum<<std::endl;
    return resultRegionNum;
}

/**
 * Returns the boundary router ID for a given chiplet router ID.
 * @author zxliu
 * @param chipletRouterId The ID of the chiplet router.
 * @return The boundary router ID.
 */
int
Router::GetBoundaryRouter(int chipletRouterId)
{
    int resultBrId = -1;
    int numRows = get_net_ptr()->getNumRows();
    int myId = chipletRouterId;
    int myChipletId = myId / (numRows * numRows);
    //find the frist router of this chiplet
    int chipletBaseId = myChipletId * (numRows * numRows);

    int halfCol = numRows / 2;
    int halfRow = numRows / 2;


    int myOnchipId = myId % (numRows * numRows);
    int myOnchipX = myOnchipId % numRows;
    int myOnchipY = myOnchipId / numRows;

    if (myOnchipX < halfCol && myOnchipY < halfRow)
        {
            resultBrId = chipletBaseId + 1;
        }
    else if (myOnchipX >= halfCol && myOnchipY < halfRow)
        {
            resultBrId = chipletBaseId + 2;
        }
    else if (myOnchipX < halfCol && myOnchipY >= halfRow)
        {
            resultBrId = chipletBaseId + 13;
        }
    else {
        resultBrId = chipletBaseId + 14;
        }

    assert(resultBrId != 0); //find successfully
    return resultBrId;
}

/**
 * Maps a boundary router ID to its corresponding interposer router ID.
 * @author zxliu
 * @param BrId The boundary router ID to be mapped.
 * @return The corresponding interposer router ID.
 * @throws std::out_of_range if the given BrId is not found in the mapping.
 */
int Router::BoundaryToInterposer(int BrId)
{
    std::map<int, int> Br2Ir = {
        {1, 64}, {2, 65}, {13, 68}, {14, 69}, // chiplet 0
        {17, 66}, {18, 67}, {29, 70}, {30, 71}, // chiplet 1
        {33, 72}, {34, 73}, {45, 76}, {46, 77}, // chiplet 2
        {49, 74}, {50, 75}, {61, 78}, {62, 79} // chiplet 4
    };

    auto iter = Br2Ir.find(BrId);
    if (iter != Br2Ir.end()) {
        return iter->second;
    } else {
        assert(0); // wrong br id
    }
}

/**
 * Determines if the current router's time slot is valid for transmission.
 * The time slot is determined based on the current cycle and the predefined
 * time slot matrix. Each router is assigned a region number based on its ID,
 * and the time slot matrix specifies the valid time slots for each region.
 *
 * @param myRegionNum The region number to check against.
 * @return An integer value indicating the type of transmission allowed in the
 *         current time slot:
 *         - 0: Intra-region transmission slot (router can be both destination
 *              and source)
 *         - 1: Destination transmission slot (router can only be destination)
 *         - 2: Source transmission slot (router can only be source)
 *         - -1: Invalid time slot (router cannot transmit in this slot)
 */
int Router::timeSlotType(int myRegionNum){
    // slot need set as vTimeSlot[i][j] == vTimeSlot[j][i]
    // std::cout<<"Router::timeSlotType-> at cycle:"<<curCycle();
    // std::cout<<std::endl;
    //std::cout<<"Router::timeSlotType-> myRegionNum:"<<myRegionNum;
    //std::cout<<std::endl;

    assert(myRegionNum >= 0);
    assert(myRegionNum <= get_net_ptr()->getNumRows());

    int slotLength = get_net_ptr()->get_slotlength();
    if (curCycle() % slotLength != 0){
        return -1;
    }
    int timeSlotNum = (curCycle()/slotLength) % 10;
    std::cout<<"curCycle(): "<<curCycle()<<std::endl;
    std::cout<<"timeSlotNum "<<timeSlotNum<<std::endl;

    // find current des and src region
    int curDesRegion = -1;
    int curSrcRegion = -1;
    for (int i = 0; i < m_vTimeSlot.size(); ++i) {
        for (int j = 0; j < m_vTimeSlot[i].size(); ++j) {
            if (m_vTimeSlot[i][j] == timeSlotNum) {
                curDesRegion = i;
                curSrcRegion = j;
            }
        }
    }
    assert(curDesRegion!=-1 && curSrcRegion!=-1);

    std::cout<<"allowed curDesRegion:"<<curDesRegion;
    std::cout<<std::endl;
    std::cout<<"allowed curSrcRegion:"<<curSrcRegion;
    std::cout<<std::endl;

    if (curDesRegion == myRegionNum || curSrcRegion == myRegionNum){
        // if (curSrcRegion == curDesRegion) {
        //     return 0;
        //     //means this a intra-region transmission slot
        //     //at this slot,this router can be both des and src
        // }
        // else if (curDesRegion == myRegionNum) {
        //     return 1;
        //     //at this slot,this router can be des
        // }
        // else if (curSrcRegion == myRegionNum){
        //     return 2;
        //     //at this slot,this router can be src
        // }
        return 1;
    }
    else{
        return -1;
    }
    return -1;

}

void Router::add_num_cfcpkt(){
    m_cur_num_cfcpkt++;
}


} // namespace garnet
} // namespace ruby
} // namespace gem5
