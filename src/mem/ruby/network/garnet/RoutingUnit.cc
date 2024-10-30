/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
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


#include "mem/ruby/network/garnet/RoutingUnit.hh"

#include "base/cast.hh"
#include "base/compiler.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/InputUnit.hh"
#include "mem/ruby/network/garnet/Router.hh"
#include "mem/ruby/slicc_interface/Message.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

RoutingUnit::RoutingUnit(Router *router)
{
    m_router = router;
    m_routing_table.clear();
    m_weight_table.clear();
}

void
RoutingUnit::addRoute(std::vector<NetDest>& routing_table_entry)
{
    if (routing_table_entry.size() > m_routing_table.size()) {
        m_routing_table.resize(routing_table_entry.size());
    }
    for (int v = 0; v < routing_table_entry.size(); v++) {
        m_routing_table[v].push_back(routing_table_entry[v]);
    }
}

void
RoutingUnit::addWeight(int link_weight)
{
    m_weight_table.push_back(link_weight);
}

bool
RoutingUnit::supportsVnet(int vnet, std::vector<int> sVnets)
{
    // If all vnets are supported, return true
    if (sVnets.size() == 0) {
        return true;
    }

    // Find the vnet in the vector, return true
    if (std::find(sVnets.begin(), sVnets.end(), vnet) != sVnets.end()) {
        return true;
    }

    // Not supported vnet
    return false;
}

/*
 * This is the default routing algorithm in garnet.
 * The routing table is populated during topology creation.
 * Routes can be biased via weight assignments in the topology file.
 * Correct weight assignments are critical to provide deadlock avoidance.
 */
int
RoutingUnit::lookupRoutingTable(int vnet, NetDest msg_destination)
{
    // First find all possible output link candidates
    // For ordered vnet, just choose the first
    // (to make sure different packets don't choose different routes)
    // For unordered vnet, randomly choose any of the links
    // To have a strict ordering between links, they should be given
    // different weights in the topology file

    int output_link = -1;
    int min_weight = INFINITE_;
    std::vector<int> output_link_candidates;
    int num_candidates = 0;

    // Identify the minimum weight among the candidate output links
    for (int link = 0; link < m_routing_table[vnet].size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(
            m_routing_table[vnet][link])) {

        if (m_weight_table[link] <= min_weight)
            min_weight = m_weight_table[link];
        }
    }

    // Collect all candidate output links with this minimum weight
    for (int link = 0; link < m_routing_table[vnet].size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(
            m_routing_table[vnet][link])) {

            if (m_weight_table[link] == min_weight) {
                num_candidates++;
                output_link_candidates.push_back(link);
            }
        }
    }

    if (output_link_candidates.size() == 0) {
        fatal("Fatal Error:: No Route exists from this Router.");
        exit(0);
    }

    // Randomly select any candidate output link
    int candidate = 0;
    if (!(m_router->get_net_ptr())->isVNetOrdered(vnet))
        candidate = rand() % num_candidates;

    output_link = output_link_candidates.at(candidate);
    return output_link;
}


void
RoutingUnit::addInDirection(PortDirection inport_dirn, int inport_idx)
{
    m_inports_dirn2idx[inport_dirn] = inport_idx;
    m_inports_idx2dirn[inport_idx]  = inport_dirn;
}

void
RoutingUnit::addOutDirection(PortDirection outport_dirn, int outport_idx)
{
    m_outports_dirn2idx[outport_dirn] = outport_idx;
    m_outports_idx2dirn[outport_idx]  = outport_dirn;
}

// outportCompute() is called by the InputUnit
// It calls the routing table by default.
// A template for adaptive topology-specific routing algorithm
// implementations using port directions rather than a static routing
// table is provided here.

int
RoutingUnit::outportCompute(RouteInfo route, int inport,
                            PortDirection inport_dirn)
{
    int outport = -1;

    if (route.dest_router == m_router->get_id()) {

        // Multiple NIs may be connected to this router,
        // all with output port direction = "Local"
        // Get exact outport id from table
        outport = lookupRoutingTable(route.vnet, route.net_dest);
        return outport;
    }

    // Routing Algorithm set in GarnetNetwork.py
    // Can be over-ridden from command line using --routing-algorithm = 1
    RoutingAlgorithm routing_algorithm =
        (RoutingAlgorithm) m_router->get_net_ptr()->getRoutingAlgorithm();

    switch (routing_algorithm) {
        case TABLE_:  outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
        case XY_:     outport =
            outportComputeXY(route, inport, inport_dirn); break;
        // any custom algorithm
        case CUSTOM_: outport =
            outportComputeCustom(route, inport, inport_dirn); break;
        default: outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
    }

    assert(outport != -1);
    return outport;
}

// XY routing implemented using port directions
// Only for reference purpose in a Mesh
// By default Garnet uses the routing table
int
RoutingUnit::outportComputeXY(RouteInfo route,
                              int inport,
                              PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops == 0)
    {
        if (y_dirn > 0)
            outport_dirn = "North";
        else
            outport_dirn = "South";
    }
    else if (y_hops == 0)
    {
        if (x_dirn > 0)
            outport_dirn = "East";
        else
            outport_dirn = "West";
    }
    else
    {
        // whichever router has more free VCs route there
        int rand = random() % 2;
        // int rand = 1;
        Router *router_Est, *router_Wst, *router_South, *router_Nrth;
        if (x_dirn && y_dirn) {// Quadrant I
            // check for routers in both 'East' and 'North'
            // direction
            router_Est = m_router->get_net_ptr()->\
                get_upstreamrouter( "East", m_router->get_id());
            router_Nrth = m_router->get_net_ptr()->\
                get_upstreamrouter( "North", m_router->get_id());
            // caution: 'dirn_' is the direction of inport of
            // downstream router
            int freeVC_East = router_Est->get_numFreeVC("West");
            int freeVC_North = router_Nrth->get_numFreeVC("South");

            if (freeVC_East > freeVC_North)
                outport_dirn = "East";
            else if (freeVC_North > freeVC_East)
                outport_dirn = "North";
            else
                outport_dirn = rand ? "East" : "North";

        }
        else if (!x_dirn && y_dirn) {// Quadrant II

            router_Wst = m_router->get_net_ptr()->\
                get_upstreamrouter( "West", m_router->get_id());
            router_Nrth = m_router->get_net_ptr()->\
                get_upstreamrouter( "North", m_router->get_id());

            int freeVC_West = router_Wst->get_numFreeVC("East");
            int freeVC_North = router_Nrth->get_numFreeVC("South");

            if (freeVC_North > freeVC_West)
                outport_dirn = "North";
            else if (freeVC_West > freeVC_North)
                outport_dirn = "West";
            else
                outport_dirn = rand ? "West" : "North";

        }
        else if (!x_dirn && !y_dirn) {// Quadrant III

            router_Wst = m_router->get_net_ptr()->\
                get_upstreamrouter( "West", m_router->get_id());
            router_South = m_router->get_net_ptr()->\
                get_upstreamrouter( "South", m_router->get_id());

            int freeVC_West = router_Wst->get_numFreeVC("East");
            int freeVC_South = router_South->get_numFreeVC("North");

            if (freeVC_South > freeVC_West)
                outport_dirn = "South";
            else if (freeVC_West > freeVC_South)
                outport_dirn = "West";
            else
                outport_dirn = rand ? "West" : "South";
        }
        else {// Quadrant IV

            router_Est = m_router->get_net_ptr()->\
                get_upstreamrouter( "East", m_router->get_id());
            router_South = m_router->get_net_ptr()->\
                get_upstreamrouter( "South", m_router->get_id());

            int freeVC_East = router_Est->get_numFreeVC("West");
            int freeVC_South = router_South->get_numFreeVC("North");

            if (freeVC_South > freeVC_East)
                outport_dirn = "South";
            else if (freeVC_East > freeVC_South)
                outport_dirn = "East";
            else
                outport_dirn = rand ? "East" : "South";
        }
    }

    return m_outports_dirn2idx[outport_dirn];
}


//A new XY routing for 4x4 mesh chiplets
//and 4x4 interposer id is fixed for simple
int
RoutingUnit::outportComputeCustom(RouteInfo route,
                              int inport,
                              PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    // std::cout << std::endl;
    // std::cout << "start calling routingunit" << std::endl;
    // std::cout << "at router id " << m_router->get_id() << std::endl;
    // std::cout << "the inport dirn is "<< inport_dirn << std::endl;
    // std::cout << "dest router id is "<< route.dest_router << std::endl;

    //[[maybe_unused]] int num_rows = m_router->get_net_ptr()->getNumRows();
    // int num_cols = m_router->get_net_ptr()->getNumCols();
    int num_cols = m_router->get_net_ptr()->getNumRows();
    int num_rows = m_router->get_net_ptr()->getNumRows();
    //std::cout << "COLS & ROWS" << num_cols << " "<< num_rows << std::endl;
    assert(num_rows > 0 && num_cols > 0);

    //* assume the num of chiplet router is C x N x N
    //* assume the num of interposer router is I x I
    //* assume the num of L2, MC, DMA router is K
    //* the ID order is C x N x N + K + I x I
    //* which means the chiplet routers are first (C x N x N)
    //* and the interposer router are last (I x I)

    //* at this time : C = N = I = 4, K = 0
    //* so chiplet router (0 ~ 63)
    //* interposer router (64 ~ 79)

    int num_chiplets = 4;
    int total_num_router = m_router->get_net_ptr()->getNumRouters();
    int my_id = m_router->get_id();
    int my_chiplet_id = my_id / (num_cols*num_rows);

    int dest_id = route.dest_router;
    int dest_chiplet_id = dest_id / (num_cols*num_rows);


    // CASE1: interposer router to (interposer/chiplet/other) router
    // check if this is interposer router (72 ~ 87)
    if (my_id >= (total_num_router - (num_cols * num_rows))){
        // std::cout << "enter CASE1: interposer router to
        // (interposer/chiplet/other) router " << std::endl;
        // the dst router is also interposer router
        // deprecated impossible
        // if (dest_id >= (total_num_router - (num_cols*num_rows))){
        //     //transfer to onchip id
        //     my_id = my_id - (total_num_router - (num_cols*num_rows));
        //     dest_id = dest_id - (total_num_router - (num_cols*num_rows));
        // }

        // the dst router is chiplet router
        if (dest_id < (num_chiplets * num_cols * num_rows)){

            //find the dest's nearest br's interposer router
            int dest_onchip_id = dest_id % (num_cols*num_rows);
            int dest_onchip_x = dest_onchip_id % num_cols;
            int dest_onchip_y = dest_onchip_id / num_cols;

            int halfCol = num_cols / 2;
            int halfRow = num_rows / 2;

            int offset = 0 ;

            if (dest_onchip_x < halfCol && dest_onchip_y < halfRow){
                offset = 0;}
            else if (dest_onchip_x >= halfCol && dest_onchip_y < halfRow){
                offset = 1;}
            else if (dest_onchip_x < halfCol && dest_onchip_y >= halfRow){
                offset = 4;}
            else {
                offset = 5;}

            // chosed interposer to send "Up"
            dest_id = offset + (num_chiplets * num_cols * num_rows) + \
            (dest_chiplet_id % halfCol) * 2 + \
            (dest_chiplet_id / halfRow) * 8;
            // << " chosed interposer to send Up: " << dest_id
            // << std::endl;

            if (my_id == dest_id){
                //std::cout << "return outport dirn: Up" << std::endl;
                outport_dirn = "Up";
                return m_outports_dirn2idx[outport_dirn];
            }
            else{
                //transfer to onchip id
                my_id = my_id - (total_num_router - (num_cols*num_rows));
                dest_id = dest_id - (total_num_router - (num_cols*num_rows));
            }

        }
        // the dst router is other router(L2,MC..)
        else{
            // assume the map is 16 offset
            // for example L2 router 71 connect to interposer 87
            // this needs to be sured at Chiplets_Mesh.py
            my_id = my_id - (total_num_router - (num_cols*num_rows));
            dest_id = dest_id - (total_num_router - (num_cols*num_rows)) + 16;

            if (my_id == dest_id){
                //std::cout << "return outport dirn: memUp" << std::endl;
                outport_dirn = "memUp";
                return m_outports_dirn2idx[outport_dirn];
            }
        }
    }
    // CASE2: chiplet router to (chiplet/other) router
    // check if this is chiplet router
    else if (my_id < (num_chiplets * num_cols * num_rows)){
        //std::cout <<
        // "enter CASE2: chiplet router to (chiplet/other) router "
        // << std::endl;
        // this is chiplet router and dest is another chiplet router
        if (dest_id != my_id){
            // this router and dst router are in different chiplet (L2/MC)
            // change the dst to nearest boundary router
            if (my_chiplet_id != dest_chiplet_id){
                //calculate on chip xy
                int my_onchip_id = my_id % (num_cols*num_rows);
                int my_onchip_x = my_onchip_id % num_cols;
                int my_onchip_y = my_onchip_id / num_cols;
                int nearset_onchip_br_id = 0 ;

                //find the nearest br
                int halfCol = num_cols / 2;
                int halfRow = num_rows / 2;


                if (my_onchip_x < halfCol && my_onchip_y < halfRow)
                    {nearset_onchip_br_id = 1;}
                else if (my_onchip_x >= halfCol && my_onchip_y < halfRow)
                    {nearset_onchip_br_id = 2;}
                else if (my_onchip_x < halfCol && my_onchip_y >= halfRow)
                    {nearset_onchip_br_id = 13;}
                else {nearset_onchip_br_id = 14;}
                assert(nearset_onchip_br_id != 0); //find successfully
                //std::cout << nearset_onchip_br_id << std::endl;
                //check if the nearest br is myself,
                //then just send to interposer
                if (nearset_onchip_br_id == my_onchip_id){
                    //std::cout << "return outport dirn: Down" << std::endl;
                    outport_dirn = "Down";
                    return m_outports_dirn2idx[outport_dirn];
                    }
                //transfer the dest_id to nearset_onchip_br_id
                else{
                    dest_id = nearset_onchip_br_id;
                }

                //transfer my_id to my_onchip_id
                my_id = my_onchip_id;
            }
            // dest and src are in same chiplet
            // just transfer to onchip id
            else{
                my_id = my_id - (my_chiplet_id * num_cols*num_rows);
                dest_id = dest_id - (dest_chiplet_id * num_cols * num_rows);
            }
        }
    }

    //CASE3: other router to (chiplet router)
    else{
        //std::cout << "enter CASE3: other router to
        //    (chiplet router) " << std::endl;
        //std::cout << "return outport dirn: memDown" << std::endl;
        outport_dirn = "memDown";
        return m_outports_dirn2idx[outport_dirn];
    }


    //std::cout << "the onchip src id is: "<< my_id <<std::endl;
    //std::cout << "the onchip des id is: "<< dest_id <<std::endl;
    // this is a XY routing for intra-chiplet
    // which means src and dst are in same chiplet or interposer
    // the input of this routing is onchip xy coordination

    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops > 0) {
        if (x_dirn) {
            assert(inport_dirn == "Local" || inport_dirn == "Up" \
            || inport_dirn == "memUp" || inport_dirn == "Down" \
            || inport_dirn == "memDown" || inport_dirn == "West");
            outport_dirn = "East";
        } else {
            assert(inport_dirn == "Local" || inport_dirn == "Up" \
            || inport_dirn == "memUp" || inport_dirn == "Down" \
            || inport_dirn == "memDown" || inport_dirn == "East");
            outport_dirn = "West";
        }
    } else if (y_hops > 0) {
        if (y_dirn) {
            // "Local" or "South" or "West" or "East"
            assert(inport_dirn != "North");
            outport_dirn = "North";
        } else {
            // "Local" or "North" or "West" or "East"
            assert(inport_dirn != "South");
            outport_dirn = "South";
        }
    } else {
        // x_hops == 0 and y_hops == 0
        // this is not possible
        // already checked that in outportCompute() function
        panic("x_hops == y_hops == 0");
    }
    //std::cout << "return outport dirn:" << outport_dirn << std::endl;
    return m_outports_dirn2idx[outport_dirn];
}


} // namespace garnet
} // namespace ruby
} // namespace gem5
