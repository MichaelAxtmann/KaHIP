/******************************************************************************
 * flow_graph_io.cpp 
 *
 * Source of Max Flow Framework 
 *
 ******************************************************************************
 * Copyright (C) 2013 Christian Schulz <christian.schulz@kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#include <fstream>
#include <iostream>
#include <limits>
#include <stdio.h>
#include <sstream>

#include "flow_graph_io.h"

flow_graph_io::flow_graph_io() {

}

flow_graph_io::~flow_graph_io() {

}

int flow_graph_io::readDIMACS(flow_graph & G, std::string filename, NodeID & source, NodeID & sink) {
        //TODO: this is currently only rudimentary implemented, 
        //TODO: error handling and outputing nice information to the user
        std::string line;

        // open file for reading
        std::ifstream in(filename.c_str());
        if (!in) {
                std::cerr << "Error opening " << filename << std::endl;
                return 1;
        }

        NodeID nmbNodes;
        NodeID nmbEdges;
        std::getline(in,line);
        char first_char;
        while( !in.eof() ) {
                std::stringstream ss(line);
                ss >> first_char;

                switch( first_char ) {
                        case 'c':
                                break;
                        case '\n':
                                break;
                        case '\0':
                                break;
                        case 'p': 
                                {
                                        std::string type;
                                        ss >> type;
                                        ss >> nmbNodes;
                                        ss >> nmbEdges;
                                        G.start_construction(nmbNodes, 2*nmbEdges);
                                }

                                break;
                        case 'n': 
                                {
                                        NodeID node; char ntype;
                                        ss >> node;
                                        ss >> ntype;
                                        switch( ntype ) {
                                                case 's':
                                                        source = node-1;
                                                        break;
                                                case 't':
                                                        sink = node-1;
                                                        break;
                                        }
                                }
                                break;
                        case 'a': 
                                {
                                        NodeID source;
                                        NodeID target;
                                        FlowType cap;
                                        ss >> source;
                                        ss >> target;
                                        ss >> cap;
                                        G.new_edge(source-1, target-1, cap);
                                }
                                break;

                }

                std::getline(in, line);
        }
        G.finish_construction();

        return 0;
}




