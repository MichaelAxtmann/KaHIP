/******************************************************************************
 * flow_graph_io.h 
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

#ifndef FLOWGRAPHIO_H_
#define FLOWGRAPHIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "definitions.h"
#include "data_structure/flow_graph.h"

class flow_graph_io {
        public:
                flow_graph_io(); 
                virtual ~flow_graph_io ();

                static
                int readDIMACS(flow_graph& G, std::string filename, NodeID & source, NodeID & sink);
};



#endif /*FLOWGRAPHIO_H_*/
