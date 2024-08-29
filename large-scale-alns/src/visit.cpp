#include "visit.h"

VisitLoc::VisitLoc() {
    /*
    Empty constructor for the visit location struct.
    */
    d_idDepot = 0;
    d_idSL = 0;
    d_posSL = 0;
    d_idVehicleType = 0;
    d_idRoute = 0;
    d_idVisit = 0;
}

VisitLoc::VisitLoc(std::size_t idDepot, std::size_t idSL, std::size_t posSL, std::size_t idVehicleType, 
                   std::size_t idRoute, std::size_t idVisit) : 
                   d_idDepot(idDepot), d_idSL(idSL), d_posSL(posSL), 
                   d_idVehicleType(idVehicleType), d_idRoute(idRoute), 
                   d_idVisit(idVisit) {};

std::tuple<std::size_t,std::size_t,std::size_t> VisitLoc::getRoute() {
    /*
    Function that returns a tuple with the route information.
    */
    return std::make_tuple(d_idDepot, d_idVehicleType, d_idRoute);
}

Visit::Visit() {
    /*
    Empty constructor for the visit struct.
    */
    d_timeArrival = 0;
    d_timeDeparture = 0;
    d_twWait = 0;
    d_twSlackStart = 0;
    d_twSlackEnd = 0;
    d_idVisit = std::numeric_limits<std::size_t>::max();
    d_idNode = std::numeric_limits<std::size_t>::max();
    d_qDel = 0;
    d_qCol = 0;
    d_type = 0;
}

Visit::Visit(int timeArrival, int timeDeparture, int twWait, int twSlackStart, int twSlackEnd, 
          std::size_t idVisit, std::size_t idNode, std::size_t qDel, std::size_t qCol, short type) :
            d_timeArrival(timeArrival), d_timeDeparture(timeDeparture), d_twWait(twWait),
            d_twSlackStart(twSlackStart), d_twSlackEnd(twSlackEnd), d_idVisit(idVisit),
            d_idNode(idNode), d_qDel(qDel), d_qCol(qCol), d_type(type) {};
