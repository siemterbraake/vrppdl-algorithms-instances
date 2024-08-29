#include "visit.hpp"

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

VisitLoc::VisitLoc(unsigned short idDepot, unsigned short idSL, unsigned short posSL, unsigned short idVehicleType, 
                   unsigned short idRoute, unsigned short idVisit) : 
                   d_idDepot(idDepot), d_idSL(idSL), d_posSL(posSL), 
                   d_idVehicleType(idVehicleType), d_idRoute(idRoute), 
                   d_idVisit(idVisit) {};

std::tuple<unsigned short,unsigned short,unsigned short> VisitLoc::getRoute() {
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
    d_idVisit = std::numeric_limits<unsigned short>::max();
    d_idNode = std::numeric_limits<unsigned short>::max();
    d_qDel = 0;
    d_qCol = 0;
    d_type = 0;
}

Visit::Visit(int timeArrival, int timeDeparture, int twWait, int twSlackStart, int twSlackEnd, 
          unsigned short idVisit, unsigned short idNode, unsigned short qDel, unsigned short qCol, short type) :
            d_timeArrival(timeArrival), d_timeDeparture(timeDeparture), d_twWait(twWait),
            d_twSlackStart(twSlackStart), d_twSlackEnd(twSlackEnd), d_idVisit(idVisit),
            d_idNode(idNode), d_qDel(qDel), d_qCol(qCol), d_type(type) {};
