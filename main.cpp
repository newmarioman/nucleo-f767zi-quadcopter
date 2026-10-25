#include "flight_app.h"

int main()
{
    static const quadcopter::BoardConfig board;
    static const quadcopter::FlightConfig config;
    static quadcopter::FlightApp application(board, config);
    application.run();
}
