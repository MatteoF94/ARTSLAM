/** @file gp_msg_handler.h
 * @brief Declaration of class GPMsgHandler
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_GP_MSG_HANDLER_H
#define ARTSLAM_GP_MSG_HANDLER_H


#include <slam_types.h>

class GPMsgHandler {
public:
    static GeoPointStampedMSG::ConstPtr gprmc_to_geopointstamped_msg(const NMEA::GprmcMSG::ConstPtr& gprmc_msg);
    static GeoPointStampedMSG::ConstPtr navsat_to_geopointstamped_msg(const NavSatFixMSG::ConstPtr& navsat_msg);

private:
    static double degmin2deg(double degmin);
};


#endif //ARTSLAM_GP_MSG_HANDLER_H
