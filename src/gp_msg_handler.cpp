/** @file backend.cpp
 * @brief Definition of class GPMsgHandler
 * @author Matteo Frosi
 */

#include <gp_msg_handler.h>

GeoPointStampedMSG::ConstPtr GPMsgHandler::gprmc_to_geopointstamped_msg(const NMEA::GprmcMSG::ConstPtr &gprmc_msg) {
    if(gprmc_msg->status != 'A') {
        return nullptr;
    }

    GeoPointStampedMSG::Ptr geopoint_msg = std::make_shared<GeoPointStampedMSG>();
    geopoint_msg->header = gprmc_msg->header;
    double latitude = degmin2deg(gprmc_msg->lat);
    geopoint_msg->lat = gprmc_msg->lat_dir == 'N' ? latitude : -latitude;
    double longitude = degmin2deg(gprmc_msg->lon);
    geopoint_msg->lon = gprmc_msg->lon_dir == 'E' ? longitude : -longitude;

}

GeoPointStampedMSG::ConstPtr GPMsgHandler::navsat_to_geopointstamped_msg(const NavSatFixMSG::ConstPtr& navsat_msg) {
    GeoPointStampedMSG::Ptr geopoint_msg = std::make_shared<GeoPointStampedMSG>();
    geopoint_msg->header = navsat_msg->header;
    geopoint_msg->lat = navsat_msg->lat;
    geopoint_msg->lon = navsat_msg->lon;
    geopoint_msg->alt = navsat_msg->alt;
    return geopoint_msg;
}

double GPMsgHandler::degmin2deg(double degmin) {
    double d = std::floor(degmin / 100.0);
    double m = (degmin - d * 100.0) / 60.0;
    return d + m;
}
