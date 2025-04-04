#include "hmm_madmap.h"

/*Point to segment distance, lon should multiple cos(lat) in the ball*/
double LatLon::distance(LatLon x, LatLon a, LatLon b )
{
    double tmp1, tmp2;
    tmp1=( b.lat - a.lat ) * ( x.lat - a.lat ) + ( b.lon - a.lon ) * ( x.lon - a.lon )*pow(cos(rad(x.lat)),2);
    tmp2=( b.lat - a.lat ) * ( x.lat - b.lat ) + ( b.lon - a.lon ) * ( x.lon - b.lon )*pow(cos(rad(x.lat)),2);
    if( tmp1 * tmp2 < 0 )// x can project to segment ab
    {
        double s_deg=std::abs( ( a.lon - b.lon ) * ( x.lat - a.lat ) + ( b.lat - a.lat ) * ( x.lon - a.lon) )*cos(rad(x.lat)) / hypot( (a.lon - b.lon)*cos(rad(x.lat)) , b.lat - a.lat );
        return rad(s_deg)* EARTH_RADIUS;
         
    }
    else    
    {
        double s_deg=std::min( hypot( x.lat-a.lat, (x.lon - a.lon)*cos(rad(x.lat)) ), hypot( x.lat-b.lat, (x.lon - b.lon)*cos(rad(x.lat)) ) );//return distance to nearst node

        return rad(s_deg)* EARTH_RADIUS;
    }
}

/*Point to segment projection point*/
LatLon LatLon::projection_point(LatLon x, LatLon a, LatLon b )
{
    double tmp1, tmp2;
    tmp1=( b.lat - a.lat ) * ( x.lat - a.lat ) + ( b.lon - a.lon ) * ( x.lon - a.lon )*pow(cos(rad(x.lat)),2);
    tmp2=( b.lat - a.lat ) * ( x.lat - b.lat ) + ( b.lon - a.lon ) * ( x.lon - b.lon )*pow(cos(rad(x.lat)),2);
    if( tmp1 * tmp2 < 0 )
    {
        double delta_x1, delta_x2;
        delta_x1 = ( ( b.lat - a.lat ) * ( x.lat - a.lat ) + ( b.lon - a.lon ) * ( x.lon - a.lon )*pow(cos(rad(x.lat)),2) )
                    / hypot( a.lat - b.lat, (a.lon - b.lon)*cos(rad(x.lat)) ) / hypot( a.lat - b.lat, (a.lon - b.lon)*cos(rad(x.lat)) ) * ( b.lat - a.lat );
        delta_x2 = ( ( b.lat - a.lat ) * ( x.lat - a.lat ) + ( b.lon - a.lon ) * ( x.lon - a.lon )*pow(cos(rad(x.lat)),2) )
                    / hypot( a.lat - b.lat, (a.lon - b.lon)*cos(rad(x.lat)) ) / hypot( a.lat - b.lat, (a.lon - b.lon)*cos(rad(x.lat)) ) * ( b.lon - a.lon );
        return LatLon(a.lat + delta_x1, a.lon + delta_x2) ;
        
    }
    else
    {
        if(hypot( x.lat-a.lat, (x.lon - a.lon)*cos(rad(x.lat)) ) < hypot( x.lat-b.lat,(x.lon - b.lon )*cos(rad(x.lat)) ))//close to node a
        {
            return a;
        }
        else//close to node b
        {
            return b;
        }
            
    }
}
double LatLon::rad(double d)
{
    return d * M_PI /180.0;
}
//TODO: check calBearing
double TraceNode::calBearing(double lat1,double lng1,double lat2,double lng2) 
{

    // double lat1_rad = lat1 * M_PI / 180;
    // double lon1_rad = lng1 * M_PI / 180;
    // double lat2_rad = lat2 * M_PI / 180;
    // double lon2_rad = lng2 * M_PI / 180;

    // double y = sin(lon2_rad - lon1_rad) * cos(lat2_rad);
    // double x = cos(lat1_rad) * sin(lat2_rad) - \
    //     sin(lat1_rad) * cos(lat2_rad) * cos(lon2_rad - lon1_rad);

    // double brng = atan2(y, x) ;
    // return brng;
    
    // https://wenku.baidu.com/view/a80edfa775a20029bd64783e0912a21615797f54.html?_wkts_=1712562606579&needWelcomeRecommand=1
    double s12,azi1,azi2;
    GeographicLib::Geodesic::WGS84().Inverse(lat1, lng1, lat2, lng2,s12,azi1,azi2);
    double bearing=azi1* M_PI / 180;
    while (bearing > 2*M_PI) bearing -= 2*M_PI;
    while (bearing < 0) bearing += 2*M_PI;

    return bearing;
}
double TraceNode::angleInterp(double a1, double w1, double a2, double w2)
{
    double diff = a2 - a1;
    double aa;

    // if (diff > M_PI/2) a1 += M_PI;
    // else if (diff < -M_PI/2) a1 -= M_PI;
    if (diff > M_PI) a1 += 2*M_PI;
    else if (diff < -M_PI) a1 -= 2*M_PI;

    aa = (w1 * a1 + w2 * a2) / (w1 + w2);


    std::cout<<"Interp"<<a1<<", "<<a2<<", "<<aa<<std::endl;
    return aa;
}
double TraceNode::dAngle(double ang1,double ang2)
{
    double diff = fabs(ang2 - ang1);
    double aa;

    // if (diff > M_PI/2) a1 += M_PI;
    // else if (diff < -M_PI/2) a1 -= M_PI;
    if (diff > M_PI) diff -= 2*M_PI;
    return fabs(diff);

}
double TraceNode::RealDistance(double lat1,double lng1,double lat2,double lng2)
{

	double a = rad(lat1) - rad(lat2);
    double b = rad(lng1) - rad(lng2);
   double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(rad(lat1))*cos(rad(lat2))*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    return s;
}
double TraceNode::RealDistance(LatLon p1, LatLon p2 )
{

	double a = rad(p1.lat) - rad(p2.lat);
    double b = rad(p1.lon) - rad(p2.lon);
   double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(rad(p1.lat))*cos(rad(p2.lat))*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    return s;
}
LatLon TraceNode::WGS84incre(double lat,double lng, double heading, double distance)
{
    // LatLon corrected_pos;
    // corrected_pos.lat=lat+distance*sin(heading)/LONLAT2METER;
    // corrected_pos.lon=lng-distance*cos(heading)/LONLAT2METER/cos(rad(lat));
    // return corrected_pos;
    double lat2,lon2;
    GeographicLib::Geodesic::WGS84().Direct(lat,lng,heading/M_PI*180,distance,lat2,lon2);
    return LatLon(lat2,lon2);

}
/*Calculate trace projection point*/
void TraceNode::projection(TraceNode &trace_node)
{

    // std::unordered_map<mad::MapObjectId,Road_MADMAP ,hash_MapObjectId>& candidate_roads=trace_node.candidate_roads;
    for(auto& road:trace_node.candidate_roads)
    {
        // LatLon x(trace_node.lat, trace_node.lon);
        LatLon x(trace_node.corrected_pos.lat, trace_node.corrected_pos.lon);

        double ans=1000;
        int min_index=0;
        for(int k=0;k< road.second.node.size()-1;k++)
        {
            double tmp= distance( x, road.second.node[k], road.second.node[k+1]);
            if(tmp < ans)
            {
                min_index=k;
                ans=tmp;
            }
        }
        trace_node.projection_distance[road.first]=ans;
        trace_node.trace_projection[road.first]=projection_point(x,road.second.node[min_index],road.second.node[min_index+1]);

        // double heading_1=atan2((road.second.node[min_index+1].lon-road.second.node[min_index].lon)*cos(trace_node.lat),(road.second.node[min_index+1].lat-road.second.node[min_index].lat));
        double heading=TraceNode::calBearing(road.second.node[min_index].lat,road.second.node[min_index].lon,road.second.node[min_index+1].lat,road.second.node[min_index+1].lon);
        
        if(road.second.direction==mad::LinkDirection::FROM_NODE_0 || road.second.direction==mad::LinkDirection::OPEN_BOTH_DIRECTIONS)
        {
            trace_node.road_heading[road.first]=heading;
        }
        else if(road.second.direction==mad::LinkDirection::FROM_NODE_1)
        {
            trace_node.road_heading[road.first]=heading+M_PI;
        }
    }


}
// double TraceNode::road_min_distance(double x, double y, Road_MADMAP &road)
// {
//     double ans=1000;
//     auto it = road.node.begin();
//     while(true)
//     {
//         double tmp;
//         auto it0=it;
//         it++;
//         if(it == road.node.end() ) break;
//         LatLon p(x,y);
//         tmp= TraceNode::distance( p, *it0, *it );
//         ans = std::min(ans, tmp);
//     }
//     return ans;
// }

