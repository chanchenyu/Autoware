#ifndef __UTM_POS_CONV__
#define __UTM_POS_CONV__

#include <math.h>

class utm_pos_conv {
private:
	double m_x;  //m
	double m_y;  //m
	double m_z;  //m

	double m_lat;  //latitude
	double m_lon; //longitude
	double m_h;	
  
  bool m_hemi;  // 0 for northern hemisphere, 1 for southern hemisphere.
  
  int m_zone;        // UTM 6 degree longitudinal zone (1..60 covering 180W..180E).
  
  double m_conver; // Meridian convergence (bearing of grid north clockwise from true north), in degrees
  double m_scale;  //Grid scale factor 

public:
	double x() const;
	double y() const;
	double z() const;
  
  double lat() const;
	double lon() const;
	double h() const;	
  
  bool hemi() const;
  int zone() const;
  double conver() const;
  double scale() const;
  
	void set_xyz(double cx,   double cy,   double cz);

	//set llh in radians
	void set_llh(double lat, double lon, double h);

	//set llh in nmea degrees
	void set_llh_nmea_degrees(double latd,double lond, double h);  

	void conv_llh2xyz(void);
	void conv_xyz2llh(void);
};

#endif
