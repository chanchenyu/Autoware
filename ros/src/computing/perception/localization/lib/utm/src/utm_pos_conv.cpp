#include <utm_pos_conv.hh>

using std::string;
using std::cout;
using std::endl;
using std::cerr;

double utm_pos_conv::x() const
{
  return m_x;
}

double utm_pos_conv::y() const
{
  return m_y;
}

double utm_pos_conv::z() const
{
  return m_z;
}

double utm_pos_conv::lat() const
{
  return m_lat;
}

double utm_pos_conv::lon() const
{
  return m_lon;
}

double utm_pos_conv::h() const
{
  return m_h;
}

bool utm_pos_conv::hemi() const
{
  return m_hemi;
}

int utm_pos_conv::zone() const
{
  return m_zone;
}

double utm_pos_conv::conver() const
{
  return m_conver;
}

double utm_pos_conv::scale() const
{
  return m_scale;
}

void utm_pos_conv::set_xyz(double cx, double cy, double cz)
{
  m_x = cx;
  m_y = cy;
  m_z = cz;
  conv_xyz2llh();
}

void utm_pos_conv::set_llh_nmea_degrees(double latd, double lond, double h)
{
  double lat, lad, lod, lon;
  // 1234.56 -> 12'34.56 -> 12+ 34.56/60

  lad = floor(latd / 100.);
  lat = latd - lad * 100.;
  lod = floor(lond / 100.);
  lon = lond - lod * 100.;
  
  m_lat = lad + lat / 60.0;
  m_lon = lod + lon / 60.0;
  m_h = h;

  conv_llh2xyz();
}

void utm_pos_conv::set_llh(double lat, double lon, double h)
{
  m_lat = lat;
  m_lon = lon;
  m_h = h;

  conv_llh2xyz();
}

/*
 * Implements Karney’s method, using Krüger series to order n^6, giving results accurate to 5nm for
 * distances up to 3900km from the central meridian.
 *
 */
void utm_pos_conv::conv_llh2xyz(void)
{
  if (m_lat >= -80 && m_lat <= 84) {
    cerr << "Outside UTM limits" << endl;
    exit(1);
  }
  double falseEasting = 500e3, falseNorthing = 10000e3;
  int zone = floor((m_lon+180)/6)+1; // longitudinal zone
  double lambda_0 = ((zone-1)*6 - 180 + 3) * M_PI / 180;
  
  // ---- handle Norway/Svalbard exceptions
  // grid zones are 8 degree tall; 0 degree N is offset 10 into latitude bands array
  string mgrsLatBands = "CDEFGHJKLMNPQRSTUVWXX"; // X is repeated for 80-84 degree N
  char latBand = mgrsLatBands[floor(m_lat/8+10)];
  // adjust zone & central meridian for Norway
  if (zone==31 && latBand=='V' && m_lon>= 3) { zone++; lambda_0 += (6)* M_PI / 180; }
  // adjust zone & central meridian for Svalbard
  if (zone==32 && latBand=='X' && m_lon<  9) { zone--; lambda_0 -= (6)* M_PI / 180; }
  if (zone==32 && latBand=='X' && m_lon>= 9) { zone++; lambda_0 += (6)* M_PI / 180; }
  if (zone==34 && latBand=='X' && m_lon< 21) { zone--; lambda_0 -= (6)* M_PI / 180; }
  if (zone==34 && latBand=='X' && m_lon>=21) { zone++; lambda_0 += (6)* M_PI / 180; }
  if (zone==36 && latBand=='X' && m_lon< 33) { zone--; lambda_0 -= (6)* M_PI / 180; }
  if (zone==36 && latBand=='X' && m_lon>=33) { zone++; lambda_0 += (6)* M_PI / 180; }
  
  
  double phi = m_lat * M_PI / 180;; // latitude +- from equator
  double lambda = m_lon * M_PI / 180 - lambda_0; // longitude +- from central meridian
  
  // WGS 84: a = 6378137, b = 6356752.314245, f = 1/298.257223563;
  double a = 6378137, f = 1/298.257223563;
  
  double k0 = 0.9996; // UTM scale on the central meridian
  
  // ---- easting, northing: Karney 2011 Eq 7-14, 29, 35:  
  double e = sqrt(f*(2-f));      // eccentricity
  double n = f / (2 - f);        // 3rd flattening
  double n2 = n*n, n3 = n*n2, n4 = n*n3, n5 = n*n4, n6 = n*n5; //Horner-form
  
  double cos_lambda = cos(lambda), sin_lambda = sin(lambda), tan_lambda = tan(lambda);
  double tau = tan(phi); // τ ≡ tanφ, τʹ ≡ tanφʹ; prime (ʹ) indicates angles on the conformal sphere
  double sigma = sinh(e*atanh(e*tau/sqrt(1+tau*tau)));
  
  double tau_prime = tau*sqrt(1+sigma*sigma) - sigma*sqrt(1+tau*tau);
  double psi_prime = atan2(tau_prime, cos_lambda);
  double ita_prime = asinh(sin_lambda / sqrt(tau_prime*tau_prime + cos_lambda*cos_lambda));
  
  double A = a/(1+n) * (1 + 1/4*n2 + 1/64*n4 + 1/256*n6); // 2 * pi * A is the circumference of a meridian
  
  double alpha = [ null, // note alpha is one-based array (6th order Kruger expressions)
                   1/2*n - 2/3*n2 + 5/16*n3 +   41/180*n4 -     127/288*n5 +      7891/37800*n6,
                         13/48*n2 -  3/5*n3 + 557/1440*n4 +     281/630*n5 - 1983433/1935360*n6,
                                  61/240*n3 -  103/140*n4 + 15061/26880*n5 +   167603/181440*n6,
                                          49561/161280*n4 -     179/168*n5 + 6601661/7257600*n6,
                                                            34729/80640*n5 - 3418889/1995840*n6,
                                                                         212378941/319334400*n6 ];
  double psi = psi_prime;
  
  for (size_t j=1; j<=6; j++) psi += alpha[j] * sin(2*j*psi_prime) * cosh(2*j*ita_prime);

  double ita = ita_prime;
  for (size_t j=1; j<=6; j++) ita += alpha[j] * cos(2*j*psi_prime) * sinh(2*j*ita_prime);
  
  
  m_x = k0 * A * ita;
  m_y = k0 * A * psi;
  m_z = m_h;

  // ---- convergence: Karney 2011 Eq 23, 24

  double p_prime = 1;
  for (size_t j=1; j<=6; j++) p_prime += 2*j*alpha[j] * cos(2*j*psi_prime) * cosh(2*j*ita_prime);
  double q_prime = 0;
  for (size_t j=1; j<=6; j++) q_prime += 2*j*alpha[j] * sin(2*j*psi_prime) * sinh(2*j*ita_prime);

  double gamma_prime = atan(tau_prime / sqrt(1+tau_prime*tau_prime)*tan_lambda);
  double gamma_d_prime = atan2(q_prime, p_prime);

  double gamma = gamma_prime + gamma_d_prime;

  // ---- scale: Karney 2011 Eq 25

  double sin_phi = sin(phi);
  double k_prime = sqrt(1 - e*e*sin_phi*sin_phi) * sqrt(1 + tau*tau) / sqrt(tau_prime*tau_prime + cos_lambda*cos_lambda);
  double k_d_prime = A / a * sqrt(p_prime*p_prime + q_prime*q_prime);

  double k = k0 * k_prime * k_d_prime;

  // ------------

  // shift x/y to false origins
  m_x = m_x + falseEasting;               // make x relative to false easting
  if (m_y < 0) m_y = m_y + falseNorthing; // make y in southern hemisphere relative to false northing


  m_conver = gamma * 180 / M_PI;
  m_scale = k;

  m_hemi = m_lat>=0 ? 0 : 1; // hemisphere  
}


void utm_pos_conv::conv_xyz2llh(void)
{
  // n/a
}

