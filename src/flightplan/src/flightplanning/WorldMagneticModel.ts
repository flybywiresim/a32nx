/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Working Title, FlyByWire Simulations
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

export function WorldMagneticModel() {

    this.coff = [
      "  1,  0,  -29404.5  ,     0.0    ,    6.7  ,      0.0",
      "  1,  1,   -1450.7  ,  4652.9    ,    7.7  ,    -25.1",
      "  2,  0,   -2500.0  ,     0.0    ,  -11.5  ,      0.0",
      "  2,  1,    2982.0  , -2991.6    ,   -7.1  ,    -30.2",
      "  2,  2,    1676.8  ,  -734.8    ,   -2.2  ,    -23.9",
      "  3,  0,    1363.9  ,     0.0    ,    2.8  ,      0.0",
      "  3,  1,   -2381.0  ,   -82.2    ,   -6.2  ,      5.7",
      "  3,  2,    1236.2  ,   241.8    ,    3.4  ,     -1.0",
      "  3,  3,     525.7  ,  -542.9    ,  -12.2  ,      1.1",
      "  4,  0,     903.1  ,     0.0    ,   -1.1  ,      0.0",
      "  4,  1,     809.4  ,   282.0    ,   -1.6  ,      0.2",
      "  4,  2,      86.2  ,  -158.4    ,   -6.0  ,      6.9",
      "  4,  3,    -309.4  ,   199.8    ,    5.4  ,      3.7",
      "  4,  4,      47.9  ,  -350.1    ,   -5.5  ,     -5.6",
      "  5,  0,    -234.4  ,     0.0    ,   -0.3  ,      0.0",
      "  5,  1,     363.1  ,    47.7    ,    0.6  ,      0.1",
      "  5,  2,     187.8  ,   208.4    ,   -0.7  ,      2.5",
      "  5,  3,    -140.7  ,  -121.3    ,    0.1  ,     -0.9",
      "  5,  4,    -151.2  ,    32.2    ,    1.2  ,      3.0",
      "  5,  5,      13.7  ,    99.1    ,    1.0  ,      0.5",
      "  6,  0,      65.9  ,     0.0    ,   -0.6  ,      0.0",
      "  6,  1,      65.6  ,   -19.1    ,   -0.4  ,      0.1",
      "  6,  2,      73.0  ,    25.0    ,    0.5  ,     -1.8",
      "  6,  3,    -121.5  ,    52.7    ,    1.4  ,     -1.4",
      "  6,  4,     -36.2  ,   -64.4    ,   -1.4  ,      0.9",
      "  6,  5,      13.5  ,     9.0    ,   -0.0  ,      0.1",
      "  6,  6,     -64.7  ,    68.1    ,    0.8  ,      1.0",
      "  7,  0,      80.6  ,     0.0    ,   -0.1  ,      0.0",
      "  7,  1,     -76.8  ,   -51.4    ,   -0.3  ,      0.5",
      "  7,  2,      -8.3  ,   -16.8    ,   -0.1  ,      0.6",
      "  7,  3,      56.5  ,     2.3    ,    0.7  ,     -0.7",
      "  7,  4,      15.8  ,    23.5    ,    0.2  ,     -0.2",
      "  7,  5,       6.4  ,    -2.2    ,   -0.5  ,     -1.2",
      "  7,  6,      -7.2  ,   -27.2    ,   -0.8  ,      0.2",
      "  7,  7,       9.8  ,    -1.9    ,    1.0  ,      0.3",
      "  8,  0,      23.6  ,     0.0    ,   -0.1  ,      0.0",
      "  8,  1,       9.8  ,     8.4    ,    0.1  ,     -0.3",
      "  8,  2,     -17.5  ,   -15.3    ,   -0.1  ,      0.7",
      "  8,  3,      -0.4  ,    12.8    ,    0.5  ,     -0.2",
      "  8,  4,     -21.1  ,   -11.8    ,   -0.1  ,      0.5",
      "  8,  5,      15.3  ,    14.9    ,    0.4  ,     -0.3",
      "  8,  6,      13.7  ,     3.6    ,    0.5  ,     -0.5",
      "  8,  7,     -16.5  ,    -6.9    ,    0.0  ,      0.4",
      "  8,  8,      -0.3  ,     2.8    ,    0.4  ,      0.1",
      "  9,  0,       5.0  ,     0.0    ,   -0.1  ,      0.0",
      "  9,  1,       8.2  ,   -23.3    ,   -0.2  ,     -0.3",
      "  9,  2,       2.9  ,    11.1    ,   -0.0  ,      0.2",
      "  9,  3,      -1.4  ,     9.8    ,    0.4  ,     -0.4",
      "  9,  4,      -1.1  ,    -5.1    ,   -0.3  ,      0.4",
      "  9,  5,     -13.3  ,    -6.2    ,   -0.0  ,      0.1",
      "  9,  6,       1.1  ,     7.8    ,    0.3  ,     -0.0",
      "  9,  7,       8.9  ,     0.4    ,   -0.0  ,     -0.2",
      "  9,  8,      -9.3  ,    -1.5    ,   -0.0  ,      0.5",
      "  9,  9,     -11.9  ,     9.7    ,   -0.4  ,      0.2",
      " 10,  0,      -1.9  ,     0.0    ,    0.0  ,      0.0",
      " 10,  1,      -6.2  ,     3.4    ,   -0.0  ,     -0.0",
      " 10,  2,      -0.1  ,    -0.2    ,   -0.0  ,      0.1",
      " 10,  3,       1.7  ,     3.5    ,    0.2  ,     -0.3",
      " 10,  4,      -0.9  ,     4.8    ,   -0.1  ,      0.1",
      " 10,  5,       0.6  ,    -8.6    ,   -0.2  ,     -0.2",
      " 10,  6,      -0.9  ,    -0.1    ,   -0.0  ,      0.1",
      " 10,  7,       1.9  ,    -4.2    ,   -0.1  ,     -0.0",
      " 10,  8,       1.4  ,    -3.4    ,   -0.2  ,     -0.1",
      " 10,  9,      -2.4  ,    -0.1    ,   -0.1  ,      0.2",
      " 10, 10,      -3.9  ,    -8.8    ,   -0.0  ,     -0.0",
      " 11,  0,       3.0  ,     0.0    ,   -0.0  ,      0.0",
      " 11,  1,      -1.4  ,    -0.0    ,   -0.1  ,     -0.0",
      " 11,  2,      -2.5  ,     2.6    ,   -0.0  ,      0.1",
      " 11,  3,       2.4  ,    -0.5    ,    0.0  ,      0.0",
      " 11,  4,      -0.9  ,    -0.4    ,   -0.0  ,      0.2",
      " 11,  5,       0.3  ,     0.6    ,   -0.1  ,     -0.0",
      " 11,  6,      -0.7  ,    -0.2    ,    0.0  ,      0.0",
      " 11,  7,      -0.1  ,    -1.7    ,   -0.0  ,      0.1",
      " 11,  8,       1.4  ,    -1.6    ,   -0.1  ,     -0.0",
      " 11,  9,      -0.6  ,    -3.0    ,   -0.1  ,     -0.1",
      " 11, 10,       0.2  ,    -2.0    ,   -0.1  ,      0.0",
      " 11, 11,       3.1  ,    -2.6    ,   -0.1  ,     -0.0",
      " 12,  0,      -2.0  ,     0.0    ,    0.0  ,      0.0",
      " 12,  1,      -0.1  ,    -1.2    ,   -0.0  ,     -0.0",
      " 12,  2,       0.5  ,     0.5    ,   -0.0  ,      0.0",
      " 12,  3,       1.3  ,     1.3    ,    0.0  ,     -0.1",
      " 12,  4,      -1.2  ,    -1.8    ,   -0.0  ,      0.1",
      " 12,  5,       0.7  ,     0.1    ,   -0.0  ,     -0.0",
      " 12,  6,       0.3  ,     0.7    ,    0.0  ,      0.0",
      " 12,  7,       0.5  ,    -0.1    ,   -0.0  ,     -0.0",
      " 12,  8,      -0.2  ,     0.6    ,    0.0  ,      0.1",
      " 12,  9,      -0.5  ,     0.2    ,   -0.0  ,     -0.0",
      " 12, 10,       0.1  ,    -0.9    ,   -0.0  ,     -0.0",
      " 12, 11,      -1.1  ,    -0.0    ,   -0.0  ,      0.0",
      " 12, 12,      -0.3  ,     0.5    ,   -0.1  ,     -0.1",
    ];

    /* static variables */

    /* some 13x13 2D arrays */
    this.c = new Array(13);
    this.cd = new Array(13);
    this.tc = new Array(13);
    this.dp = new Array(13);
    this.k = new Array(13);

    for(var i: any = 0; i< 13; i++)
    {
        this.c[i] = new Array(13);
        this.cd[i] = new Array(13);
        this.tc[i] = new Array(13);
        this.dp[i] = new Array(13);
        this.k[i] = new Array(13);
    }

    /* some 1D arrays */
    this.snorm = new Array(169);
    this.sp = new Array(13);
    this.cp = new Array(13);
    this.fn = new Array(13);
    this.fm = new Array(13);
    this.pp = new Array(13);


    /* locals */

    var maxdeg = 12;
    var maxord;
    var i,j,D1,D2,n,m;
    var a,b,a2,b2,c2,a4,b4,c4,re;
    var gnm,hnm,dgnm,dhnm,flnmj;
    var c_str;
    var c_flds;

    /* INITIALIZE CONSTANTS */

    maxord = maxdeg;
    this.sp[0] = 0.0;
    this.cp[0] = this.snorm[0] = this.pp[0] = 1.0;
    this.dp[0][0] = 0.0;
    a = 6378.137;
    b = 6356.7523142;
    re = 6371.2;
    a2 = a*a;
    b2 = b*b;
    c2 = a2-b2;
    a4 = a2*a2;
    b4 = b2*b2;
    c4 = a4 - b4;

    /* READ WORLD MAGNETIC MODEL SPHERICAL HARMONIC COEFFICIENTS */
    this.c[0][0] = 0.0;
    this.cd[0][0] = 0.0;

    for (i = 0; i < this.coff.length; i++) {
      c_str = this.coff[i];
      c_flds = c_str.split(",");

      n = parseInt(c_flds[0],10);
      m = parseInt(c_flds[1],10);
      gnm = parseFloat(c_flds[2]);
      hnm = parseFloat(c_flds[3]);
      dgnm = parseFloat(c_flds[4]);
      dhnm = parseFloat(c_flds[5]);

      if (m <= n) {
        this.c[m][n] = gnm;
        this.cd[m][n] = dgnm;

        if (m != 0) {
          this.c[n][m-1] = hnm;
          this.cd[n][m-1] = dhnm;
        }
      }
    }

    /* CONVERT SCHMIDT NORMALIZED GAUSS COEFFICIENTS TO UNNORMALIZED */

    this.snorm[0] = 1.0;
    for (n=1; n<=maxord; n++)
    {
        this.snorm[n] = this.snorm[n-1]*(2*n-1)/n;
        j = 2;
        for (m=0,D1=1,D2=(n-m+D1)/D1; D2>0; D2--,m+=D1)
        {
            this.k[m][n] = (((n-1)*(n-1))-(m*m))/((2*n-1)*(2*n-3));
            if (m > 0)
            {
            flnmj = ((n-m+1)*j)/(n+m);
            this.snorm[n+m*13] = this.snorm[n+(m-1)*13]*Math.sqrt(flnmj);
            j = 1;
            this.c[n][m-1] = this.snorm[n+m*13]*this.c[n][m-1];
            this.cd[n][m-1] = this.snorm[n+m*13]*this.cd[n][m-1];
            }
            this.c[m][n] = this.snorm[n+m*13]*this.c[m][n];
            this.cd[m][n] = this.snorm[n+m*13]*this.cd[m][n];
        }
        this.fn[n] = (n+1);
        this.fm[n] = n;
    }
    this.k[1][1] = 0.0;
    this.fm[0] = 0.0;// !!!!!! WMM C and Fortran both have a bug in that fm[0] is not initialised
  };

  WorldMagneticModel.prototype.declination = function(altitudeKm, latitudeDegrees, longitudeDegrees, yearFloat) {

  /* locals */

  var a = 6378.137;
  var b = 6356.7523142;
  var re = 6371.2;
  var a2 = a*a;
  var b2 = b*b;
  var c2 = a2-b2;
  var a4 = a2*a2;
  var b4 = b2*b2;
  var c4 = a4 - b4;
  var D3, D4;
  var dip, ti, gv, dec;
  var n,m;

  var pi, dt, rlon, rlat, srlon, srlat, crlon, crlat,srlat2,
  crlat2, q, q1, q2, ct, d,aor,ar, br, r, r2, bpp, par,
  temp1,parp,temp2,bx,by,bz,bh,dtr,bp,bt, st,ca,sa;

  var maxord = 12;
  var alt = altitudeKm;
  var glon = longitudeDegrees;
  var glat = latitudeDegrees;

  /*************************************************************************/

  dt = yearFloat - 2020.0;
  //if more then 5 years has passed since last epoch update then return invalid
  if ((dt < 0.0) || (dt > 5.0))
      return -999;


  pi = 3.14159265359;
  dtr = pi/180.0;
  rlon = glon*dtr;
  rlat = glat*dtr;
  srlon = Math.sin(rlon);
  srlat = Math.sin(rlat);
  crlon = Math.cos(rlon);
  crlat = Math.cos(rlat);
  srlat2 = srlat*srlat;
  crlat2 = crlat*crlat;
  this.sp[1] = srlon;
  this.cp[1] = crlon;

  /* CONVERT FROM GEODETIC COORDS. TO SPHERICAL COORDS. */

  q = Math.sqrt(a2-c2*srlat2);
  q1 = alt*q;
  q2 = ((q1+a2)/(q1+b2))*((q1+a2)/(q1+b2));
  ct = srlat/Math.sqrt(q2*crlat2+srlat2);
  st = Math.sqrt(1.0-(ct*ct));
  r2 = (alt*alt)+2.0*q1+(a4-c4*srlat2)/(q*q);
  r = Math.sqrt(r2);
  d = Math.sqrt(a2*crlat2+b2*srlat2);
  ca = (alt+d)/r;
  sa = c2*crlat*srlat/(r*d);

  for (m=2; m<=maxord; m++)
  {
      this.sp[m] = this.sp[1]*this.cp[m-1]+this.cp[1]*this.sp[m-1];
      this.cp[m] = this.cp[1]*this.cp[m-1]-this.sp[1]*this.sp[m-1];
  }

  aor = re/r;
  ar = aor*aor;
  br = bt = bp = bpp = 0.0;

  for (n=1; n<=maxord; n++)
  {
      ar = ar*aor;
      for (m=0,D3=1,D4=(n+m+D3)/D3; D4>0; D4--,m+=D3)
      {
          /*
             COMPUTE UNNORMALIZED ASSOCIATED LEGENDRE POLYNOMIALS
             AND DERIVATIVES VIA RECURSION RELATIONS
          */


          if (n == m)
          {
            this.snorm[n+m*13] = st*this.snorm[n-1+(m-1)*13];
            this.dp[m][n] = st*this.dp[m-1][n-1]+ct*this.snorm[n-1+(m-1)*13];
          }
          else if (n == 1 && m == 0)
          {
            this.snorm[n+m*13] = ct*this.snorm[n-1+m*13];
            this.dp[m][n] = ct*this.dp[m][n-1]-st*this.snorm[n-1+m*13];
          }
          else if (n > 1 && n != m)
          {
            if (m > n-2) this.snorm[n-2+m*13] = 0.0;
            if (m > n-2) this.dp[m][n-2] = 0.0;
            this.snorm[n+m*13] = ct*this.snorm[n-1+m*13]-this.k[m][n]*this.snorm[n-2+m*13];
            this.dp[m][n] = ct*this.dp[m][n-1] - st*this.snorm[n-1+m*13]-this.k[m][n]*this.dp[m][n-2];
           }

          /*
          TIME ADJUST THE GAUSS COEFFICIENTS
          */
          this.tc[m][n] = this.c[m][n]+dt*this.cd[m][n];
          if (m != 0) this.tc[n][m-1] = this.c[n][m-1]+dt*this.cd[n][m-1];

          /*
          ACCUMULATE TERMS OF THE SPHERICAL HARMONIC EXPANSIONS
          */
          par = ar*this.snorm[n+m*13];
          if (m == 0)
          {
              temp1 = this.tc[m][n]*this.cp[m];
              temp2 = this.tc[m][n]*this.sp[m];
          }
          else
          {
              temp1 = this.tc[m][n]*this.cp[m]+this.tc[n][m-1]*this.sp[m];
              temp2 = this.tc[m][n]*this.sp[m]-this.tc[n][m-1]*this.cp[m];
          }
          bt = bt-ar*temp1*this.dp[m][n];
          bp += (this.fm[m]*temp2*par);
          br += (this.fn[n]*temp1*par);
          /*
          SPECIAL CASE:  NORTH/SOUTH GEOGRAPHIC POLES
          */
          if (st == 0.0 && m == 1)
          {
              if (n == 1) this.pp[n] = this.pp[n-1];
              else this.pp[n] = this.ct*this.pp[n-1]-this.k[m][n]*this.pp[n-2];
              parp = ar*this.pp[n];
              bpp += (this.fm[m]*temp2*parp);
          }
      }
  }

  if (st == 0.0)
      bp = bpp;
  else
      bp /= st;

  /*
      ROTATE MAGNETIC VECTOR COMPONENTS FROM SPHERICAL TO
      GEODETIC COORDINATES
  */
  bx = -bt*ca-br*sa;
  by = bp;
  bz = bt*sa-br*ca;
  /*
      COMPUTE DECLINATION (DEC), INCLINATION (DIP) AND
      TOTAL INTENSITY (TI)
  */
  bh = Math.sqrt((bx*bx)+(by*by));
  ti = Math.sqrt((bh*bh)+(bz*bz));
  dec = Math.atan2(by,bx)/dtr;
  dip = Math.atan2(bz,bh)/dtr;
  /*
      COMPUTE MAGNETIC GRID VARIATION IF THE CURRENT
      GEODETIC POSITION IS IN THE ARCTIC OR ANTARCTIC
      (I.E. GLAT > +55 DEGREES OR GLAT < -55 DEGREES)

      OTHERWISE, SET MAGNETIC GRID VARIATION TO -999.0
  */
  gv = -999.0;
  if (Math.abs(glat) >= 55.0)
  {
      if (glat > 0.0 && glon >= 0.0) gv = dec-glon;
      if (glat > 0.0 && glon < 0.0) gv = dec+Math.abs(glon);
      if (glat < 0.0 && glon >= 0.0) gv = dec+glon;
      if (glat < 0.0 && glon < 0.0) gv = dec-Math.abs(glon);
      if (gv > +180.0) gv -= 360.0;
      if (gv < -180.0) gv += 360.0;
  }

  return dec;
  }

  WorldMagneticModel.prototype.knownAnswerTest = function(){

  /* http://www.ngdc.noaa.gov/geomag/WMM WMM2010testvalues.pdf */

  /* Lat	Lon Dec	    */
  /* Lon 240 = 120W, Lon 300 = 60W */

  /* Alt 0 km */
  var kat2010 = [
  "80.00	,0.00	 ,-6.13	    ",
  "0.00	,120.00	 ,0.97	    ",
  "-80.00	,240.00	 ,70.21	    "
  ];

  var kat2012p5 = [
  "80.00	,0.00	 ,-5.21	    ",
  "0.00	,120.00	 ,0.88	    ",
  "-80.00	,240.00	 ,70.04	    "
  ];

  var maxErr = 0.0;

  for(var i=0; i<kat2010.length; i++){

      var c_str = kat2010[i];
    var c_flds = c_str.split(",");

    var lat = parseFloat(c_flds[0]);
    var lon = parseFloat(c_flds[1]);
    var exp = parseFloat(c_flds[2]);
    var maxExp;

    var dec = this.declination(0,lat,lon,2010.0);
    if(Math.abs(dec-exp) > maxErr){
        maxErr = Math.abs(dec-exp);
        maxExp = exp;
    }

  }

  for (var i = 0; i < kat2012p5.length; i++) {

      var c_str = kat2012p5[i];
      var c_flds = c_str.split(",");

      var lat = parseFloat(c_flds[0]);
      var lon = parseFloat(c_flds[1]);
      var exp = parseFloat(c_flds[2]);
      var maxExp;

      var dec = this.declination(0, lat, lon, 2012.5);
      if (Math.abs(dec - exp) > maxErr) {
          maxErr = Math.abs(dec - exp);
          maxExp = exp;
      }

  }


  return maxErr * 100 / maxExp;//max % error

  }

  /*

  C***********************************************************************
  C
  C
  C     SUBROUTINE GEOMAG (GEOMAGNETIC FIELD COMPUTATION)
  C
  C
  C***********************************************************************
  C
  C     GEOMAG IS A NATIONAL GEOSPATIAL INTELLIGENCE AGENCY (NGA) STANDARD
  C     PRODUCT.  IT IS COVERED UNDER NGA MILITARY SPECIFICATION:
  C     MIL-W-89500 (1993).
  C
  C***********************************************************************
  C     Contact Information
  C
  C     Software and Model Support
  C     	National Geophysical Data Center
  C     	NOAA EGC/2
  C     	325 Broadway
  C     	Boulder, CO 80303 USA
  C     	Attn: Susan McLean or Stefan Maus
  C     	Phone:  (303) 497-6478 or -6522
  C     	Email:  Susan.McLean@noaa.gov or Stefan.Maus@noaa.gov
  C		Web: http://www.ngdc.noaa.gov/seg/WMM/
  C
  C     Sponsoring Government Agency
  C	   National Geospatial-Intelligence Agency
  C    	   PRG / CSAT, M.S. L-41
  C    	   3838 Vogel Road
  C    	   Arnold, MO 63010
  C    	   Attn: Craig Rollins
  C    	   Phone:  (314) 263-4186
  C    	   Email:  Craig.M.Rollins@Nga.Mil
  C
  C      Original Program By:
  C        Dr. John Quinn
  C        FLEET PRODUCTS DIVISION, CODE N342
  C        NAVAL OCEANOGRAPHIC OFFICE (NAVOCEANO)
  C        STENNIS SPACE CENTER (SSC), MS 39522-5001
  C
  C***********************************************************************
  C
  C     PURPOSE:  THIS ROUTINE COMPUTES THE DECLINATION (DEC),
  C               INCLINATION (DIP), TOTAL INTENSITY (TI) AND
  C               GRID VARIATION (GV - POLAR REGIONS ONLY, REFERENCED
  C               TO GRID NORTH OF A STEREOGRAPHIC PROJECTION) OF THE
  C               EARTH'S MAGNETIC FIELD IN GEODETIC COORDINATES
  C               FROM THE COEFFICIENTS OF THE CURRENT OFFICIAL
  C               DEPARTMENT OF DEFENSE (DOD) SPHERICAL HARMONIC WORLD
  C               MAGNETIC MODEL (WMM.COF).  THE WMM SERIES OF MODELS IS
  C               UPDATED EVERY 5 YEARS ON JANUARY 1ST OF THOSE YEARS
  C               WHICH ARE DIVISIBLE BY 5 (I.E. 2000, 2005, 2010 ETC.)
  C               BY NOAA'S NATIONAL GEOPHYSICAL DATA CENTER IN
  C               COOPERATION WITH THE BRITISH GEOLOGICAL SURVEY (BGS).
  C               THE MODEL IS BASED ON GEOMAGNETIC FIELD MEASUREMENTS
  C               FROM SATELLITE AND GROUND OBSERVATORIES.
  C
  C***********************************************************************
  C
  C     MODEL:  THE WMM SERIES GEOMAGNETIC MODELS ARE COMPOSED
  C             OF TWO PARTS:  THE MAIN FIELD MODEL, WHICH IS
  C             VALID AT THE BASE EPOCH OF THE CURRENT MODEL AND
  C             A SECULAR VARIATION MODEL, WHICH ACCOUNTS FOR SLOW
  C             TEMPORAL VARIATIONS IN THE MAIN GEOMAGNETIC FIELD
  C             FROM THE BASE EPOCH TO A MAXIMUM OF 5 YEARS BEYOND
  C             THE BASE EPOCH.  FOR EXAMPLE, THE BASE EPOCH OF
  C             THE WMM-2005 MODEL IS 2005.0.  THIS MODEL IS THEREFORE
  C             CONSIDERED VALID BETWEEN 2005.0 AND 2010.0. THE
  C             COMPUTED MAGNETIC PARAMETERS ARE REFERENCED TO THE
  C             WGS-84 ELLIPSOID.
  C
  C***********************************************************************
  C
  C     ACCURACY:  IN OCEAN AREAS AT THE EARTH'S SURFACE OVER THE
  C                ENTIRE 5 YEAR LIFE OF THE DEGREE AND ORDER 12
  C                SPHERICAL HARMONIC MODEL WMM-2005, THE ESTIMATED
  C                MAXIMUM RMS ERRORS FOR THE VARIOUS MAGNETIC COMPONENTS
  C                ARE:
  C
  C                DEC  -   0.5 Degrees
  C                DIP  -   0.5 Degrees
  C                TI   - 280.0 nanoTeslas (nT)
  C                GV   -   0.5 Degrees
  C
  C                OTHER MAGNETIC COMPONENTS THAT CAN BE DERIVED FROM
  C                THESE FOUR BY SIMPLE TRIGONOMETRIC RELATIONS WILL
  C                HAVE THE FOLLOWING APPROXIMATE ERRORS OVER OCEAN AREAS:
  C
  C                X    - 140 nT (North)
  C                Y    - 140 nT (East)
  C                Z    - 200 nT (Vertical) Positive is down
  C                H    - 200 nT (Horizontal)
  C
  C                OVER LAND THE MAXIMUM RMS ERRORS ARE EXPECTED TO BE
  C                HIGHER, ALTHOUGH THE RMS ERRORS FOR DEC, DIP, AND GV
  C                ARE STILL ESTIMATED TO BE LESS THAN 1.0 DEGREE, FOR
  C                THE ENTIRE 5-YEAR LIFE OF THE MODEL AT THE EARTH's
  C                SURFACE.  THE OTHER COMPONENT ERRORS OVER LAND ARE
  C                MORE DIFFICULT TO ESTIMATE AND SO ARE NOT GIVEN.
  C
  C                THE ACCURACY AT ANY GIVEN TIME FOR ALL OF THESE
  C                GEOMAGNETIC PARAMETERS DEPENDS ON THE GEOMAGNETIC
  C                LATITUDE.  THE ERRORS ARE LEAST FROM THE EQUATOR TO
  C                MID-LATITUDES AND GREATEST NEAR THE MAGNETIC POLES.
  C
  C                IT IS VERY IMPORTANT TO NOTE THAT A DEGREE AND
  C                ORDER 12 MODEL, SUCH AS WMM-2005, DESCRIBES ONLY
  C                THE LONG WAVELENGTH SPATIAL MAGNETIC FLUCTUATIONS
  C                DUE TO EARTH'S CORE.  NOT INCLUDED IN THE WMM SERIES
  C                MODELS ARE INTERMEDIATE AND SHORT WAVELENGTH
  C                SPATIAL FLUCTUATIONS OF THE GEOMAGNETIC FIELD
  C                WHICH ORIGINATE IN THE EARTH'S MANTLE AND CRUST.
  C                CONSEQUENTLY, ISOLATED ANGULAR ERRORS AT VARIOUS
  C                POSITIONS ON THE SURFACE (PRIMARILY OVER LAND, IN
  C                CONTINENTAL MARGINS AND OVER OCEANIC SEAMOUNTS,
  C                RIDGES AND TRENCHES) OF SEVERAL DEGREES MAY BE
  C                EXPECTED. ALSO NOT INCLUDED IN THE MODEL ARE
  C                NONSECULAR TEMPORAL FLUCTUATIONS OF THE GEOMAGNETIC
  C                FIELD OF MAGNETOSPHERIC AND IONOSPHERIC ORIGIN.
  C                DURING MAGNETIC STORMS, TEMPORAL FLUCTUATIONS CAN
  C                CAUSE SUBSTANTIAL DEVIATIONS OF THE GEOMAGNETIC
  C                FIELD FROM MODEL VALUES.  IN ARCTIC AND ANTARCTIC
  C                REGIONS, AS WELL AS IN EQUATORIAL REGIONS, DEVIATIONS
  C                FROM MODEL VALUES ARE BOTH FREQUENT AND PERSISTENT.
  C
  C                IF THE REQUIRED DECLINATION ACCURACY IS MORE
  C                STRINGENT THAN THE WMM SERIES OF MODELS PROVIDE, THEN
  C                THE USER IS ADVISED TO REQUEST SPECIAL (REGIONAL OR
  C                LOCAL) SURVEYS BE PERFORMED AND MODELS PREPARED.
  C                REQUESTS OF THIS NATURE SHOULD BE MADE TO NIMA
  C                AT THE ADDRESS ABOVE.
  C
  C***********************************************************************
  C
  C     USAGE:  THIS ROUTINE IS BROKEN UP INTO TWO PARTS:
  C
  C             A) AN INITIALIZATION MODULE, WHICH IS CALLED ONLY
  C                ONCE AT THE BEGINNING OF THE MAIN (CALLING)
  C                PROGRAM
  C             B) A PROCESSING MODULE, WHICH COMPUTES THE MAGNETIC
  C                FIELD PARAMETERS FOR EACH SPECIFIED GEODETIC
  C                POSITION (ALTITUDE, LATITUDE, LONGITUDE) AND TIME
  C
  C             INITIALIZATION IS MADE VIA A SINGLE CALL TO THE MAIN
  C             ENTRY POINT (GEOMAG), WHILE SUBSEQUENT PROCESSING
  C             CALLS ARE MADE THROUGH THE SECOND ENTRY POINT (GEOMG1).
  C             ONE CALL TO THE PROCESSING MODULE IS REQUIRED FOR EACH
  C             POSITION AND TIME.
  C
  C             THE VARIABLE MAXDEG IN THE INITIALIZATION CALL IS THE
  C             MAXIMUM DEGREE TO WHICH THE SPHERICAL HARMONIC MODEL
  C             IS TO BE COMPUTED.  IT MUST BE SPECIFIED BY THE USER
  C             IN THE CALLING ROUTINE.  NORMALLY IT IS 12 BUT IT MAY
  C             BE SET LESS THAN 12 TO INCREASE COMPUTATIONAL SPEED AT
  C             THE EXPENSE OF REDUCED ACCURACY.
  C
  C             THE PC VERSION OF THIS SUBROUTINE MUST BE COMPILED
  C             WITH A FORTRAN 77 COMPATIBLE COMPILER SUCH AS THE
  C             MICROSOFT OPTIMIZING FORTRAN COMPILER VERSION 4.1
  C             OR LATER.
  C
  C**********************************************************************
  C
  C     REFERENCES:
  C
  C       JOHN M. QUINN, DAVID J. KERRIDGE AND DAVID R. BARRACLOUGH,
  C            WORLD MAGNETIC CHARTS FOR 1985 - SPHERICAL HARMONIC
  C            MODELS OF THE GEOMAGNETIC FIELD AND ITS SECULAR
  C            VARIATION, GEOPHYS. J. R. ASTR. SOC. (1986) 87,
  C            PP 1143-1157
  C
  C       DEFENSE MAPPING AGENCY TECHNICAL REPORT, TR 8350.2:
  C            DEPARTMENT OF DEFENSE WORLD GEODETIC SYSTEM 1984,
  C            SEPT. 30 (1987)
  C
  C       JOHN M. QUINN, RACHEL J. COLEMAN, MICHAEL R. PECK, AND
  C            STEPHEN E. LAUBER; THE JOINT US/UK 1990 EPOCH
  C            WORLD MAGNETIC MODEL, TECHNICAL REPORT NO. 304,
  C            NAVAL OCEANOGRAPHIC OFFICE (1991)
  C
  C       JOHN M. QUINN, RACHEL J. COLEMAN, DONALD L. SHIEL, AND
  C            JOHN M. NIGRO; THE JOINT US/UK 1995 EPOCH WORLD
  C            MAGNETIC MODEL, TECHNICAL REPORT NO. 314, NAVAL
  C            OCEANOGRAPHIC OFFICE (1995)
  C
  C            SUSAN AMCMILLAN, DAVID R. BARRACLOUGH, JOHN M. QUINN, AND
  C            RACHEL J. COLEMAN;  THE 1995 REVISION OF THE JOINT US/UK
  C            GEOMAGNETIC FIELD MODELS - I. SECULAR VARIATION, JOURNAL OF
  C            GEOMAGNETISM AND GEOELECTRICITY, VOL. 49, PP. 229-243
  C            (1997)
  C
  C            JOHN M. QUINN, RACHEL J. COELMAN, SUSAM MACMILLAN, AND
  C            DAVID R. BARRACLOUGH;  THE 1995 REVISION OF THE JOINT
  C            US/UK GEOMAGNETIC FIELD MODELS: II. MAIN FIELD,JOURNAL OF
  C            GEOMAGNETISM AND GEOELECTRICITY, VOL. 49, PP. 245 - 261
  C            (1997)
  C
  C***********************************************************************
  C
  C     PARAMETER DESCRIPTIONS:
  C
  C       A      - SEMIMAJOR AXIS OF WGS-84 ELLIPSOID (KM)
  C       B      - SEMIMINOR AXIS OF WGS-84 ELLIPSOID (KM)
  C       RE     - MEAN RADIUS OF IAU-66 ELLIPSOID (KM)
  C       SNORM  - SCHMIDT NORMALIZATION FACTORS
  C       C      - GAUSS COEFFICIENTS OF MAIN GEOMAGNETIC MODEL (NT)
  C       CD     - GAUSS COEFFICIENTS OF SECULAR GEOMAGNETIC MODEL (NT/YR)
  C       TC     - TIME ADJUSTED GEOMAGNETIC GAUSS COEFFICIENTS (NT)
  C       OTIME  - TIME ON PREVIOUS CALL TO GEOMAG (YRS)
  C       OALT   - GEODETIC ALTITUDE ON PREVIOUS CALL TO GEOMAG (YRS)
  C       OLAT   - GEODETIC LATITUDE ON PREVIOUS CALL TO GEOMAG (DEG.)
  C       TIME   - COMPUTATION TIME (YRS)                        (INPUT)
  C                (EG. 1 JULY 1995 = 1995.500)
  C       ALT    - GEODETIC ALTITUDE (KM)                        (INPUT)
  C       GLAT   - GEODETIC LATITUDE (DEG.)                      (INPUT)
  C       GLON   - GEODETIC LONGITUDE (DEG.)                     (INPUT)
  C       EPOCH  - BASE TIME OF GEOMAGNETIC MODEL (YRS)
  C       DTR    - DEGREE TO RADIAN CONVERSION
  C       SP(M)  - SINE OF (M*SPHERICAL COORD. LONGITUDE)
  C       CP(M)  - COSINE OF (M*SPHERICAL COORD. LONGITUDE)
  C       ST     - SINE OF (SPHERICAL COORD. LATITUDE)
  C       CT     - COSINE OF (SPHERICAL COORD. LATITUDE)
  C       R      - SPHERICAL COORDINATE RADIAL POSITION (KM)
  C       CA     - COSINE OF SPHERICAL TO GEODETIC VECTOR ROTATION ANGLE
  C       SA     - SINE OF SPHERICAL TO GEODETIC VECTOR ROTATION ANGLE
  C       BR     - RADIAL COMPONENT OF GEOMAGNETIC FIELD (NT)
  C       BT     - THETA COMPONENT OF GEOMAGNETIC FIELD (NT)
  C       BP     - PHI COMPONENT OF GEOMAGNETIC FIELD (NT)
  C       P(N,M) - ASSOCIATED LEGENDRE POLYNOMIALS (UNNORMALIZED)
  C       PP(N)  - ASSOCIATED LEGENDRE POLYNOMIALS FOR M=1 (UNNORMALIZED)
  C       DP(N,M)- THETA DERIVATIVE OF P(N,M) (UNNORMALIZED)
  C       BX     - NORTH GEOMAGNETIC COMPONENT (NT)
  C       BY     - EAST GEOMAGNETIC COMPONENT (NT)
  C       BZ     - VERTICALLY DOWN GEOMAGNETIC COMPONENT (NT)
  C       BH     - HORIZONTAL GEOMAGNETIC COMPONENT (NT)
  C       DEC    - GEOMAGNETIC DECLINATION (DEG.)                (OUTPUT)
  C                  EAST=POSITIVE ANGLES
  C                  WEST=NEGATIVE ANGLES
  C       DIP    - GEOMAGNETIC INCLINATION (DEG.)                (OUTPUT)
  C                  DOWN=POSITIVE ANGLES
  C                    UP=NEGATIVE ANGLES
  C       TI     - GEOMAGNETIC TOTAL INTENSITY (NT)              (OUTPUT)
  C       GV     - GEOMAGNETIC GRID VARIATION (DEG.)             (OUTPUT)
  C                REFERENCED TO GRID NORTH
  C                GRID NORTH REFERENCED TO 0 MERIDIAN
  C                OF A POLAR STEREOGRAPHIC PROJECTION
  C                (ARCTIC/ANTARCTIC ONLY)
  C       MAXDEG - MAXIMUM DEGREE OF SPHERICAL HARMONIC MODEL    (INPUT)
  C       MOXORD - MAXIMUM ORDER OF SPHERICAL HARMONIC MODEL
  C
  C***********************************************************************
  C
  C     NOTE:  THIS VERSION OF GEOMAG USES A WMM SERIES GEOMAGNETIC
  C            FIELS MODEL REFERENCED TO THE WGS-84 GRAVITY MODEL
  C            ELLIPSOID
  C


  */