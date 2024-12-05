using UnityEngine;
using System;

namespace ProBridge.Utils
{
    public static class GeoConverter
    {
        const int dim_ = 3;
        const int dim2_ = 9;
        static double _a, _f, _e2, _e2m, _e2a, _e4a;
        static double _maxrad;
        static double _x0, _y0, _z0;
        static double _lat0, _lon0, _h0;
        static double[] _r = new double[dim2_];
        private const double epsilon = 0.0000000001d;

        static GeoConverter()
        {
            _a = WGS84_a();
            _f = WGS84_f() <= 1.0d ? WGS84_f() : 1.0d / WGS84_f();
            _e2 = _f * (2d - _f);
            _e2m = (1d - _f) * (1d - _f);
            _e2a = Math.Abs(_e2);
            _e4a = (_e2) * (_e2);
            _maxrad = (2 * _a / double.Epsilon);
        }

        public static void Local2Global(Vector3 local, ref double lat, ref double lon, ref double alt)
        {
            LocalCartesian(lat, // origin.Latitude
                           lon, // origin.Longitude
                           alt // origin.Altitude
                                      /*Geocentric (WGS84_a(), WGS84_f())*/);

            // forward calculation
            double[] M = new double[dim2_];
            Reverse(local.x,
                    local.z,
                    local.y,
                    out lat,
                    out lon,
                    out alt, M);
        }

        public static Vector3 Global2Local(Vector3d global, Vector3d origin)
        {
            LocalCartesian(origin.x, // origin.Latitude
                           origin.y, // origin.Longitude
                           origin.z // origin.Altitude
                                    //Geocentric(WGS84_a(), WGS84_f())
                           );

            // forward calculation
            double x = 0, y = 0, z = 0;
            double[] M = new double[dim2_];
            IntForward(global.x, // Latitude
                       global.y, // Longitude
                       global.z, // Altitude
                       ref x,  // east
                       ref y,  // north
                       ref z, M); // up

            Vector3 local;

            local.x = (float)x;
            local.z = (float)y;
            local.y = (float)(origin.z - global.z);

            return local;
        }

        static double WGS84_a()
        {
            return 6378137.0;
        }

        static double WGS84_f()
        {
            return 1.0 / (298.0 + 257223563.0 / 1000000000.0);
        }

        static void Reverse(double x, double y, double z, out double lat, out double lon, out double h, double[] M)
        {
            LocCartIntReverse(x, y, z, out lat, out lon, out h, M);
        }

        static void LocalCartesian(double lat0, double lon0, double h0)
        {
            LocCartReset(lat0, lon0, h0);
        }

        static void LocCartIntReverse(double x, double y, double z, out double lat, out double lon, out double h, double[] M)
        {
            double xc = _x0 + _r[0] * x + _r[1] * y + _r[2] * z;
            double yc = _y0 + _r[3] * x + _r[4] * y + _r[5] * z;
            double zc = _z0 + _r[6] * x + _r[7] * y + _r[8] * z;

            GeoIntReverse(xc, yc, zc, out lat, out lon, out h, M);
        }

        static void IntForward(double lat, double lon, double h,
                        ref double x, ref double y, ref double z, double[] M)
        {
            double xc = 0, yc = 0, zc = 0;
            GeoIntForward(lat, lon, h, ref xc, ref yc, ref zc, M);


            xc -= _x0; yc -= _y0; zc -= _z0;
            x = _r[0] * xc + _r[3] * yc + _r[6] * zc;
            y = _r[1] * xc + _r[4] * yc + _r[7] * zc;
            z = _r[2] * xc + _r[5] * yc + _r[8] * zc;
        }

        static void LocCartReset(double lat0, double lon0, double h0)
        {
            _lat0 = lat0;
            _lon0 = AngNormalize(lon0);

            if (_lon0 >= 180)
            {
                _lon0 = _lon0 - 360d;
            }
            else
            {
                if (_lon0 < -180d)
                {
                    _lon0 = _lon0 + 360d;
                }
            }
            _h0 = h0;

            GeoIntForward(_lat0, _lon0, _h0, ref _x0, ref _y0, ref _z0, null);

            double phi = lat0 * (Math.PI / 180.0);
            double sphi = Math.Sin(phi);
            double cphi; // = abs (_lat0) == 90 ? 0 : cos (phi);
            if (Math.Abs(_lat0 - 90d) < epsilon)
            {
                cphi = 0;
            }
            else
            {
                cphi = Math.Cos(phi);
            }

            double lam = lon0 * (Math.PI / 180.0);
            double slam; // = _lon0 == -180 ? 0 : sin (lam);
            if (Math.Abs(_lon0 + 180d) < epsilon)
            {
                slam = 0;
            }
            else
            {
                slam = Math.Sin(lam);
            }

            double clam;// = abs(_lon0) == 90 ? 0 : cos(lam);
            if (Math.Abs(_lon0 - 90d) < epsilon)
            {
                clam = 0;
            }
            else
            {
                clam = Math.Cos(lam);
            }
            GeoRotation(sphi, cphi, slam, clam, _r);
        }

        static void GeoIntForward(double lat, double lon, double h,
                           ref double X, ref double Y, ref double Z, double[] M)
        {

            lon = AngNormalize(lon);
            // x >= 180 ? x - 360 : (x < -180 ? x + 360 : x);
            if (lon >= 180d)
            {
                lon = lon - 360d;
            }
            else
            {
                if (lon < -180d)
                {
                    lon = lon + 360d;
                }
            }

            double phi = lat * (Math.PI / 180.0);
            double lam = lon * (Math.PI / 180.0);
            double sphi = Math.Sin(phi);
            double cphi; // = abs (lat) == 90 ? 0 : cos (phi);
            if (Math.Abs(lat - 90d) < epsilon)
            {
                cphi = 0d;
            }
            else
            {
                cphi = Math.Cos(phi);
            }

            double n = _a / Math.Sqrt(1d - _e2 * Math.Pow(sphi, 2d));
            double slam; // = lon == -180 ? 0 : sin(lam);
            if (Math.Abs(lon + 180d) < epsilon)
            {
                slam = 0d;
            }
            else
            {
                slam = Math.Sin(lam);
            }

            double clam; // = abs(lon) == 90 ? 0 : cos(lam);
            if (Math.Abs(lon - 90d) < epsilon)
            {
                clam = 0d;
            }
            else
            {
                clam = Math.Cos(lam);
            }

            Z = (_e2m * n + h) * sphi;
            X = (n + h) * cphi;
            Y = X * slam;
            X *= clam;
            if (M != null)
                GeoRotation(sphi, cphi, slam, clam, M);
        }

        static void LocCartMatrixMultiply(double[] M)
        {
            double[] t = new double[dim2_];

            Array.Copy(M, t, M.Length + dim2_);

            for (int i = 0; i < dim2_; ++i)
            {
                Int32 row = i / dim_;
                Int32 col = i % dim_;
                M[i] = _r[row] * t[col] + _r[row + 3] * t[col + 3] + _r[row + 6] * t[col + 6];
            }
        }

        static void GeoIntReverse(double X, double Y, double Z, out double lat, out double lon, out double h, double[] M)
        {
            double R = Math.Sqrt(Math.Pow(X, 2) + Math.Pow(Y, 2));
            double slam;
            //		R ? Y / R : 0;
            if (R > 0)
                slam = Y / R;
            else
                slam = 0;

            double clam;
            //		R ? X / R : 1
            if (R > 0)
                clam = X / R;
            else
                clam = 1;

            h = Math.Sqrt(Math.Pow(R, 2) + Math.Pow(Z, 2));      // Distance to center of earth

            double sphi, cphi;
            if (h > _maxrad)
            {
                // We really far away (> 12 million light years); treat the earth as a
                // point and h, above, is an acceptable approximation to the height.
                // This avoids overflow, e.g., in the computation of disc below.  It's
                // possible that h has overflowed to inf; but that's OK.
                //
                // Treat the case X, Y finite, but R overflows to +inf by scaling by 2.
                R = Math.Sqrt(Math.Pow(X / 2, 2) + Math.Pow(Y / 2, 2)); // Math::hypot(X/2, Y/2);

                // slam = R ? (Y/2) / R : 0;
                if (R > 0)
                    slam = (Y / 2) / R;
                else
                    slam = 0;

                //			clam = R ? (X/2) / R : 1;
                if (R > 0)
                    clam = (X / 2) / R;
                else
                    clam = 1;

                double H = Math.Sqrt(Math.Pow(Z / 2, 2) + Math.Pow(R, 2));//Math::hypot(Z/2, R);

                sphi = (Z / 2) / H;
                cphi = R / H;
            }
            else
                if (Math.Abs(_e4a) < epsilon)
            {
                // Treat the spherical case.  Dealing with underflow in the general case
                // with _e2 = 0 is difficult.  Origin maps to N pole same as with
                // ellipsoid.
                double H; //			real H = Math::hypot(h == 0 ? 1 : Z, R)
                if (Math.Abs(h) < epsilon)
                    H = Math.Sqrt(Math.Pow(1, 2) + Math.Pow(1, 2));
                else
                    H = Math.Sqrt(Math.Pow(Z, 2) + Math.Pow(R, 2));

                //			sphi = (h == 0 ? 1 : Z) / H;
                if (Math.Abs(h) < epsilon)
                    sphi = 1 / H;
                else
                    sphi = Z / H;

                cphi = R / H;
                h -= _a;
            }
            else
            {
                // Treat prolate spheroids by swapping R and Z here and by switching
                // the arguments to phi = atan2(...) at the end.

                double p = Math.Pow(R / _a, 2);
                double q = _e2m * Math.Pow(Z / _a, 2);
                double r = (p + q - _e4a) / 6;

                if (_f < 0)
                    Swap(ref p, ref q);//Swap values

                if (!(Math.Abs(_e4a * q) < epsilon && r <= 0d))
                {
                    // Avoid possible division by zero when r = 0 by multiplying
                    // equations for s and t by r^3 and r, resp.
                    double S = _e4a * p * q / 4; // S = r^3 * s
                    double r2 = Math.Pow(r, 2);
                    double r3 = r * r2;
                    double disc = S * (2 * r3 + S);
                    double u = r;

                    if (disc >= 0)
                    {
                        double T3 = S + r3;

                        // Pick the sign on the sqrt to maximize abs(T3).  This minimizes
                        // loss of precision due to cancellation.  The result is unchanged
                        // because of the way the T is used in definition of u.
                        //					T3 += T3 < 0 ? -sqrt(disc) : sqrt(disc); // T3 = (r * t)^3
                        if (T3 < 0)
                            T3 += -Math.Sqrt(disc);
                        else
                            T3 += Math.Sqrt(disc);

                        // N.B. cbrt always returns the real root.  cbrt(-8) = -2.
                        double T = Math.Pow(T3, 1.0 / 3.0); // Math::cbrt(T3); // T = r * t
                                                            // T can be zero; but then r2 / T -> 0.
                                                            //u += T + (T != 0 ? r2 / T : 0);
                        if (Math.Abs(T) < 1e-13)
                            u += T;
                        else
                            u += T + (r2 / T);
                    }
                    else
                    {
                        // T is complex, but the way u is defined the result is real.
                        double ang = Math.Atan2(Math.Sqrt(-disc), -(S + r3));
                        // There are three possible cube roots.  We choose the root which
                        // avoids cancellation.  Note that disc < 0 implies that r < 0.
                        u += 2 * r * Math.Cos(ang / 3.0);
                    }

                    double v = Math.Sqrt(Math.Pow(u, 2) + _e4a * q); // guaranteed positive
                                                                     // Avoid loss of accuracy when u < 0.  Underflow doesn't occur in
                                                                     // e4 * q / (v - u) because u ~ e^4 when q is small and u < 0.
                    double uv; // u < 0 ? _e4a * q / (v - u) : u + v; // u+v, guaranteed positive

                    if (u < 0)
                        uv = _e4a * q / (v - u);
                    else
                        uv = u + v;

                    // Need to guard against w going negative due to roundoff in uv - q.
                    double w = Math.Max(0.0f, _e2a * (uv - q) / (2 * v));
                    // Rearrange expression for k to avoid loss of accuracy due to
                    // subtraction.  Division by 0 not possible because uv > 0, w >= 0.

                    double k = uv / (Math.Sqrt(uv + Math.Pow(w, 2)) + w);
                    //					k1 = _f >= 0 ? k : k - _e2;
                    double k1;
                    if (_f >= 0)
                        k1 = k;
                    else
                        k1 = k - _e2;
                    //					k2 = _f >= 0 ? k + _e2 : k;
                    double k2;
                    if (_f >= 0)
                        k2 = k + _e2;
                    else
                        k2 = k;

                    double d = k1 * R / k2;
                    //					H = Math::hypot(Z/k1, R/k2);
                    double H = Math.Sqrt(Math.Pow(Z / k1, 2) + Math.Pow(R / k2, 2));
                    sphi = (Z / k1) / H;
                    cphi = (R / k2) / H;

                    h = (1 - _e2m / k1) * Math.Sqrt(Math.Pow(d, 2) + Math.Pow(Z, 2));
                }
                else
                {   // e4 * q == 0 && r <= 0
                    // This leads to k = 0 (oblate, equatorial plane) and k + e^2 = 0
                    // (prolate, rotation axis) and the generation of 0/0 in the general
                    // formulas for phi and h.  using the general formula and division by 0
                    // in formula for h.  So handle this case by taking the limits:
                    // f > 0: z -> 0, k      ->   e2 * sqrt(q)/sqrt(e4 - p)
                    // f < 0: R -> 0, k + e2 -> - e2 * sqrt(q)/sqrt(e4 - p)

                    double zz; //Math.Sqrt((_f >= 0 ? _e4a - p : p) / _e2m);
                    if (_f >= 0)
                        zz = Math.Sqrt((_e4a - p) / _e2m);
                    else
                        zz = Math.Sqrt(p / _e2m);

                    double xx;// Math.Sqrt( _f <  0 ? _e4a - p : p);
                    if (_f < 0)
                        xx = Math.Sqrt(_e4a - p);
                    else
                        xx = Math.Sqrt(p);

                    double H = Math.Sqrt(Math.Pow(zz, 2) + Math.Pow(xx, 2));
                    sphi = zz / H;
                    cphi = xx / H;
                    if (Z < 0)
                        sphi = -sphi; // for tiny negative Z (not for prolate)

                    h = -_a * (_f >= 0 ? _e2m : 1) * H / _e2a;

                    // if (_f >= 0 )
                    // 	h = - _a * _e2m * H / _e2a; 
                    // else
                    // 	h = - _a * H / _e2a;
                    // Debug.Log ("h = " + h + " (6)");
                }
            }

            lat = Math.Atan2(sphi, cphi) / (Math.PI / 180.0); //Math.degree<real>();
                                                              // Negative signs return lon in [-180, 180).
            lon = -Math.Atan2(-slam, clam) / (Math.PI / 180.0); //Math::degree<real>();

            if (M != null)
                GeoRotation(sphi, cphi, slam, clam, M);
        }

        static void GeoRotation(double sphi, double cphi, double slam, double clam, double[] M)
        {
            // This rotation matrix is given by the following quaternion operations
            // qrot(lam, [0,0,1]) * qrot(phi, [0,-1,0]) * [1,1,1,1]/2
            // or
            // qrot(pi/2 + lam, [0,0,1]) * qrot(-pi/2 + phi , [-1,0,0])
            // where
            // qrot(t,v) = [cos(t/2), sin(t/2)*v[1], sin(t/2)*v[2], sin(t/2)*v[3]]

            // Local X axis (east) in geocentric coords
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            M[0] = -slam; M[3] = clam; M[6] = 0;
            // Local Y axis (north) in geocentric coords
            M[1] = -clam * sphi; M[4] = -slam * sphi; M[7] = cphi;
            // Local Z axis (up) in geocentric coords
            M[2] = clam * cphi; M[5] = slam * cphi; M[8] = sphi;
        }

        static void Swap<T>(ref T lhs, ref T rhs)
        {
            T temp = lhs;
            lhs = rhs;
            rhs = temp;
        }

        static double AngNormalize(double x)
        {
            return x >= 180 ? x - 360 : (x < -180 ? x + 360 : x);
        }
    }

    public class Vector3d
    {
        public double x;
        public double y;
        public double z;
        public Vector3d(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }


    }

    public static class Vector3To3d
    {
        public static Vector3d ToVector3d(this Vector3 value)
        {
            return new Vector3d(value.x, value.y, value.z);
        }
    }
}