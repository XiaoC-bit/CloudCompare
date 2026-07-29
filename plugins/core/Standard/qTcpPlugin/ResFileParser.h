#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <regex>
#include "Eigen/Dense"


// ============================================================
//  .res file parser & rectangular axis deviation calculation
//
//  File format (4 lines per measurement point):
//    Line 1:  B0.000 C0.000              (machine angles, ignored)
//    Line 2:  I<ix> J<iy> K<iz>          (probe direction vector)
//    Line 3:  X<x> Y<y> Z<z>             (theoretical point coordinates)
//    Line 4:  X<x> Y<y> Z<z>             (actual measured point coordinates)
//
//  For each Z height, 8 points (2 per face, 4 faces) are fitted to a rectangle,
//  yielding the rectangle center.
//  Two Z heights per axis give two centers; their connecting line is the axis direction.
//  Finally, the angular deviation between actual and theoretical axes is computed.
// ============================================================

struct MeasurePoint
{
    Eigen::Vector3d ijk;        // probe direction
    Eigen::Vector3d theory;     // theoretical point
    Eigen::Vector3d actual;     // actual measured point
};

struct CircleFitResult
{
    Eigen::Vector3d center;     // circle center (in XY plane, Z = average)
    double          radius;     // radius
    double          z;          // Z height of this cross-section
};

struct TubeAxisResult
{
    Eigen::Vector3d center_lo;  // low-Z circle center
    Eigen::Vector3d center_hi;  // high-Z circle center
    Eigen::Vector3d direction;  // unit direction vector (from low to high)
    double          z_lo;
    double          z_hi;
};

struct AngularDeviation
{
    double angle_deg;           // angle between actual and theoretical axes (degrees)
    Eigen::Vector3d theory_dir; // theoretical direction (unit vector)
    Eigen::Vector3d actual_dir; // actual direction (unit vector)
};

struct RectFitResult
{
    Eigen::Vector2d center;   // rectangle center (XY plane)
    double z;                 // average Z
};

struct RectAxisResult
{
    Eigen::Vector3d center_lo;  // low-Z center
    Eigen::Vector3d center_hi;  // high-Z center
    Eigen::Vector3d direction;  // unit direction vector (from low to high)
    double          z_lo;
    double          z_hi;
};

// Build a line in the form ax + by = c from two points (unit normal form)
// Returns {a, b, c}
static Eigen::Vector3d lineFrom2Points(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
    Eigen::Vector2d d = p2 - p1;
    // Normal vector n = (-dy, dx), normalized
    Eigen::Vector2d n(-d.y(), d.x());
    n.normalize();
    double c = n.dot(p1);
    return Eigen::Vector3d(n.x(), n.y(), c);
}

// Intersect two lines in ax+by=c form
static Eigen::Vector2d lineIntersect(const Eigen::Vector3d& l1, const Eigen::Vector3d& l2)
{
    // | a1 b1 | | x |   | c1 |
    // | a2 b2 | | y | = | c2 |
    Eigen::Matrix2d A;
    A << l1.x(), l1.y(),
        l2.x(), l2.y();
    Eigen::Vector2d b(l1.z(), l2.z());
    return A.colPivHouseholderQr().solve(b);
}

// Fit a 2D line (ax + by = c, unit normal) to a set of points using least squares.
// Returns {a, b, c}.
static Eigen::Vector3d fitLine2D(const std::vector<Eigen::Vector2d>& pts)
{
    // Compute centroid
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (auto& p : pts) mean += p;
    mean /= (double)pts.size();

    // Build covariance matrix
    double sxx = 0, sxy = 0, syy = 0;
    for (auto& p : pts) {
        double dx = p.x() - mean.x(), dy = p.y() - mean.y();
        sxx += dx * dx;  sxy += dx * dy;  syy += dy * dy;
    }

    // The line direction is the eigenvector of the largest eigenvalue of [[sxx,sxy],[sxy,syy]]
    // Normal direction = eigenvector of smallest eigenvalue
    // Using 2x2 eigen decomposition analytically:
    double tr  = sxx + syy;
    double det = sxx * syy - sxy * sxy;
    double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
    // Smallest eigenvalue:
    double lam = tr / 2.0 - disc;

    // Eigenvector for smallest eigenvalue (= normal direction):
    Eigen::Vector2d n;
    if (std::abs(sxy) > 1e-12) {
        n = Eigen::Vector2d(lam - syy, sxy);
    } else {
        // Already diagonal: pick axis with smaller variance
        n = (sxx <= syy) ? Eigen::Vector2d(1, 0) : Eigen::Vector2d(0, 1);
    }
    n.normalize();

    double c = n.dot(mean);
    return Eigen::Vector3d(n.x(), n.y(), c);
}

// pts: 8 points on the faces of a rectangle (2 points per face, 4 faces).
// Point order is not guaranteed.
// Algorithm:
//   1. Compute the principal axis direction of all 8 points (PCA on XY).
//   2. Project each point onto that axis; split into two groups by median projection
//      — these correspond to the two pairs of parallel edges.
//   3. Within each group, further split by the secondary axis to get 4 edge point sets.
//   4. Fit a line to each edge point set; intersect adjacent edges to get 4 corners;
//      average the 4 corners to get the center.
//
// Simpler equivalent (robust): cluster the 8 points into two direction groups by PCA,
// fit one line per group (robust least-squares), intersect the two lines → center.
// But since the rectangle has 2 pairs of parallel edges we need 4 lines total.
//
// Implementation: use all-pairs line fitting with direction clustering.
inline bool fitRect2D(const std::vector<Eigen::Vector3d>& pts, RectFitResult& result)
{
    if (pts.size() < 8) return false;

    double zsum = 0.0;
    for (auto& p : pts) zsum += p.z();

    std::vector<Eigen::Vector2d> xy;
    xy.reserve(pts.size());
    for (auto& p : pts) {
        xy.push_back(Eigen::Vector2d(p.x(), p.y()));
    }

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (auto& p : xy) mean += p;
    mean /= (double)xy.size();

    double sxx = 0, sxy = 0, syy = 0;
    for (auto& p : xy) {
        double dx = p.x() - mean.x(), dy = p.y() - mean.y();
        sxx += dx * dx;  sxy += dx * dy;  syy += dy * dy;
    }

    double tr   = sxx + syy;
    double disc = std::sqrt(std::max(0.0, (sxx - syy) * (sxx - syy) / 4.0 + sxy * sxy));
    double lam1 = tr / 2.0 + disc;
    Eigen::Vector2d v1;
    if (std::abs(sxy) > 1e-12)
        v1 = Eigen::Vector2d(lam1 - syy, sxy).normalized();
    else
        v1 = (sxx >= syy) ? Eigen::Vector2d(1, 0) : Eigen::Vector2d(0, 1);
    Eigen::Vector2d v2(-v1.y(), v1.x());

    std::vector<double> proj1(xy.size()), proj2(xy.size());
    for (size_t i = 0; i < xy.size(); i++) {
        Eigen::Vector2d d = xy[i] - mean;
        proj1[i] = v1.dot(d);
        proj2[i] = v2.dot(d);
    }

    std::vector<Eigen::Vector2d> grpA_pos, grpA_neg, grpB_pos, grpB_neg;
    for (size_t i = 0; i < xy.size(); i++) {
        if (std::abs(proj2[i]) >= std::abs(proj1[i])) {
            if (proj2[i] >= 0) grpA_pos.push_back(xy[i]);
            else               grpA_neg.push_back(xy[i]);
        } else {
            if (proj1[i] >= 0) grpB_pos.push_back(xy[i]);
            else               grpB_neg.push_back(xy[i]);
        }
    }

    if (grpA_pos.empty() || grpA_neg.empty() || grpB_pos.empty() || grpB_neg.empty())
        return false;

    auto fitGrp = [](const std::vector<Eigen::Vector2d>& g) -> Eigen::Vector3d {
        if (g.size() == 1) {
            return Eigen::Vector3d(0, 0, 0);
        }
        return fitLine2D(g);
    };

    Eigen::Vector3d lineA_pos = fitGrp(grpA_pos);
    Eigen::Vector3d lineA_neg = fitGrp(grpA_neg);
    Eigen::Vector3d lineB_pos = fitGrp(grpB_pos);
    Eigen::Vector3d lineB_neg = fitGrp(grpB_neg);

    const double kDetThresh = 1e-6;
    Eigen::Vector2d centerSum = Eigen::Vector2d::Zero();
    int validCount = 0;

    auto intersect2 = [&](const Eigen::Vector3d& la, const Eigen::Vector3d& lb) {
        double det = la.x() * lb.y() - la.y() * lb.x();
        if (std::abs(det) < kDetThresh) return;
        Eigen::Matrix2d A;
        A << la.x(), la.y(), lb.x(), lb.y();
        Eigen::Vector2d rhs(la.z(), lb.z());
        centerSum += A.colPivHouseholderQr().solve(rhs);
        validCount++;
    };

    intersect2(lineA_pos, lineB_pos);
    intersect2(lineA_pos, lineB_neg);
    intersect2(lineA_neg, lineB_pos);
    intersect2(lineA_neg, lineB_neg);

    if (validCount == 0)
        return false;

    result.center = centerSum / (double)validCount;
    result.z = zsum / (double)pts.size();
    return true;
}

// ============================================================
//  Circle fitting: least-squares fit to N points in the same Z plane
// ============================================================
inline bool fitCircle2D(const std::vector<Eigen::Vector3d>& pts, CircleFitResult& result)
{
    int n = (int)pts.size();
    if (n < 3) return false;

    Eigen::MatrixXd M(n, 3);
    Eigen::VectorXd rhs(n);
    double zsum = 0.0;
    for (int i = 0; i < n; i++)
    {
        double x = pts[i].x(), y = pts[i].y();
        M(i, 0) = x;
        M(i, 1) = y;
        M(i, 2) = 1.0;
        rhs(i)  = x * x + y * y;
        zsum   += pts[i].z();
    }

    Eigen::Vector3d abc = M.colPivHouseholderQr().solve(rhs);

    double cx = abc(0) / 2.0;
    double cy = abc(1) / 2.0;
    double r  = std::sqrt(abc(2) + cx * cx + cy * cy);

    result.center = Eigen::Vector3d(cx, cy, zsum / n);
    result.radius = r;
    result.z      = zsum / n;
    return true;
}

// ============================================================
//  ResFileParser: parse .res format files and compute axis deviations
// ============================================================
class ResFileParser
{
public:
    // ---- Public data ----
    std::vector<MeasurePoint> points;   // all parsed measurement points
  std::vector<MeasurePoint> heightPoints;//Z方向
  std::vector<MeasurePoint> xPoints;//X方向

	// ---- Load file ----
	bool load(const std::string& filepath)
	{
		points.clear();
		heightPoints.clear();
		xPoints.clear();
		std::ifstream f(filepath);
		if (!f.is_open())
		{
			std::cerr << "[ResFileParser] Cannot open: " << filepath << "\n";
			return false;
		}

		std::vector<std::string> lines;
		std::string              line;
		while (std::getline(f, line))
		{
			std::string content = stripLineNumber(line);
			if (!content.empty())
				lines.push_back(content);
		}

		// Parse one measurement point per 4 lines
		size_t i = 0;
		for (i = 0; i + 3 < lines.size(); i += 4)
		{
			// Check if line 1 matches B... C... format
			if (lines[i].find('B') == std::string::npos && lines[i].find('I') == std::string::npos)
				continue;

			MeasurePoint mp;
			// Line 2: I J K
			if (!parseIJK(lines[i + 1], mp.ijk))
			{
				// Possibly line 1 is already IJK (variant format), skip
				continue;
			}
			// Line 3: theoretical point
			if (!parseXYZ(lines[i + 2], mp.theory))
				continue;
			// Line 4: actual point
			if (!parseXYZ(lines[i + 3], mp.actual))
				continue;

			points.push_back(mp);
		}

		if (points.size() >= 2)
		{
			xPoints.push_back(points.back());
			points.pop_back();
			xPoints.push_back(points.back());
			points.pop_back();
		}

		if (points.size() >= 2)
		{
			heightPoints.push_back(points.back());
			points.pop_back();
			heightPoints.push_back(points.back());
			points.pop_back();
		}

		std::cout << "[ResFileParser] Loaded " << points.size()
				<< " measure points, " << heightPoints.size()
				<< " height points from " << filepath << "\n";
		return !points.empty();
	}


    bool fitCircle(int start, int count, CircleFitResult& result, bool useActual = true) const
    {
        std::vector<Eigen::Vector3d> pts;
        for (int i = start; i < start + count && i < (int)points.size(); i++)
            pts.push_back(useActual ? points[i].actual : points[i].theory);
        return fitCircle2D(pts, result);
    }

    bool fitTubeAxis(int startLo, int startHi, int count, TubeAxisResult& result, bool useActual = true) const
    {
        CircleFitResult cLo, cHi;
        if (!fitCircle(startLo, count, cLo, useActual)) return false;
        if (!fitCircle(startHi, count, cHi, useActual)) return false;

        result.center_lo = cLo.center;
        result.center_hi = cHi.center;
        result.z_lo      = cLo.z;
        result.z_hi      = cHi.z;
        result.direction = (cHi.center - cLo.center).normalized();
        return true;
    }

    bool calcAngularDeviation(int startLo, int startHi, int count, AngularDeviation& result) const
    {
        TubeAxisResult actual, theory;
        if (!fitTubeAxis(startLo, startHi, count, actual, true)) return false;
        if (!fitTubeAxis(startLo, startHi, count, theory, false)) return false;

        result.actual_dir = actual.direction;
        result.theory_dir = theory.direction;

        double cosAngle = std::abs(result.actual_dir.dot(result.theory_dir));
        cosAngle = std::min(cosAngle, 1.0);
        result.angle_deg = std::acos(cosAngle) * 180.0 / M_PI;
        return true;
    }

    // ---- Print all measurement points ----
    void printPoints() const
    {
        std::cout << std::fixed;
        std::cout.precision(4);
        for (size_t i = 0; i < points.size(); i++)
        {
            const auto& p = points[i];
            std::cout << "[" << i << "] "
                      << "theory=(" << p.theory.x() << "," << p.theory.y() << "," << p.theory.z() << ")  "
                      << "actual=(" << p.actual.x() << "," << p.actual.y() << "," << p.actual.z() << ")  "
                      << "ijk=(" << p.ijk.x() << "," << p.ijk.y() << "," << p.ijk.z() << ")\n";
        }
    }

    bool fitRect(int start, RectFitResult& result, bool useActual = true) const
   {
        std::vector<Eigen::Vector3d> pts;
        for (int i = start; i < start + 8 && i < (int)points.size(); i++)
            pts.push_back(useActual ? points[i].actual : points[i].theory);
        return fitRect2D(pts, result);
    }

    bool fitRectAxis(int startLo, int startHi, RectAxisResult& result, bool useActual = true) const
    {
        RectFitResult rLo, rHi;
        if (!fitRect(startLo, rLo, useActual)) return false;
        if (!fitRect(startHi, rHi, useActual)) return false;

        result.center_lo = Eigen::Vector3d(rLo.center.x(), rLo.center.y(), rLo.z);
        result.center_hi = Eigen::Vector3d(rHi.center.x(), rHi.center.y(), rHi.z);
        result.z_lo      = rLo.z;
        result.z_hi      = rHi.z;
        result.direction = (result.center_hi - result.center_lo).normalized();
        return true;
    }

    // ---- Analyze all tubes automatically (standard format: 2 Z heights x groupSize points per tube) ----
    // groupSize: points per Z height (default 3)
    // tubeCount: number of tubes (-1 = auto-detect)
    // Assumed data layout:
    //   tube1 low-Z (groupSize pts), tube1 high-Z (groupSize pts),
    //   tube2 low-Z (groupSize pts), tube2 high-Z (groupSize pts), ...
    void analyzeAllTubes(int groupSize = 3, int tubeCount = -1) const
    {
        int totalGroups = (int)points.size() / groupSize;
        if (tubeCount < 0)
            tubeCount = totalGroups / 2;

        std::cout << "\n===== ResFileParser: Tube Axis Analysis =====\n";
        std::cout << "Total points: " << points.size()
                  << ", per group: " << groupSize
                  << ", tube count: " << tubeCount << "\n\n";

        for (int t = 0; t < tubeCount; t++)
        {
            int startLo = t * 2 * groupSize;
            int startHi = startLo + groupSize;
            if (startHi + groupSize > (int)points.size())
            {
                std::cout << "Tube " << t + 1 << ": insufficient data, skipped\n";
                continue;
            }

            TubeAxisResult actual, theory;
            if (!fitTubeAxis(startLo, startHi, groupSize, actual, true))
            {
                std::cout << "Tube " << t + 1 << ": failed to fit actual axis, skipped\n";
                continue;
            }
            if (!fitTubeAxis(startLo, startHi, groupSize, theory, false))
            {
                std::cout << "Tube " << t + 1 << ": failed to fit theory axis, skipped\n";
                continue;
            }

            double cosA = std::abs(actual.direction.dot(theory.direction));
            cosA = std::min(cosA, 1.0);
            double angleDeg = std::acos(cosA) * 180.0 / M_PI;

            std::cout << std::fixed;
            std::cout.precision(4);
            std::cout << "Tube " << t + 1 << ":\n";
            std::cout << "  theoretical axis direction: ("
                      << theory.direction.x() << ", "
                      << theory.direction.y() << ", "
                      << theory.direction.z() << ")\n";
            std::cout << "  actual axis direction:      ("
                      << actual.direction.x() << ", "
                      << actual.direction.y() << ", "
                      << actual.direction.z() << ")\n";
            std::cout << "  axis angular deviation: " << angleDeg << " deg\n";

            // Low-Z and high-Z center offsets
            Eigen::Vector3d dLo = actual.center_lo - theory.center_lo;
            Eigen::Vector3d dHi = actual.center_hi - theory.center_hi;
            std::cout << "  low-Z  center offset: dX=" << dLo.x()
                      << " dY=" << dLo.y() << " (norm=" << dLo.head<2>().norm() << " mm)\n";
            std::cout << "  high-Z center offset: dX=" << dHi.x()
                      << " dY=" << dHi.y() << " (norm=" << dHi.head<2>().norm() << " mm)\n\n";
        }
    }

private:
    static std::string stripLineNumber(const std::string& line)
    {
        std::string s = line;
        if (!s.empty() && s.back() == '\r') s.pop_back();

        static const std::regex lineNumberPattern(R"(^\s*\d+[\xe2\x86\x92>-]\s*)");
        std::string result = std::regex_replace(s, lineNumberPattern, "");
        return trim(result);
    }

    static std::string trim(const std::string& s)
    {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    static bool parseIJK(const std::string& s, Eigen::Vector3d& v)
    {
        static const std::regex ijkPattern(R"(I([+-]?[0-9]*\.?[0-9]+)\s+J([+-]?[0-9]*\.?[0-9]+)\s+K([+-]?[0-9]*\.?[0-9]+))");
        std::smatch match;
        if (std::regex_search(s, match, ijkPattern))
        {
            v = Eigen::Vector3d(
                std::stod(match[1].str()),
                std::stod(match[2].str()),
                std::stod(match[3].str())
            );
            return true;
        }
        return false;
    }

    static bool parseXYZ(const std::string& s, Eigen::Vector3d& v)
    {
        static const std::regex xyzPattern(R"(X([+-]?[0-9]*\.?[0-9]+)\s+Y([+-]?[0-9]*\.?[0-9]+)\s+Z([+-]?[0-9]*\.?[0-9]+))");
        std::smatch match;
        if (std::regex_search(s, match, xyzPattern))
        {
            v = Eigen::Vector3d(
                std::stod(match[1].str()),
                std::stod(match[2].str()),
                std::stod(match[3].str())
            );
            return true;
        }
        return false;
    }
};
