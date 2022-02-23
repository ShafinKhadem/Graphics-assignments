//#undef DEBUG
#include <bits/stdc++.h>
using namespace std;
using ll = long long;
const ll llinf = (1ll<<61)-1;
#define sz(a) int(a.size())
#define all(x) begin(x), end(x)
#ifdef DEBUG
const int DEBUG_END = 26;
#define DOS cout
#include <debug.h>
#else
#define bug(args...) void()
#define cbug(a, args...)
#endif
#define ASSERT(a, o, b, args...) if (!((a)o(b))) bug(a, b, ##args), assert((a)o(b));
#define fi first
#define se second
int TC = 1, CN;
const int inf = 1e6;
#include "bitmap_image.hpp"

const double pi = acos(-1);

class point {
public:
    double x, y, z;
    friend istream &operator>>(istream &is, point &rs) { return is >> rs.x >> rs.y >> rs.z; }
    friend ostream &operator<<(ostream &os, const point &rs) { return os << rs.x << ' ' << rs.y << ' ' << rs.z; }
    bool operator <(const point &rs) const { return tie(x,y,z)<tie(rs.x,rs.y,rs.z); }
    point operator -() const { return point{-x,-y,-z}; }
    point operator +(const point &rs) const { return point{x+rs.x,y+rs.y,z+rs.z}; }
    point& operator +=(const point &rs) { return *this = *this + rs; }
    point operator -(const point &rs) const { return point{x-rs.x,y-rs.y,z-rs.z}; }
    #define TCF template<class F, class = enable_if_t<is_arithmetic<F>::value, F>>
    TCF auto operator +(const F &rs) { return point{x+rs, y+rs, z+rs}; }
    TCF auto operator -(const F &rs) { return point{x-rs, y-rs, z-rs}; }
    TCF auto operator *(const F &rs) { return point{x*rs, y*rs, z*rs}; }
    TCF auto operator /(const F &rs) { return point{x/rs, y/rs, z/rs}; }
    #undef TCF

    double dot(const point &rs) { return x*rs.x + y*rs.y + z*rs.z; }

    point cross(const point &rs) {
        return {y * rs.z - z * rs.y, z * rs.x - x * rs.z, x * rs.y - y * rs.x};
    }

    double norm() { return sqrt(dot(*this)); }

    point& normalize() { return *this = *this / norm(); }

    void rotate(const point &axisOrigin, point axisDir, const double &theta, bool thisIsAxis = 0) {
        axisDir.normalize();
        double a = axisOrigin.x, b = axisOrigin.y, c = axisOrigin.z, u = axisDir.x, v = axisDir.y, w = axisDir.z;
        double sinn = sin(theta), coss = cos(theta), dirdot = dot(axisDir);
        *this = {
            (1-coss)*(a*(v*v+w*w)-u*(b*v+c*w-dirdot)) + coss*x + sinn*(-c*v+b*w-w*y+v*z),
            (1-coss)*(b*(u*u+w*w)-v*(a*u+c*w-dirdot)) + coss*y + sinn*(c*u-a*w+w*x-u*z),
            (1-coss)*(c*(u*u+v*v)-w*(a*u+b*v-dirdot)) + coss*z + sinn*(-b*u+a*v-v*x+u*y)
        };
        if (thisIsAxis) normalize();    // we are rotating axis represented by point, hence unit vector
    }
};

struct Line {
    point p, p2;
    friend ostream& operator <<(ostream &os, const Line &rs) {
        return os<<"{"<<"p: "<<rs.p<<", "<<"p2: "<<rs.p2<<"}";
    }
    friend istream& operator >>(istream &is, Line &rs) {
        return is>>rs.p>>rs.p2;
    }

    point intersectPlane(const double &a, const double &b, const double &c, const double &d) {
        // ax + by + cz + d == 0
        point v = p2-p;
        // t*(v.dot({a, b, c})) + p.dot({a, b, c}) + d == 0
        double vdot = v.dot({a, b, c});
        if (abs(vdot) < 1e-6) return {inf, inf, inf};
        double t = -(p.dot({a, b, c}) + d)/vdot;
        return p + v*t;
    }
};

#define deg(x) (pi*(x)/180.0)
#define rad(x) (180.0*(x)/pi)

using Matrix = array<array<double, 4>, 4>;

Matrix operator *(const Matrix &ls, const Matrix &rs) {
    Matrix ret;
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
        ret[i][j] = 0;
        for (int k = 0; k < 4; k++) ret[i][j] += ls[i][k]*rs[k][j];
    }
    return ret;
}

point operator *(const point &ls, const Matrix &rs) {
    array<double, 4> res, p = {ls.x, ls.y, ls.z, 1};
    for (int i = 0; i < 4; ++i) {
        res[i] = 0;
        for (int k = 0; k < 4; ++k) { res[i] += rs[i][k]*p[k]; }
    }
    return point{res[0]/res[3], res[1]/res[3], res[2]/res[3]};
}

template<class C, class D> C& operator *=(C &x, const D &y) { return x = x*y; }

signed main(int argc, char **argv) {
    if (argc < 2) {
        cout << "Please give location of scene.txt as command line argument\n";
        exit(1);
    }

    ifstream ifs(string(argv[1])+"/scene.txt");
    if (ifs.fail()) {
        cout << "Couldn't open: " << argv[1] << "/scene.txt\n";
        exit(1);
    }
    ifs.exceptions(ifs.failbit);
    ofstream ofs1("stage1.txt"), ofs2("stage2.txt"), ofs3("stage3.txt");
    ofs1.precision(7), ofs2.precision(7), ofs3.precision(7);
    ofs1.setf(ios::fixed), ofs2.setf(ios::fixed), ofs3.setf(ios::fixed);

    point eye, look, up;
    double fovY, aspectRatio, near, far, fovX;
    ifs >> eye >> look >> up >> fovY >> aspectRatio >> near >> far;
    fovX = fovY * aspectRatio;

    Matrix P = {
        1/tan(deg(fovX)/2), 0, 0, 0,
        0, 1/tan(deg(fovY)/2), 0, 0,
        0, 0, -(far+near)/(far-near), -(2*far*near)/(far-near),
        0, 0, -1, 0
    };

    auto l = look-eye;
    l.normalize();
    auto r = l.cross(up);
    r.normalize();
    auto u = r.cross(l);
    Matrix R = {
        r.x, r.y, r.z, 0,
        u.x, u.y, u.z, 0,
        -l.x, -l.y, -l.z, 0,
        0, 0, 0, 1
    };

    Matrix T = {
        1, 0, 0, -eye.x,
        0, 1, 0, -eye.y,
        0, 0, 1, -eye.z,
        0, 0, 0, 1
    };

    Matrix V = R*T;

    stack<Matrix> stak;
    Matrix m = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    struct Triangle
    {
        array<point, 3> points;
        array<unsigned char, 3> color;
    };

    vector<Triangle> stage3triangles;

    for (string command; ifs >> command and command != "end"; ) {
        if (command == "triangle") {
            point p1, p2, p3;
            ifs >> p1 >> p2 >> p3;
            p1 *= m, p2 *= m, p3 *= m;
            ofs1 << p1 << '\n' << p2 << '\n' << p3 << '\n' << '\n';
            p1 *= V, p2 *= V, p3 *= V;
            ofs2 << p1 << '\n' << p2 << '\n' << p3 << '\n' << '\n';
            p1 *= P, p2 *= P, p3 *= P;
            stage3triangles.push_back({{p1, p2, p3}, {}});
            ofs3 << p1 << '\n' << p2 << '\n' << p3 << '\n' << '\n';
        } else if (command == "translate") {
            point t;
            ifs >> t;
            m *= Matrix {
                1, 0, 0, t.x,
                0, 1, 0, t.y,
                0, 0, 1, t.z,
                0, 0, 0, 1
            };
        } else if (command == "scale") {
            point s;
            ifs >> s;
            m *= Matrix {
                s.x, 0, 0, 0,
                0, s.y, 0, 0,
                0, 0, s.z, 0,
                0, 0, 0, 1
            };
        } else if (command == "rotate") {
            double ad;
            point a;
            ifs >> ad >> a;
            a.normalize();
            point c1 = {1, 0, 0}, c2 = {0, 1, 0}, c3 = {0, 0, 1};
            c1.rotate({}, a, deg(ad), 1);
            c2.rotate({}, a, deg(ad), 1);
            c3.rotate({}, a, deg(ad), 1);
            m *= Matrix {
                c1.x, c2.x, c3.x, 0,
                c1.y, c2.y, c3.y, 0,
                c1.z, c2.z, c3.z, 0,
                0, 0, 0, 1
            };
        } else if (command == "push") {
            stak.push(m);
        } else if (command == "pop") {
            if (stak.empty()) {
                cout << "Stack is empty! Popping isn't allowed";
                exit(1);
            }
            m = stak.top(); stak.pop();
        }
    }

    ifs.close();
    ofs1.close(), ofs2.close(), ofs3.close();

    ifstream ifc(string(argv[1])+"/config.txt");
    if (ifc.fail()) {
        cout << "Couldn't open: " << argv[1] << "/config.txt\n";
        exit(1);
    }
    ifc.exceptions(ifc.failbit);
    int width, height;
    double minx, maxx, miny, maxy, minz, maxz;
    ifc >> width >> height >> minx >> miny >> minz >> maxz;
    maxx = -minx, maxy = -miny;
    ifc.close();

    double dx = (maxx-minx)/width, dy = (maxy-miny)/height, topy = maxy-dy/2, leftx = minx+dx/2, bottomy = miny+dy/2, rightx = maxx-dx/2;

    for (auto &i : stage3triangles) {
        i.color = {(unsigned char)(rand()%255), (unsigned char)(rand()%255), (unsigned char)(rand()%255)};
    }

    vector<vector<double> > z_buffer(height, vector<double>(width, maxz));
    vector<vector<array<unsigned char, 3>> > pixels(height, vector<array<unsigned char, 3>>(width));
    for (auto &[t, c] : stage3triangles) {
        auto cmpy = [](auto ls, auto rs){return ls.y < rs.y; };
        double tminy = min_element(all(t), cmpy)->y, tmaxy = max_element(all(t), cmpy)->y;
        int topRow = int(round((topy-min(tmaxy, topy))/dy)+1e-6), bottomRow = int(round((topy-max(tminy, bottomy))/dy)+1e-6);
        vector<Line> edges;
        for (int j = 0, szt = sz(t); j < szt; j++) {
            edges.push_back({t[j], t[j == szt-1 ? 0 : j+1]});
        }
        for (int i = topRow; i <= bottomRow; i++) {
            double ys = topy-i*dy;
            point tmin = {inf, 0, 0}, tmax = {-inf, 0, 0};
            for (auto &j : edges) {
                auto p = j.intersectPlane(0, 1, 0, -ys);
                if (p.x > min(j.p.x, j.p2.x)-1e-6 and p.x < max(j.p.x, j.p2.x)+1e-6) tmin = min(p, tmin), tmax = max(p, tmax);
            }
            double xa = tmin.x, xb = tmax.x, za = tmin.z, zb = tmax.z;
            if (xb > xa+1e-6) {
                const double slope = (zb-za)/(xb-xa);
                int leftCol = int(round((max(xa, leftx)-leftx)/dx)+1e-6), rightCol = int(round((min(xb, rightx)-leftx)/dx)+1e-6);
                for (int j = leftCol; j <= rightCol; j++) {
                    double xp = leftx+j*dx, zp = zb-(xb-xp)*slope;
                    if (zp < z_buffer[i][j] and zp >= minz) {
                        z_buffer[i][j] = zp;
                        pixels[i][j] = c;
                    }
                }
            }
        }
    }


    ofstream ofsz("z_buffer.txt");
    ofsz.setf(ios::fixed);
    bitmap_image image(width,height);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (z_buffer[i][j] < maxz) ofsz << z_buffer[i][j] << '\t';
            image.set_pixel(j, i, pixels[i][j][0], pixels[i][j][1], pixels[i][j][2]);
        }
        ofsz << '\n';
    }
    ofsz.close();
    image.save_image("out.bmp");
}