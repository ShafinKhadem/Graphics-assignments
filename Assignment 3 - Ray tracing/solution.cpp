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

#if defined(_WIN32) && !defined(APIENTRY) && !defined(__CYGWIN__)
#include <windows.h>
#include <glut.h>
#else
#include <GL/glut.h>
#endif

#define deg(x) (pi*(x)/180.0)
#define rad(x) (180.0*(x)/pi)
#define vertex(p) glVertex3d(p.x,p.y,p.z)

bool bugged = 0;
const int FOV = 80, windowHeight = 500, windowWidth = 500;
const double pi = acos(-1), eps = 1e-6;
int recursionLevel, pixel;

pair<double, double> quadratic(double a, double b, double c) {  // a*x*x + b*x + c = 0
    double d = b*b - 4*a*c;
    if (d < 0) return {inf, inf};
    double t0 = (-b - sqrt(d)) / (2*a), t1 = (-b + sqrt(d)) / (2*a);
    return {t0, t1};
}

class point {   // axis / vector can be represented by it using axisOrigin={} and ensuring normalization always
public:
    double x, y, z;
    friend istream &operator>>(istream &is, point &rs) { return is >> rs.x >> rs.y >> rs.z; }
    friend ostream &operator<<(ostream &os, const point &rs) { return os << '{' << rs.x << ',' << rs.y << ',' << rs.z << '}'; }
    bool operator <(const point &rs) const { return tie(x,y,z)<tie(rs.x,rs.y,rs.z); }
    point operator -() const { return point{-x,-y,-z}; }
    point operator +(const point &rs) const { return point{x+rs.x,y+rs.y,z+rs.z}; }
    point operator *(const point &rs) const { return point{x*rs.x,y*rs.y,z*rs.z}; }
    point& operator +=(const point &rs) { return *this = *this + rs; }
    point operator -(const point &rs) const { return point{x-rs.x,y-rs.y,z-rs.z}; }
    #define TCF template<class F, class = enable_if_t<is_arithmetic<F>::value, F>>
    TCF auto operator +(const F &rs) { return point{x+rs, y+rs, z+rs}; }
    TCF auto operator -(const F &rs) { return point{x-rs, y-rs, z-rs}; }
    TCF auto operator *(const F &rs) { return point{x*rs, y*rs, z*rs}; }
    TCF auto operator /(const F &rs) { return point{x/rs, y/rs, z/rs}; }
    #undef TCF

    double dot(const point &rs) { return x*rs.x + y*rs.y + z*rs.z; }

    point cross(const point &rs, bool normalizeResult = 0) {
        point ret = {y * rs.z - z * rs.y, z * rs.x - x * rs.z, x * rs.y - y * rs.x};
        if (normalizeResult) ret.normalize();
        return ret;
    }

    double norm() { return sqrt(dot(*this)); }

    point& normalize() { return *this = *this / norm(); }

    void rotate(const point &axisOrigin /* should be {} for vector */, point axisDir, const double &theta) {
        // simulation: https://twist-and-shout.appspot.com/
        axisDir.normalize();
        auto [a, b, c] = axisOrigin;
        auto [u, v, w] = axisDir;
        double sinn = sin(theta), coss = cos(theta), dirdot = dot(axisDir);
        *this = {
            (1-coss)*(a*(v*v+w*w)-u*(b*v+c*w-dirdot)) + coss*x + sinn*(-c*v+b*w-w*y+v*z),
            (1-coss)*(b*(u*u+w*w)-v*(a*u+c*w-dirdot)) + coss*y + sinn*(c*u-a*w+w*x-u*z),
            (1-coss)*(c*(u*u+v*v)-w*(a*u+b*v-dirdot)) + coss*z + sinn*(-b*u+a*v-v*x+u*y)
        };
    }
} pos, u, r, l;

struct Ray {
    point p, dir;
    friend ostream& operator <<(ostream &os, const Ray &rs) {
        return os<<"{"<<"p: "<<rs.p<<", "<<"dir: "<<rs.dir<<"}";
    }
    friend istream& operator >>(istream &is, Ray &rs) {
        return is>>rs.p>>rs.dir;
    }

    Ray(){}
    Ray(point start, point direction) : p(start), dir(direction) {
        dir.normalize();
    }

    point pointAt(double t) {
        return p+dir*t;
    }

    double intersectPlane(const double &a, const double &b, const double &c, const double &d) {
        // ax + by + cz + d == 0
        // t*(dir.dot({a, b, c})) + p.dot({a, b, c}) + d == 0
        double vdot = dir.dot({a, b, c});
        if (abs(vdot) < 1e-6) return inf;
        double t = -(p.dot({a, b, c}) + d)/vdot;
        return t;   // pointAt(t) is the intersection point
    }
};

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


void drawLine(point lineStart, point lineEnd) {
    glBegin(GL_LINES);{
        vertex(lineStart);
        vertex(lineEnd);
    }glEnd();
}
void drawPoint(double x, double y, double z, double size) {
    glPointSize(size);
    glBegin(GL_POINTS);{
        glVertex3d(x, y, z);
    }glEnd();
    glPointSize(1);
}
void drawQuad(point topLeft, point topRight, point bottomRight, point bottomLeft) {
    glBegin(GL_QUADS);{
        vertex(topLeft);
        vertex(topRight);
        vertex(bottomRight);
        vertex(bottomLeft);
    }glEnd();
}

struct Light
{
    point pos, color;
    friend ostream& operator <<(ostream &os, const Light &rs) {
        return os<<"{"<<"pos: "<<rs.pos<<", "<<"color: "<<rs.color<<"}";
    }
    friend istream& operator >>(istream &is, Light &rs) {
        return is>>rs.pos>>rs.color;
    }

    void draw() {
        glColor3d(color.x, color.y, color.z);
        drawPoint(pos.x, pos.y, pos.z, 7);
    }
};
vector<Light> lights;

struct ReflectionCoeff
{
    double ambient, diffuse, specular, recursive;
    friend ostream& operator <<(ostream &os, const ReflectionCoeff &rs) {
        return os<<"{"<<"ambient: "<<rs.ambient<<", "<<"diffuse: "<<rs.diffuse<<", "<<"specular: "<<rs.specular<<", "<<"recursive: "<<rs.recursive<<"}";
    }
    friend istream& operator >>(istream &is, ReflectionCoeff &rs) {
        return is>>rs.ambient>>rs.diffuse>>rs.specular>>rs.recursive;
    }
};

struct Object{
    point color;
    ReflectionCoeff coeff;
    int shine; // exponent term of specular component
    virtual void draw() = 0;
    virtual point getColor(point ip) {
        return this->color;
    }
    virtual point normal(point qp) = 0;
    virtual double intersectingT(Ray r) = 0;
    friend ostream& operator <<(ostream &os, const Object &rs) {
        return os<<"{"<<"color: "<<rs.color<<", "<<"coeff: "<<rs.coeff<<", "<<"shine: "<<rs.shine<<"}";
    }
    friend istream& operator >>(istream &is, Object &rs) {
        return is>>rs.color>>rs.coeff>>rs.shine;
    }

    point intersectColor(Ray r, int level);
};
vector<Object*> objects;


double getNearest(Ray r) {
    int ret = -1;
    double t_min = inf;
    for(int i = 0; i < size(objects); i++) {
        double t = objects[i]->intersectingT(r);
        if (t > 0 and t < t_min) {
            t_min = t;
            ret = i;
        }
    }
    return ret;
}

point Object::intersectColor(Ray v, int level) {
    double t = intersectingT(v);
    if (level == 0 or t > inf-eps or t <= 0) return {};

    point vrev = -v.dir, ip = v.pointAt(t), n = normal(ip), intersectionPointColor = getColor(ip);

    /*
    diffuse and ambient coefficient depends on surface color, we are inputting one base coefficient and multiplying it
    with intersectionPointColor to get/approximate final coefficient.
    */
    point color = intersectionPointColor*coeff.ambient; // assuming there is a white (1,1,1) ambient light

    for (auto &light : lights) {
        Ray rayl = {light.pos, ip-light.pos};
        double dist = (ip-light.pos).norm();
        bool obscured = 0;
        for (auto object : objects) {
            double t = object->intersectingT(rayl);
            if (t > 0 and t < dist-eps) {
                obscured = 1;
                break;
            }
        }
        if (!obscured) {
            point lrev = -rayl.dir, r = (n*2*lrev.dot(n)-lrev).normalize();
            double lambertValue = max(0.0, lrev.dot(n)), phongValue = max(0.0, r.dot(vrev));
            color += light.color*intersectionPointColor*coeff.diffuse*lambertValue;
            #warning specular component should not depend on intersectionPointColor but done according to offline spec.
            color += light.color*intersectionPointColor*coeff.specular*pow(phongValue, shine);
        }
    }

    if (level < recursionLevel) {
        Ray vReflect = {ip, {n*2*vrev.dot(n)-vrev}};
        // moving the start a little bit to avoid self intersection
        vReflect.p += vReflect.dir*eps;
        int nearest = getNearest(vReflect);
        if (nearest != -1) {
            point colorReflect = objects[nearest]->intersectColor(vReflect, level+1);
            color += colorReflect*coeff.recursive;
        }
    }

    color.x = clamp(color.x, 0.0, 1.0), color.y = clamp(color.y, 0.0, 1.0), color.z = clamp(color.z, 0.0, 1.0);

    return color;
}


struct Sphere : public Object
{
    point center;
    double radius;
    friend ostream& operator <<(ostream &os, const Sphere &rs) {
        return os<<"{"<<"center: "<<rs.center<<", "<<"radius: "<<rs.radius<<", "<<"color: "<<rs.color<<", "<<"coeff: "<<rs.coeff<<", "<<"shine: "<<rs.shine<<"}";
    }
    friend istream& operator >>(istream &is, Sphere &rs) {
        return is>>rs.center>>rs.radius>>rs.color>>rs.coeff>>rs.shine;
    }

    void draw()
    {
        glColor3d(color.x, color.y, color.z);
        glTranslated(center.x, center.y, center.z);

        int slices = 30, stacks = 30;
        struct point points[100][100];
        int i,j;
        double h,rr;
        //generate points
        for(i=0;i<=stacks;i++)
        {
            h=radius*sin(((double)i/(double)stacks)*(pi/2));
            rr=radius*cos(((double)i/(double)stacks)*(pi/2));
            for(j=0;j<=slices;j++)
            {
                points[i][j].x=rr*cos(((double)j/(double)slices)*2*pi);
                points[i][j].y=rr*sin(((double)j/(double)slices)*2*pi);
                points[i][j].z=h;
            }
        }
        //draw quads using generated points
        for(i=0;i<stacks;i++)
        {
            for(j=0;j<slices;j++)
            {
                glBegin(GL_QUADS);{
                    //upper hemisphere
                    glVertex3d(points[i][j].x,points[i][j].y,points[i][j].z);
                    glVertex3d(points[i][j+1].x,points[i][j+1].y,points[i][j+1].z);
                    glVertex3d(points[i+1][j+1].x,points[i+1][j+1].y,points[i+1][j+1].z);
                    glVertex3d(points[i+1][j].x,points[i+1][j].y,points[i+1][j].z);
                    //lower hemisphere
                    glVertex3d(points[i][j].x,points[i][j].y,-points[i][j].z);
                    glVertex3d(points[i][j+1].x,points[i][j+1].y,-points[i][j+1].z);
                    glVertex3d(points[i+1][j+1].x,points[i+1][j+1].y,-points[i+1][j+1].z);
                    glVertex3d(points[i+1][j].x,points[i+1][j].y,-points[i+1][j].z);
                }glEnd();
            }
        }
        glTranslated(-center.x, -center.y, -center.z);
    }

    point normal(point qp) { return (qp-center).normalize(); }

    double intersectingT(Ray r) {
        point p = r.p-center;

        double a = r.dir.dot(r.dir), b = 2*r.dir.dot(p), c = p.dot(p) - radius*radius;

        auto [t0, t1] = quadratic(a, b, c);
        return t0 > 0 ? t0 : t1;
    }
};

struct Triangle : public Object
{
    point a, b, c;
    friend ostream& operator <<(ostream &os, const Triangle &rs) {
        return os<<"{"<<"a: "<<rs.a<<", "<<"b: "<<rs.b<<", "<<"c: "<<rs.c<<", "<<"color: "<<rs.color<<", "<<"coeff: "<<rs.coeff<<", "<<"shine: "<<rs.shine<<"}";
    }
    friend istream& operator >>(istream &is, Triangle &rs) {
        return is>>rs.a>>rs.b>>rs.c>>rs.color>>rs.coeff>>rs.shine;
    }

    void draw(){
        glColor3d(color.x, color.y, color.z);

        glBegin(GL_TRIANGLES);{
            glVertex3d(a.x, a.y, a.z);
            glVertex3d(b.x, b.y, b.z);
            glVertex3d(c.x, c.y, c.z);
        }glEnd();
    }

    point normal(point qp) {
        point u = b-a, v = c-a, n = u.cross(v, 1);
        //UNSURE
        return (pos-a).dot(n) < 0 ? -n : n;
    }

    double intersectingT(Ray r) {
        point n = normal(a);
        double t = r.intersectPlane(n.x, n.y, n.z, -n.dot(a));
        point ip = r.pointAt(t);
        point edge0 = b - a, edge1 = c - b, edge2 = a - c, vp0 = ip - a, vp1 = ip - b, vp2 = ip - c;
        // for any point inside triangle, edgei.cross(vpi) will be the same direction as n
        if (n.dot(edge0.cross(vp0)) > 0 and
            n.dot(edge1.cross(vp1)) > 0 and
            n.dot(edge2.cross(vp2)) > 0) return t;
        else return inf;
    }
};

struct General : public Object
{
    double a, b, c, d, e, f, g, h, i, j;
    point ref;
    double length, width, height;
    friend ostream& operator <<(ostream &os, const General &rs) {
        return os<<"{"<<"a: "<<rs.a<<", "<<"b: "<<rs.b<<", "<<"c: "<<rs.c<<", "<<"d: "<<rs.d<<", "<<"e: "<<rs.e<<", "<<"f: "<<rs.f<<", "<<"g: "<<rs.g<<", "<<"h: "<<rs.h<<", "<<"i: "<<rs.i<<", "<<"j: "<<rs.j<<", "<<"ref: "<<rs.ref<<", "<<"length: "<<rs.length<<", "<<"width: "<<rs.width<<", "<<"height: "<<rs.height<<", "<<"color: "<<rs.color<<", "<<"coeff: "<<rs.coeff<<", "<<"shine: "<<rs.shine<<"}";
    }
    friend istream& operator >>(istream &is, General &rs) {
        return is>>rs.a>>rs.b>>rs.c>>rs.d>>rs.e>>rs.f>>rs.g>>rs.h>>rs.i>>rs.j>>rs.ref>>rs.length>>rs.width>>rs.height>>rs.color>>rs.coeff>>rs.shine;
    }

    bool inBound(point p) {
        return (length == 0 or (p.x == clamp(p.x, -length, +length))) and (width == 0 or (p.y == clamp(p.y, -width, +width))) and (height == 0 or (p.z == clamp(p.z, -height, +height)));
    }

    void draw() {
        // auto [x, y, z] = ref;
        // drawPoint(x, y, z, 50);
    }

    point normal(point qp) {
        auto [x, y, z] = qp-ref;
        return point{2*a*x + d*y + e*z + g, 2*b*y + d*x + f*z + h, 2*c*z + e*x + f*y + i}.normalize();
    }

    double intersectingT(Ray r) {
        auto [p, dir] = r;
        p = p-ref;
        point abc = {a, b, c}, dfe = {d, f, e}, ghi = {g, h, i};
        auto pp = p*p, dd = dir*dir;
        double aa = dd.dot(abc) + (dir*dfe).dot({dir.y, dir.z, dir.x});
        double bb = 2*p.dot(abc*dir) + (dir*dfe).dot({p.y, p.z, p.x}) + (p*dfe).dot({dir.y, dir.z, dir.x}) + dir.dot(ghi);
        double cc = pp.dot(abc) + (p*dfe).dot({p.y, p.z, p.x}) + p.dot(ghi) + j;

        auto [t0, t1] = quadratic(aa, bb, cc);
        auto p0 = r.pointAt(t0), p1 = r.pointAt(t1);
        return t0 > 0 and inBound(p0) ? t0 : (inBound(p1) ? t1 : inf);
    }
};

vector<Sphere*> spheres;
vector<Triangle*> triangles;
vector<General*> generals;

struct Floor : public Object
{
    double floorWidth, tileWidth;
    friend ostream& operator <<(ostream &os, const Floor &rs) {
        return os<<"{"<<"floorWidth: "<<rs.floorWidth<<", "<<"tileWidth: "<<rs.tileWidth<<", "<<"color: "<<rs.color<<", "<<"coeff: "<<rs.coeff<<", "<<"shine: "<<rs.shine<<"}";
    }

    void draw(){
        auto drawSquare = [&](double a)->void {
            a /= 2;
            glBegin(GL_QUADS);{
                glVertex3d( a,  a, 0);
                glVertex3d( a, -a, 0);
                glVertex3d(-a, -a, 0);
                glVertex3d(-a,  a, 0);
            }glEnd();
        };
        for (int i = -floorWidth/2, rowcolor = 1, row = 0; i < floorWidth/2; i+=tileWidth, rowcolor = 1-rowcolor, ++row) {
            for (int j = -floorWidth/2, color = rowcolor, col = 0; j < floorWidth/2; j+=tileWidth, color = 1-color, ++col) {
                glTranslated(j+tileWidth/2, i+tileWidth/2, 0);
                glColor3d(color, color, color);
                // int cntTile = floorWidth/tileWidth;
                // if (max(abs(row-cntTile/2), abs(col-cntTile/2)) < 1) glColor3d(1, 0, 0);
                drawSquare(tileWidth);
                glTranslated(-(j+tileWidth/2), -(i+tileWidth/2), 0);
            }
        }
    }

    point getColor(point ip) {
        int cntTile = floorWidth/tileWidth, row = (ip.x+floorWidth/2)/tileWidth, col = (ip.y+floorWidth/2)/tileWidth;
        double tmp = (row&1)^(col&1)^1;
        color = {tmp, tmp, tmp};//bug(ip, cntTile, row, col, color);
        return color;
    }

    point normal(point qp) { return {0, 0, pos.z < 0.0 ? -1.0 : 1.0}; }

    double intersectingT(Ray r) {
        double t = r.intersectPlane(0, 0, 1, 0);
        point ip = r.pointAt(t);
        return max(abs(ip.x), abs(ip.y)) > floorWidth/2-eps ? inf : t;
    }
};



void capture() {
    cout << "capturing...";
    cout.flush();
    double width = pixel, height = pixel;
    bitmap_image image(width,height);

    double planeDistance = (windowHeight/2.0)/tan(deg(FOV)/2.0);
    double du = double(windowWidth)/width, dv = double(windowHeight)/height;
    point topLeft = pos + l*planeDistance - r*windowWidth/2 + u*windowHeight/2 + r*.5*du - u*.5*dv;

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            point curPixel = topLeft+r*j*du-u*i*dv;
            Ray ray(pos, curPixel-pos);

            int nearest = getNearest(ray);

            if (nearest != -1) {
                point color = objects[nearest]->intersectColor(ray, 1);
                auto [r, g, b] = color*255;
                image.set_pixel(j, i, r, g, b);
            } else {
                image.set_pixel(j, i, 0, 0, 0);
            }
        }
    }
    image.save_image("out.bmp");
    cout << "done" << endl;
}


const double movespeed = 2, rotatespeed = deg(3);
double cameraHeight, cameraAngle;
int drawgrid, drawaxes;
void drawAxes(double cr = 1, double cg = 1, double cb = 1) {
    double rgba[4];
    glGetDoublev(GL_CURRENT_COLOR, rgba);
    glColor3d(cr, cg, cb);
    drawLine({200, 0, 0}, {-200, 0, 0});
    drawLine({0, -200, 0}, {0, 200, 0});
    drawLine({0, 0, 200}, {0, 0, -200});
    drawPoint(0, 0, 0, 15);
    glColor3dv(rgba);
}
void drawGrid() {
    double rgba[4];
    glGetDoublev(GL_CURRENT_COLOR, rgba);
    glBegin(GL_LINES);{
        for(int i=-18;i<=18;i++){
            if(i==0)
                continue;   //SKIP the MAIN axes

            glColor3d(0.6, 0.4, 0.4);   //redish
            //lines parallel to Y-axis
            glVertex3d(i*10, -190, 0);
            glVertex3d(i*10,  190, 0);

            glColor3d(0.4, 0.6, 0.4);   //greenish
            //lines parallel to X-axis
            glVertex3d(-190, i*10, 0);
            glVertex3d( 190, i*10, 0);
        }
    }glEnd();
    glColor3dv(rgba);
}

void setupCamera(){
    pos = {0, -100, 50}, u = {0, 0, 1}, r = {1, 0, 0}, l = {0, 1, 0};
    // bug(pos, l, r, u);
}

void drawSS()
{
    for (auto &i : objects) {
        i->draw();
    }

    for (auto &i : lights) {
        i.draw();
    }
}

string inp;
void keyboardListener(unsigned char key, int x,int y){
    switch(key){
        case 13:
            bug(inp);
            if (sz(inp)==2) {
                inp.push_back('x'^'y'^'z'^inp[0]^inp[1]);
                pos = {150.0*(inp[0] == 'x'), 150.0*(inp[0] == 'y'), 150.0*(inp[0] == 'z')};
                l = {-1.0*(inp[0] == 'x'), -1.0*(inp[0] == 'y'), -1.0*(inp[0] == 'z')};
                r = {1.0*(inp[1] == 'x'), 1.0*(inp[1] == 'y'), 1.0*(inp[1] == 'z')};
                u = {1.0*(inp[2] == 'x'), 1.0*(inp[2] == 'y'), 1.0*(inp[2] == 'z')};
            } else {
                setupCamera();
            }
            inp = "";
            break;
        case '-':
            pos = {-pos.x, -pos.y, pos.z}, l = -l, r = -r;
            break;


        case '1':
            l.rotate({}, u, rotatespeed), r.rotate({}, u, rotatespeed);
            break;
        case '2':
            l.rotate({}, u, -rotatespeed), r.rotate({}, u, -rotatespeed);
            break;

        case '3':
            u.rotate({}, r, rotatespeed), l.rotate({}, r, rotatespeed);
            break;
        case '4':
            u.rotate({}, r, -rotatespeed), l.rotate({}, r, -rotatespeed);
            break;

        case '5':
            u.rotate({}, l, -rotatespeed), r.rotate({}, l, -rotatespeed);
            break;
        case '6':
            u.rotate({}, l, rotatespeed), r.rotate({}, l, rotatespeed);
            break;

        case '0':{
            capture();
            break;
        }

        case 'b':{
        #ifdef DEBUG
            bugged = 1;
            // auto g = generals[0];
            // Ray ray = {pos, g->ref-pos};
            // point ip = ray.pointAt(g->intersectingT(ray));
            // bug(g->ref, ip, g->normal(ip));
            // auto s = spheres[0];
            // ray = {pos, s->center-pos};
            // ip = ray.pointAt(s->intersectingT(ray));
            // bug(s->center, ip, s->normal(ip));
            bugged = 0;
        #endif
            break;
        }
        case 'c':
            pos = {-78.0247, 143.56, 191.998}, l = {0.500304, -0.510662, -0.699228}, r = {-0.635191, -0.765266, 0.104407}, u = {0.588412, -0.391908, 0.707234};
            break;
        default:
            inp.push_back(key);
            break;
    }
    // bug(pos, l, r, u);
}

void specialKeyListener(int key, int x,int y){
    switch(key){
        case GLUT_KEY_UP:       //down arrow key
            pos += l*movespeed;
            break;
        case GLUT_KEY_DOWN:     // up arrow key
            pos += -l*movespeed;
            break;

        case GLUT_KEY_RIGHT:
            pos += r*movespeed;
            break;
        case GLUT_KEY_LEFT:
            pos += -r*movespeed;
            break;

        case GLUT_KEY_PAGE_UP:
            pos += u*movespeed;
            break;
        case GLUT_KEY_PAGE_DOWN:
            pos += -u*movespeed;
            break;

        case GLUT_KEY_INSERT:
            break;

        case GLUT_KEY_HOME:
            break;
        case GLUT_KEY_END:
            break;

        default:
            break;
    }
    // bug(pos, l, r, u);
}


void mouseListener(int button, int state, int x, int y){    //x, y is the x-y of the screen (2D)
    switch(button){
        case GLUT_LEFT_BUTTON:
            if(state == GLUT_DOWN){     // 2 times?? in ONE click? -- solution is checking DOWN or UP
                drawgrid=1-drawgrid;
            }
            break;

        case GLUT_RIGHT_BUTTON:
            if(state == GLUT_DOWN){      // 2 times?? in ONE click? -- solution is checking DOWN or UP
                drawaxes=1-drawaxes;
            }
            break;

        case GLUT_MIDDLE_BUTTON:
            //........
            break;

        default:
            break;
    }
}



void display(){

    //clear the display
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0,0,0,0);  //color black
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /********************
    / set-up camera here
    ********************/
    //load the correct matrix -- MODEL-VIEW matrix
    glMatrixMode(GL_MODELVIEW);

    //initialize the matrix
    glLoadIdentity();

    //now give three info
    //1. where is the camera (viewer)?
    //2. where is the camera looking?
    //3. Which direction is the camera's UP direction?

    //gluLookAt(100,100,100,    0,0,0,  0,0,1);
    //gluLookAt(200*cos(cameraAngle), 200*sin(cameraAngle), cameraHeight,       0,0,0,      0,0,1);
    // gluLookAt(0,0,200,   0,0,0,  0,1,0);
    gluLookAt(pos.x, pos.y, pos.z, pos.x+l.x, pos.y+l.y, pos.z+l.z, u.x, u.y, u.z);


    //again select MODEL-VIEW
    glMatrixMode(GL_MODELVIEW);


    /****************************
    / Add your objects from here
    ****************************/
    //add objects

    if (drawaxes) drawAxes(1, 0, 0);
    if (drawgrid) drawGrid();

    drawSS();

    //ADD this line in the end --- if you use double buffer (i.e. GL_DOUBLE)
    glutSwapBuffers();
}


void animate(){
    //codes for any changes in Models, Camera
    glutPostRedisplay();
}

void init(){
    //codes for initialization
    drawgrid=0;
    drawaxes=1;
    cameraHeight=150.0;
    cameraAngle=1.0;

    setupCamera();

    //clear the screen
    glClearColor(0,0,0,0);

    /************************
    / set-up projection here
    ************************/
    //load the PROJECTION matrix
    glMatrixMode(GL_PROJECTION);

    //initialize the matrix
    glLoadIdentity();

    //give PERSPECTIVE parameters
    gluPerspective(FOV,  1,  1,  1000.0);
    //field of view in the Y (vertically)
    //aspect ratio that determines the field of view in the X direction (horizontally)
    //near distance
    //far distance
}

void clearMemory() {
    for (auto &i : objects) {
        delete i;
    }
}


signed main(int argc, char **argv) {
    auto loadData = [&]()->void {
        ifstream ifs("scene.txt");
        if (ifs.fail()) {
            cout << "Couldn't open scene.txt\n";
            exit(1);
        }
        ifs.exceptions(ifs.failbit);

        ifs >> recursionLevel >> pixel;
        int n;
        ifs >> n;
        for (int i = 0; i < n; i++) {
            string s;
            ifs >> s;
            Object *o;
            if (s=="sphere") {
                auto tmp = new Sphere();
                spheres.push_back(tmp);
                ifs >> *tmp;
                o = tmp;
            } else if (s=="triangle") {
                auto tmp = new Triangle();
                triangles.push_back(tmp);
                ifs >> *tmp;
                o = tmp;
            } else if (s=="general") {
                auto tmp = new General();
                generals.push_back(tmp);
                ifs >> *tmp;
                o = tmp;
            } else {
                cout << "Invalid input object type " << s << '\n';
                exit(1);
            }
            objects.push_back(o);
        }
        ifs >> n;
        for (int i = 0; i < n; i++) {
            Light tmp;
            ifs >> tmp;
            lights.push_back(tmp);
        }
        ifs.close();

        auto f = new Floor();
        f->floorWidth = 1000, f->tileWidth = 20, f->coeff = {0.4, 0.4, 0.1, 0.2}, f->shine = 5;
        objects.push_back(f);
    };

    loadData();

    glutInit(&argc,argv);
    glutInitWindowSize(windowHeight, windowWidth);
    glutInitWindowPosition(0, 0);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);   //Depth, Double buffer, RGB color

    glutCreateWindow("Shafin Khadem, 1605045");

    init();

    glEnable(GL_DEPTH_TEST);    //enable Depth Testing

    glutDisplayFunc(display);   //display callback function
    glutIdleFunc(animate);      //what you want to do in the idle time (when no drawing is occuring)

    glutKeyboardFunc(keyboardListener);
    glutSpecialFunc(specialKeyListener);
    glutMouseFunc(mouseListener);

    glutMainLoop();     //The main loop of OpenGL


    clearMemory();
}
