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


#if defined(_WIN32) && !defined(APIENTRY) && !defined(__CYGWIN__)
#include <windows.h>
#include <glut.h>
#else
#include <GL/glut.h>
#endif

const double pi = acos(-1), invroot2 = 1.0/sqrt(2);

double cameraHeight;
double cameraAngle;
int drawgrid;
int drawaxes;

class point {
public:
    double x, y, z;
    friend ostream &operator<<(ostream &os, const point &rs) { return os<<'{'<<rs.x<<','<<rs.y<<','<<rs.z<<'}'; }
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

    double norm() { return sqrt(x*x + y*y + z*z); }

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



#define deg(x) (pi*(x)/180.0)
#define rad(x) (180.0*(x)/pi)
#define vertex(p) glVertex3d(p.x,p.y,p.z)
#define mat glGetDoublev(GL_MODELVIEW_MATRIX, curmat), bug(curmat)

const double movespeed = 2, tiltspeed = deg(1), rotatespeed = deg(3), planeHalfLength = 100, maxAngle = deg(60);
double curmat[16], angles[5];
vector<point> shots;


void drawLine(point lineStart, point lineEnd) {
    glBegin(GL_LINES);{
        vertex(lineStart);
        vertex(lineEnd);
    }glEnd();
}


void drawAxes(double cr = 1, double cg = 1, double cb = 1) {
    double rgba[4];
    glGetDoublev(GL_CURRENT_COLOR, rgba);
    glColor3d(cr, cg, cb);
    drawLine({200, 0, 0}, {-200, 0, 0});
    drawLine({0, -200, 0}, {0, 200, 0});
    drawLine({0, 0, 200}, {0, 0, -200});
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


void drawQuad(point topLeft, point topRight, point bottomRight, point bottomLeft) {
    glBegin(GL_QUADS);{
        vertex(topLeft);
        vertex(topRight);
        vertex(bottomRight);
        vertex(bottomLeft);
    }glEnd();
}

void drawSquare(double a)
{
    glBegin(GL_QUADS);{
        glVertex3d( a, a,2);
        glVertex3d( a,-a,2);
        glVertex3d(-a,-a,2);
        glVertex3d(-a, a,2);
    }glEnd();
}


void sphericalDisk(double radius, int slices, int layers, double smallerSliceAngle, double largerSliceAngle, double smallerLayerAngle, double largerLayerAngle/*int minslice, int maxslice, int minlayer, int maxlayer*/) {
    int minslice = int(round(slices*smallerSliceAngle/(2*pi))), maxslice = int(round(slices*largerSliceAngle/(2*pi)))-1;
    int minlayer = int(round(layers*smallerLayerAngle/(2*pi))), maxlayer = int(round(layers*largerLayerAngle/(2*pi)))-1;
    struct point points[100], nextLayerPoints[100];
    for(int i = minslice; i <= maxslice+1; i++) {
        nextLayerPoints[i] = {radius*cos((i/(double)slices)*2*pi), radius*sin((i/(double)slices)*2*pi), 0};
        nextLayerPoints[i].rotate({}, {0, 1, 0}, minlayer*2*pi/layers);
    }
    for (int lay = minlayer; lay <= maxlayer; lay++) {
        glColor3d(lay&1, lay&1, lay&1);
        for(int i = minslice; i <= maxslice+1; i++) {
            points[i] = nextLayerPoints[i];
            nextLayerPoints[i].rotate({}, {0, 1, 0}, 2*pi/layers);
        }
        for (int i = minslice; i <= maxslice; i++) {
            drawQuad(points[i], points[i+1], nextLayerPoints[i+1], nextLayerPoints[i]);
        }
    }
}

void cylinder(double radius, double len, int layers, int slices) {
    for (int i = 0; i < layers; i++) {
        glColor3d(i&1, i&1, i&1);
        point topLeft = {radius, 0, 0}, topRight = {radius, len, 0}, bottomLeft = topLeft;
        bottomLeft.rotate({}, {0, 1, 0}, 2*pi/layers);
        point bottomRight = bottomLeft+point{0, len, 0};
        drawQuad(topLeft, topRight, bottomRight, bottomLeft);
        for (int j = 0; j < slices; j++) {
            topLeft = topRight, bottomLeft = bottomRight;
            topRight.rotate({radius*2, len, 0}, {0, 0, -1}, pi/(2*slices));
            bottomRight = topRight, bottomRight.rotate({}, {0, 1, 0}, 2*pi/layers);
            drawQuad(topLeft, topRight, bottomRight, bottomLeft);
        }
        glRotated(rad(2*pi/layers), 0, 1, 0);
    }
}

void drawGun(double radius, int slices, int layers) {
    glRotated(rad(angles[0]), 0, 0, 1);{
        sphericalDisk(radius, slices, layers, pi, 2*pi, 0, 2*pi);
        glRotated(rad(angles[1]), 1, 0, 0);{
            sphericalDisk(radius, slices, layers, 0, pi, 0, 2*pi);
            glRotated(rad(angles[2]), 1, 0, 0);{
                glRotated(rad(angles[3]), 0, 1, 0);{
                    glTranslated(0, (radius*3/2), 0); {
                        sphericalDisk(radius/2, slices/2, layers, pi, 2*pi, 0, 2*pi);
                        cylinder(radius/2, radius*3, layers, slices);
                    } glTranslated(0, -(radius*3/2), 0);
                }glRotated(-(rad(angles[3])), 0, 1, 0);
            }glRotated(-rad(angles[2]), 1, 0, 0);
        }glRotated(-rad(angles[1]), 1, 0, 0);
    }glRotated(-(rad(angles[0])), 0, 0, 1);
}


void drawSS()
{
    drawGun(20, 30, 100);
    glTranslated(0, 200, 0);{
        glColor3d(0.5, 0.5, 0.5);
        glRotated(90, 1, 0, 0);{
            drawSquare(planeHalfLength);
        }glRotated(-(90), 1, 0, 0);
    }glTranslated(-(0), -(200), -(0));

    point gunLine = {0, 1, 0};
    gunLine.rotate({}, {0, 0, 1}, angles[0]);
    gunLine.rotate({}, {1, 0, 0}, angles[1]+angles[2]);
    glColor3d(1, 0, 0);
    drawLine({}, gunLine*300);
    for (auto &i : shots) {
        glTranslated(i.x, i.y, i.z);{
            glRotated(90, 1, 0, 0);{
                drawSquare(3);
            }glRotated(-(90), 1, 0, 0);
        }glTranslated(-(i.x), -(i.y), -(i.z));
    }
}

string inp;
map<char, int> angleOf = {{'q', 0}, {'w', 0}, {'e', 1}, {'r', 1}, {'a', 2}, {'s', 2}, {'d', 3}, {'f', 3}};
void keyboardListener(unsigned char key, int x,int y){
    switch(key){
        case 13:
            if (sz(inp)==2) {
                inp.push_back('x'^'y'^'z'^inp[0]^inp[1]);
                pos = {150.0*(inp[0] == 'x'), 150.0*(inp[0] == 'y'), 150.0*(inp[0] == 'z')};
                l = {-1.0*(inp[0] == 'x'), -1.0*(inp[0] == 'y'), -1.0*(inp[0] == 'z')};
                r = {1.0*(inp[1] == 'x'), 1.0*(inp[1] == 'y'), 1.0*(inp[1] == 'z')};
                u = {1.0*(inp[2] == 'x'), 1.0*(inp[2] == 'y'), 1.0*(inp[2] == 'z')};
            } else {
                memset(angles, 0, sizeof(angles));
                pos = {100, 100, 0}, u = {0, 0, 1}, r = {-invroot2, invroot2, 0}, l = {-invroot2, -invroot2, 0};
            }
            bug(inp, pos, l, r, u);
            inp = "";
            break;
        case '-':
            pos = -pos, l = -l, r = -r;
            bug(pos, l, r, u);
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

        case 'q': case 'e': case 'a': case 'd':
            {
                auto &a = angles[angleOf[key]];
                a += tiltspeed;
                a = min(a, maxAngle);
            }
            break;

        case 'w': case 'r': case 's': case 'f':
            {
                auto &a = angles[angleOf[key]];
                a -= tiltspeed;
                a = max(a, -maxAngle);
            }
            break;

        case '0':
            drawgrid=1-drawgrid;
            break;

        default:
            inp.push_back(key);
            break;
    }
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
}


void mouseListener(int button, int state, int x, int y){    //x, y is the x-y of the screen (2D)
    switch(button){
        case GLUT_LEFT_BUTTON:
            if(state == GLUT_DOWN){     // 2 times?? in ONE click? -- solution is checking DOWN or UP
                point gunLine = {0, 1, 0};
                gunLine.rotate({}, {0, 0, 1}, angles[0]);
                gunLine.rotate({}, {1, 0, 0}, angles[1]+angles[2]);
                // t*gunLine.y == 200
                point shot = {200*gunLine.x/gunLine.y, 198, 200*gunLine.z/gunLine.y};
                if (max(abs(shot.x), abs(shot.z)) < planeHalfLength) shots.push_back(shot), bug(shot);
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

    if (drawaxes) drawAxes();
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

    pos = {100, 100, 0}, u = {0, 0, 1}, r = {-invroot2, invroot2, 0}, l = {-invroot2, -invroot2, 0};

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
    gluPerspective(80,  1,  1,  1000.0);
    //field of view in the Y (vertically)
    //aspect ratio that determines the field of view in the X direction (horizontally)
    //near distance
    //far distance
}

int main(int argc, char **argv){
    glutInit(&argc,argv);
    glutInitWindowSize(500, 500);
    glutInitWindowPosition(0, 0);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);   //Depth, Double buffer, RGB color

    glutCreateWindow("My OpenGL Program");

    init();

    glEnable(GL_DEPTH_TEST);    //enable Depth Testing

    glutDisplayFunc(display);   //display callback function
    glutIdleFunc(animate);      //what you want to do in the idle time (when no drawing is occuring)

    glutKeyboardFunc(keyboardListener);
    glutSpecialFunc(specialKeyListener);
    glutMouseFunc(mouseListener);

    glutMainLoop();     //The main loop of OpenGL

    return 0;
}
