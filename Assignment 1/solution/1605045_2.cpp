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

const double acceleration = 0.03, ballRadius = 5, squareHalfLen = 75, circleRadius = 50, maxVelocity = 3, minVelocity = 0.1;

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

    double norm() { return sqrt(x*x + y*y + z*z); }

    point& normalize() { return *this = *this / norm(); }

    void rotate(const point &axisOrigin, point axisDir, const double &theta, bool thisIsAxis = 0) {
        axisDir.normalize();
        double a = axisOrigin.x, b = axisOrigin.y, c = axisOrigin.z, u = axisDir.x, v = axisDir.y, w = axisDir.z;
        double sinn = sin(theta), coss = cos(theta), dirdot = u*x+v*y+w*z;
        *this = {
            (1-coss)*(a*(v*v+w*w)-u*(b*v+c*w-dirdot)) + coss*x + sinn*(-c*v+b*w-w*y+v*z),
            (1-coss)*(b*(u*u+w*w)-v*(a*u+c*w-dirdot)) + coss*y + sinn*(c*u-a*w+w*x-u*z),
            (1-coss)*(c*(u*u+v*v)-w*(a*u+b*v-dirdot)) + coss*z + sinn*(-b*u+a*v-v*x+u*y)
        };
        if (thisIsAxis) normalize();    // we are rotating axis represented by point, hence unit vector
    }
} pos, u, r, l;

class ball {
public:
    double x, y, vx, vy;
    friend ostream &operator<<(ostream &os, const ball &rs) { return os<<'{'<<rs.x<<','<<rs.y<<','<<rs.vx<<','<<rs.vy<<'}'; }
    bool operator <(const ball &rs) const { return tie(x,y,vx,vy)<tie(rs.x,rs.y,rs.vx,rs.vy); }
    ball operator -() const { return ball{-x,-y,-vx,-vy}; }
    ball operator +(const ball &rs) const { return ball{x+rs.x,y+rs.y,vx+rs.vx,vy+rs.vy}; }
    ball& operator +=(const ball &rs) { return *this = *this + rs; }
    ball operator -(const ball &rs) const { return ball{x-rs.x,y-rs.y,vx-rs.vx,vy-rs.vy}; }
    #define TCG template<class G, class = enable_if_t<is_arithmetic<G>::value, G>>
    TCG auto operator +(const G &rs) { return ball{x+rs, y+rs, vx+rs, vy+rs}; }
    TCG auto operator -(const G &rs) { return ball{x-rs, y-rs, vx-rs, vy-rs}; }
    TCG auto operator *(const G &rs) { return ball{x*rs, y*rs, vx*rs, vy*rs}; }
    TCG auto operator /(const G &rs) { return ball{x/rs, y/rs, vx/rs, vy/rs}; }
    #undef TCG

    point ballTouchDirection(const ball &rs) {
        return point{x-rs.x, y-rs.y, 0}.norm() > ballRadius*2 ? point{1e9, 1e9, 1e9} : point{(rs.x-x)/2, (rs.y-y)/2, 0}.normalize();
    }
};


#define deg(x) (pi*(x)/180.0)
#define rad(x) (180.0*(x)/pi)
#define vertex(p) glVertex3d(p.x,p.y,p.z)
#define mat glGetDoublev(GL_MODELVIEW_MATRIX, curmat), bug(curmat)

double curmat[16], velocity = 1;
bool pause;
vector<ball> balls;


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

void drawSquare(double a)
{
    drawLine({ a, a, 0}, { a, -a, 0});
    drawLine({ a, -a, 0}, {-a, -a, 0});
    drawLine({-a,-a, 0}, {-a, a, 0});
    drawLine({-a, a, 0}, {a, a, 0});
}


void drawCircle(double radius,int segments)
{
    int i;
    struct point points[100];
    //generate points
    for(i=0;i<=segments;i++)
    {
        points[i].x=radius*cos(((double)i/(double)segments)*2*pi);
        points[i].y=radius*sin(((double)i/(double)segments)*2*pi);
    }
    //draw segments using generated points
    for(i=0;i<segments;i++)
    {
        glBegin(GL_LINES);
        {
            glVertex3d(points[i].x,points[i].y,0);
            glVertex3d(points[i+1].x,points[i+1].y,0);
        }
        glEnd();
    }
}


void drawSS()
{
    glColor3d(0, 1, 0);
    drawSquare(squareHalfLen);
    glColor3d(1, 0, 0);
    drawCircle(circleRadius, 50);
    glColor3d(.906, .996, 1);
    for (auto &i : balls) {
        glTranslated(i.x, i.y, 0);{
            drawCircle(ballRadius, 20);
        }glTranslated(-(i.x), -(i.y), -(0));
    }
}

void keyboardListener(unsigned char key, int x,int y){
	switch(key){
        case 'p':
            pause ^= 1;
            break;

		case '0':
			drawgrid=1-drawgrid;
			break;

		default:
			break;
	}
}


void specialKeyListener(int key, int x,int y){
	switch(key){
		case GLUT_KEY_UP:		//down arrow key
            if (velocity < maxVelocity) velocity += acceleration;
			break;
		case GLUT_KEY_DOWN:		// up arrow key
            if (velocity > minVelocity) velocity -= acceleration;
			break;

		case GLUT_KEY_RIGHT:
			break;
		case GLUT_KEY_LEFT:
			break;

		case GLUT_KEY_PAGE_UP:
			break;
		case GLUT_KEY_PAGE_DOWN:
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


void mouseListener(int button, int state, int x, int y){	//x, y is the x-y of the screen (2D)
	switch(button){
		case GLUT_LEFT_BUTTON:
			if(state == GLUT_DOWN){		// 2 times?? in ONE click? -- solution is checking DOWN or UP
                drawaxes=1-drawaxes;
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
	glClearColor(0,0,0,0);	//color black
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

	//gluLookAt(100,100,100,	0,0,0,	0,0,1);
	//gluLookAt(200*cos(cameraAngle), 200*sin(cameraAngle), cameraHeight,		0,0,0,		0,0,1);
	// gluLookAt(0,0,200,	0,0,0,	0,1,0);
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


double bal = 0;
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());
bitset<10> vitore;

void animate(){
    if (!pause) {
        if (balls.size() < 5 and (bal += 0.05) > 1) {
            double vx = uniform_real_distribution<double> (0.01, 0.99)(rng), vy = sqrt(1-vx*vx);
            balls.push_back({-squareHalfLen+ballRadius, -squareHalfLen+ballRadius, vx, vy}), bal = 0, bug(vx, vy);
        }
        for (auto &i : balls) {
            i.x += velocity*i.vx, i.y += velocity*i.vy;
            if (abs(i.x)+ballRadius > squareHalfLen + 1e-6) i.vx = -i.vx;
            if (abs(i.y)+ballRadius > squareHalfLen + 1e-6) i.vy = -i.vy;
        }
        int sz = balls.size();
        for (int i = 0; i < sz; i++) {
            auto &bi = balls[i];
            double curDist = point{bi.x, bi.y, 0}.norm();
            if (curDist > circleRadius - ballRadius) {
                if (vitore[i]) bi.vx = -bi.x/curDist, bi.vy = -bi.y/curDist;
            } else {
                vitore[i] = 1;
            }
            if (!vitore[i]) continue;
            for (int j = i+1; j <= sz-1; j++) {
                auto &bj = balls[j];
                if (!vitore[j]) continue;
                point colDir = bi.ballTouchDirection(bj);
                if (colDir.x < 1e8) {
                    bi.vx = -colDir.x, bi.vy = -colDir.y;
                    bj.vx = colDir.x, bj.vy = colDir.y;
                }
            }
        }
    }
	//codes for any changes in Models, Camera
	glutPostRedisplay();
}

void init(){
	//codes for initialization
	drawgrid=0;
	drawaxes=0;
	cameraHeight=150.0;
	cameraAngle=1.0;

    pos = {0, 0, 100}, l = {0, 0, -1}, u = {0, 1, 0}, r = {1, 0, 0};

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
	gluPerspective(80,	1,	1,	1000.0);
	//field of view in the Y (vertically)
	//aspect ratio that determines the field of view in the X direction (horizontally)
	//near distance
	//far distance
}

int main(int argc, char **argv){
	glutInit(&argc,argv);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);	//Depth, Double buffer, RGB color

	glutCreateWindow("My OpenGL Program");

	init();

	glEnable(GL_DEPTH_TEST);	//enable Depth Testing

	glutDisplayFunc(display);	//display callback function
	glutIdleFunc(animate);		//what you want to do in the idle time (when no drawing is occuring)

	glutKeyboardFunc(keyboardListener);
	glutSpecialFunc(specialKeyListener);
	glutMouseFunc(mouseListener);

	glutMainLoop();		//The main loop of OpenGL

	return 0;
}
