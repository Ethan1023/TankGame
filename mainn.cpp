
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <iostream>
#include <vector>
#include <stdlib.h>

/*TODO:
Health and damage
Walls
pathfinding
 - 2d array of booleans representing small parts of the map, then use dead end filling
 - recursive algorithm, saving all combinations, delete ones that dont work, use shortest onedw
AI - suicidally tracks down player
Shields
Friction??? - transfer of energy on impact
*/

//{DECLARATIONS

//{CONSTANTS & VARIABLES
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 960;
const double PI = 3.14159265358979323846;

double GRAVITY = 9.81;
int mouseX = 0;
int mouseY = 0;
bool LmouseState = 0;
bool LmouseClick = 0;
bool RmouseState = 0;
bool RmouseClick = 0;
SDL_Texture* loadTexture( std::string path );
SDL_Window* gWindow = NULL;
SDL_Renderer* gRenderer = NULL;
double stepSize = 0.01;
double mul = 2;
double TICKS_PER_FRAME;
double MAX_FPS = 60;
double FPS = MAX_FPS;
int collisionAccuracy = 100;
//}

//{STRUCTS
struct PercentRect{ //used for clipping images
	double x;
	double y;
	double w;
	double h;
};

struct Projectile{
    double x;
    double y;
    double xvel;
    double yvel;
    double angle;               //Angle of projectile in degrees
    int width;
    int length;
    SDL_Texture* look;          //visual look of projectile
    SDL_Rect clip;              //clip of look to be rendered

    double timer = 0x00;
    double m = 0;
    double c = 0;
    double tx = 0;              //origin of projectile
    double ty = 0;
    bool right = 0;
};

struct AngleRect{
    double x1;
    double y1;

    double x2;
    double y2;

    double x3;
    double y3;

    double x4;
    double y4;
};

struct Shape{
    double* x;
    double* y;
    int points;
};
//}

//{ENUMERATIONS
enum command { ACCELERATE, BRAKE, TURN, TURNTURRET, BRAKETURRET };
//}

//{CLASSES
class Tank{
	public:
		Tank(std::string, double, double, double);
		~Tank();
		void step();
		void render();
		void free();
		void loadLook0();
		void loadLook(std::string, std::string, std::string, std::string, std::string, PercentRect*, PercentRect*, PercentRect*, PercentRect*, PercentRect*, SDL_Rect*, SDL_Rect*, SDL_Rect*, SDL_Rect*, SDL_Rect*);
		void setDimensions(double, double, double, double, double, double, double, double, double, double);
		void setMass(double, double, double);
		void setOther(double, double, double, double, double, double);
		void setWeapons(double, double, int, int, double, double, int, int);
		bool primaryReloaded();
		bool secondaryReloaded();

				//commands
        void pointTo(double, double);
        void shootPrimary(Projectile*);
        void shootSecondary(Projectile*);
		void accelerate(double);	//accelerate with power
		void brake(double);		//brake with force
		void turn(double);		//set power distribution between tracks (to steer)
		void turnturret(double);	//rotate turret with power
		void braketurret(double);	//brake turret with torque
				//accessors
		double getX();			//get x position
		double getY();			//get y position
		double getXvel();
		double getYvel();
		double getRot();		//get rotation of tank
		double getTurretrot();		//get rotation of turret
		double getTurretvel();		//get rotational velocity of turret
		int getPburst();    //projectiles fired per shot
		int getSburst();
		void getCorners(double*, double*);  //get coordinates of corners
		std::string getName();
		void setName(std::string);
		int getIndex();
		void setIndex(int);
		AngleRect getAngleRect();
	private:
				//graphics
		SDL_Texture* body;		//image of tank body
		SDL_Rect bodyclip;		//clip of body image file
		SDL_Texture* turret;		//image of turret
		SDL_Rect turretclip;		//clip of turret image file
		SDL_Texture* barrel;		//image of barrel
		SDL_Rect barrelclip;		//clip of barrel
		SDL_Texture* pproj;      //image of primary projectile
		SDL_Rect pprojclip;      //clip of primary projectile
		SDL_Texture* sproj;      //image of secondary projectile
		SDL_Rect sprojclip;      //clip of secondary projectile
				//position
		double x;			//x position
		double y;			//y position
		double xvel;			//x velocity
		double yvel;			//y velocity
		double vel;             //velocity
		double tankrot;			//rotation of tank
		double tankrotvel;		//rotational velocity of tank
		double turretrot;		//rotation of turret relative to tank
		double turretvel;		//rotational velocity of turret relative to tank
		double targetX;
		double targetY;
				//specs
					//mass
		double mass;			//mass of tank body
		double turretmass;		//mass of turret
		double barrelmass;		//mass of barrel
					//size
		double length;			//length of body
		double width;			//width of body
		double trackwidth;		//width of tracks
		double turretradius;		//radius of turret
		double barrellength;		//length of barrel
		double barrelwidth;		//width of barrel
		double pprojlength;
		double pprojwidth;
		double sprojlength;
		double sprojwidth;
					//other
		double power;			//power of engine (continous gearing is used)
		double brakeforce;		//force of brakes
		double turretpower;		//power of turret motor
		double turretbraketorque;	//braking torque of turret
		double strackgrip;		//static friction of tracks
		double dtrackgrip;		//dynamic friction of tracks;
                    //weapons
		double primaryProjectileSpeed;
		double secondaryProjectileSpeed;
		int primaryROF;     //reload time in game steps
		int secondaryROF;
		double primaryReload;  //time spent reloading
		double secondaryReload;
		double pspread;
		double sspread;
		int pburst;
		int sburst;

        int index;  //index of tank in
        std::string name;
		double commands [5] = {0, 0, 0, 0, 0};		//array containing values of commands passed to tank
};
//}

//{METHODS
void init();
void loadMedia();
void close();
void handleInput(Tank*);	//Reads keyboard input, and sends controls to the Tank passed in
bool stepProjectile(Projectile*);
double yfromx(double, double, double, double, double);
double yfromx(double, double, double, double, double);
bool checkcollision(AngleRect, double, double);
bool checkcollision(AngleRect, AngleRect);
void fire(bool, bool, Tank*, std::vector<Projectile*>*);
bool collision(int, double*, double*);
bool passedovershape(double, double, double, double, double*, double*, int);
bool passedover(double, double, double, double, double, double, double, double);
void sortpoints(double*, double*, int);
bool isInShape(double, double, Shape);
double atan2(double, double, double, double);
//}

//}

//{DEFINITIONS

//{TANK

void Tank::step(){
    double oldx[4];
    double oldy[4];
    getCorners(oldx, oldy);
//    std::cout << x << " " << y << "\n";
//    AngleRect T = getAngleRect();
//    std::cout << "1: " << T.x1 << ", " << T.y1 << "\n";
//    std::cout << "2: " << T.x2 << ", " << T.y2 << "\n";
//    std::cout << "3: " << T.x3 << ", " << T.y3 << "\n";
//    std::cout << "4: " << T.x4 << ", " << T.y4 << "\n";
    primaryReload+=200.0/FPS;
    secondaryReload+=200.0/FPS;
    turretrot = (atan((targetY - y)/(targetX - x))*180/PI) - tankrot;
    targetX > x ? turretrot += 180 : turretrot;
    //std::cout << "TurretRot: " << turretrot << "\n";
    //std::cout << "TankRot:   " << tankrot << "\n";
//    std::cout << tankrot << "\n";
//    std::cout << "tankrot += " << commands[TURN] << " * " << stepSize << " * " << sqrt(abs(vel)+1) << " / 10;\n";
//	std::cout << "abs(vel)+1 = " << abs(vel)+1 << "\n";
//	std::cout << "vel = " << vel << "\n";
	tankrot += commands[TURN] * stepSize * sqrt(abs(vel)+1) / 10;
	if(collision(index, oldx, oldy)){
        tankrot -= commands[TURN] * stepSize * sqrt(abs(vel)+1) / 10;
        std::cout << "Collision Detected...\n";
	}
	tankrot > 360 ? tankrot -= 360 : (tankrot < 0 ? tankrot += 360 : tankrot = tankrot);
//	double tankrottemp = tankrot;
//	tankrottemp > 360 ? tankrottemp -= 360 : (tankrottemp < 0 ? tankrottemp += 360 : tankrottemp = tankrottemp);
//    std::cout << tankrot << "\n";
	double tankrotrad = (tankrot) * PI / 180;    //convert from visual rotation to rotation for calculation
//	std::cout << "Rot: " << tankrot << "\nSin: " << sin(tankrotrad) << "\nCos: " << cos(tankrotrad) << "\n";
//	std::cout << tankrotrad << "\n";
	double usedpower = commands[ACCELERATE] * power / 100;  //calculate power used
	double drag = vel * vel / 10;   //drag so that tank can coast to a stop
	double braking = commands[BRAKE] * brakeforce / 100 + 10 + drag;   //strength of brakes being applied
	double totalmass = mass + turretmass + barrelmass;  //total mass of tank
	double thrust = usedpower / ( abs(vel) + 0.1 );	//calculate thrust at current velocity
	thrust > usedpower/10 ? thrust = usedpower/10 : thrust = thrust;
	thrust < usedpower/10 ? thrust = usedpower/10 : thrust = thrust;
    double dvel;
											//calculate acceleration provided to tracks by engine and brakes
	if( abs( thrust ) > braking ){							//tracks are powered by the engine
		vel > 0 ? thrust -= braking : thrust += braking;			//calculate net thrust
		dvel = stepSize * thrust / totalmass;				//calculate acceleration
	}else{										//tracks are braking
		thrust = braking - abs(thrust);						//calculate net braking
		vel > 0 ? dvel = -1 * stepSize * thrust / totalmass : dvel = stepSize * thrust / totalmass;				//calculate acceleration
	}
	vel += dvel;
    xvel = cos(tankrotrad)*vel;
    yvel = sin(tankrotrad)*vel;
//    std::cout << tankrotrad << "\n";
    x+=stepSize*xvel;
    y+=stepSize*yvel;
    if(collision(index, oldx, oldy)){
        x-=stepSize*xvel;
        y-=stepSize*yvel;
        vel/=2;
        std::cout << "Collision Detected...\n";
	}
//	std::cout << x << " " << y << "\n";
//	std::cout << "Thrust:   " << thrust << "\n";
//	std::cout << "Velocity: " << vel << "\n";
/*	std::cout << "X Vel:    " << xvel << "\n";
	std::cout << "Y Vel:    " << yvel << "\n";*/
//	std::cout << "X coord:  " << x << "\n";
//	std::cout << "Y coord:  " << y << "\n";
}

void Tank::shootPrimary(Projectile* p){
    primaryReload = 0;
    p->x = x-cos((tankrot+turretrot)*PI/180)*mul*(turretradius + barrellength)/2;
    p->y = y-sin((tankrot+turretrot)*PI/180)*mul*(turretradius + barrellength)/2;
    p->angle = turretrot+tankrot;
    if(pspread>0)
        p->angle += ((rand()%1000)*1.0)/(500/(pspread)) - pspread;
    p->xvel = xvel-cos(p->angle*PI/180)*primaryProjectileSpeed;
    p->yvel = yvel-sin(p->angle*PI/180)*primaryProjectileSpeed;
    p->width = pprojwidth;
    p->length = pprojlength;
    p->look = pproj;
    p->clip = pprojclip;

    p->m = (targetY - y)/(targetX - x);
    p->c = y - p->m*x;
    p->tx = p->x;//-cos((tankrot+turretrot)*PI/180)*mul*(turretradius + barrellength)/2;
    p->ty = p->y;//-sin((tankrot+turretrot)*PI/180)*mul*(turretradius + barrellength)/2;
    p->tx > targetX ? p->right = 0 : p->right = 1;
}

void Tank::shootSecondary(Projectile* p){
    secondaryReload = 0;
    p->x = x-cos((tankrot+turretrot)*PI/180)*mul*(turretradius + barrellength)/2;
    p->y = y-sin((tankrot+turretrot)*PI/180)*mul*(turretradius + barrellength)/2;
    p->angle = turretrot+tankrot;
    if(sspread>0)
        p->angle += ((rand()%1000)*1.0)/(500/(sspread)) - sspread;
    p->xvel = xvel-cos(p->angle*PI/180)*secondaryProjectileSpeed;
    p->yvel = yvel-sin(p->angle*PI/180)*secondaryProjectileSpeed;

    p->width = sprojwidth;
    p->length = sprojlength;
    p->look = sproj;
    p->clip = sprojclip;

    p->m = /*(targetY - y)/(targetX - x) +*/ tan(p->angle*PI/180);
    p->c = y - p->m*x;
    p->tx = p->x;//-cos((p->angle)*PI/180)*mul*(turretradius + barrellength)/2;
    p->ty = p->y;//-sin((p->angle)*PI/180)*mul*(turretradius + barrellength)/2;
    p->tx > targetX ? p->right = 0 : p->right = 1;
}

void Tank::pointTo(double Tx, double Ty){
    targetX = Tx;
    targetY = Ty;
}

Tank::Tank(std::string nname, double xpos, double ypos, double rot){
	name = nname;
	x = xpos;
	y = ypos;
	tankrot = rot;
	turretrot = 0;
	turretvel = 0;
	xvel = 0;
	yvel = 0;
	tankrotvel = 0;
	vel = 0;
}

Tank::~Tank(){
	//free();
}

void Tank::setOther(double npower, double nbrakeforce, double nturretpower, double nturretbraketorque, double nstrackgrip, double ndtrackgrip){
	if( npower != 0 )
		power = npower;
	if( nbrakeforce != 0 )
		brakeforce = nbrakeforce;
	if( nturretpower != 0 )
		turretpower = nturretpower;
	if( nturretbraketorque != 0 )
		turretbraketorque = nturretbraketorque;
	if( nstrackgrip != 0 )
		strackgrip = nstrackgrip;
	if( ndtrackgrip != 0 )
		dtrackgrip = ndtrackgrip;
}

void Tank::setWeapons(double nPprojectileSpeed, double nSprojectileSpeed, int nprimaryROF, int nsecondaryROF, double npspread, double nsspread, int npburst, int nsburst){
    if( nPprojectileSpeed > 0 )
        primaryProjectileSpeed = nPprojectileSpeed;
    if( nSprojectileSpeed > 0 )
        secondaryProjectileSpeed = nSprojectileSpeed;
    if( nprimaryROF > 0 )
        primaryROF = nprimaryROF;
    if( nsecondaryROF > 0 )
        secondaryROF = nsecondaryROF;
    primaryReload = primaryROF;
    secondaryReload = secondaryROF;
    pspread = npspread;
    sspread = nsspread;
    if( npburst > 0)
        pburst = npburst;
    if( nsburst > 0)
        sburst = nsburst;
}

void Tank::setMass(double nmass, double nturretmass, double nbarrelmass){
	if( nmass != 0 )
		mass = nmass;
	if( nturretmass != 0 )
		turretmass = nturretmass;
	if( nbarrelmass != 0 )
		barrelmass = nbarrelmass;
}

void Tank::setDimensions(double nlength, double nwidth, double ntrackwidth, double nturretradius, double nbarrellength, double nbarrelwidth, double npprojlength, double npprojwidth, double nsprojlength, double nsprojwidth){
	if( nlength != 0 )
		length = nlength*mul;
	if( nwidth != 0 )
		width = nwidth*mul;
	if( ntrackwidth != 0 )
		trackwidth = ntrackwidth*mul;
	if( nturretradius != 0 )
		turretradius = nturretradius*mul;
	if( nbarrellength != 0 )
		barrellength = nbarrellength*mul;
	if( nbarrelwidth != 0 )
		barrelwidth = nbarrelwidth*mul;
    if( npprojlength != 0 )
        pprojlength = npprojlength*mul;
    if( npprojwidth != 0 )
        pprojwidth = npprojwidth*mul;
    if( nsprojlength != 0 )
        sprojlength = nsprojlength*mul;
    if( nsprojwidth != 0 )
        sprojwidth = nsprojwidth*mul;
}

void Tank::free(){
    //Causes segfaults
	if(body!=NULL){
		SDL_DestroyTexture(body);
	}
	if(turret!=NULL){
		SDL_DestroyTexture(turret);
	}
	if(barrel!=NULL){
		SDL_DestroyTexture(barrel);
    }
    if(pproj!=NULL){
        SDL_DestroyTexture(pproj);
    }
    if(sproj!=NULL){
        SDL_DestroyTexture(sproj);
    }
}

void Tank::loadLook0(){
    loadLook("body.png", "turret.png", "barrel.png", "primaryprojectile.png", "secondaryprojectile.png", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}

void Tank::loadLook(std::string bodypath = "body.png", std::string turretpath = "turret.png", std::string barrelpath = "barrel.png", std::string pprojpath = "projectile.png", std::string sprojpath = "secondaryprojectile.png", PercentRect* bodysource = NULL, PercentRect* turretsource = NULL, PercentRect* barrelsource = NULL, PercentRect* pprojsource = NULL, PercentRect* sprojsource = NULL, SDL_Rect* bodystretchn = NULL, SDL_Rect* turretstretchn = NULL, SDL_Rect* barrelstretchn = NULL, SDL_Rect* pprojstretchn = NULL, SDL_Rect* sprojstretchn = NULL){
	//free();

	SDL_Surface* bodyimage = IMG_Load( bodypath.c_str() );						//Load images
	SDL_Surface* turretimage = IMG_Load( turretpath.c_str() );
	SDL_Surface* barrelimage = IMG_Load( barrelpath.c_str() );
	SDL_Surface* pprojimage = IMG_Load( pprojpath.c_str() );
	SDL_Surface* sprojimage = IMG_Load( sprojpath.c_str() );

	SDL_SetColorKey( bodyimage, SDL_TRUE, SDL_MapRGB( bodyimage->format, 0xFF, 0xFF, 0xFF ) );	//Colour key loaded images
	SDL_SetColorKey( turretimage, SDL_TRUE, SDL_MapRGB( turretimage->format, 0xFF, 0xFF, 0xFF ) );
	SDL_SetColorKey( barrelimage, SDL_TRUE, SDL_MapRGB( barrelimage->format, 0xFF, 0xFF, 0xFF ) );
	SDL_SetColorKey( pprojimage, SDL_TRUE, SDL_MapRGB( pprojimage->format, 0xFF, 0xFF, 0xFF ) );
	SDL_SetColorKey( sprojimage, SDL_TRUE, SDL_MapRGB( sprojimage->format, 0xFF, 0xFF, 0xFF ) );

	body = SDL_CreateTextureFromSurface( gRenderer, bodyimage );					//Create texture
	turret = SDL_CreateTextureFromSurface( gRenderer, turretimage );
	barrel = SDL_CreateTextureFromSurface( gRenderer, barrelimage );
	pproj = SDL_CreateTextureFromSurface( gRenderer, pprojimage );
	sproj = SDL_CreateTextureFromSurface( gRenderer, sprojimage );

	if( bodysource == NULL ){									//Prepare rectangles for clipping textures
		SDL_Rect temp = {0, 0, bodyimage->w, bodyimage->h};
		bodyclip = temp;
	}else{
		bodyclip.w = bodyimage->w * bodysource->w / 100;
		bodyclip.h = bodyimage->h * bodysource->h / 100;
		bodyclip.x = bodyimage->w * bodysource->x / 100;
		bodyclip.y = bodyimage->h * bodysource->y / 100;
	}
	if( turretsource == NULL ){
		SDL_Rect temp = {0, 0, turretimage->w, turretimage->h};
		turretclip = temp;
	}else{
		turretclip.w = turretimage->w * turretsource->w / 100;
		turretclip.h = turretimage->h * turretsource->h / 100;
		turretclip.x = turretimage->w * turretsource->x / 100;
		turretclip.y = turretimage->h * turretsource->y / 100;
	}
	if( barrelsource == NULL ){
		SDL_Rect temp = {0, 0, barrelimage->w, barrelimage->h};
		barrelclip = temp;
	}else{
		barrelclip.w = barrelimage->w * barrelsource->w / 100;
		barrelclip.h = barrelimage->h * barrelsource->h / 100;
		barrelclip.x = barrelimage->w * barrelsource->x / 100;
		barrelclip.y = barrelimage->h * barrelsource->y / 100;
	}
	if( pprojsource == NULL ){
		SDL_Rect temp = {0, 0, pprojimage->w, pprojimage->h};
		pprojclip = temp;
	}else{
		pprojclip.w = pprojimage->w * pprojsource->w / 100;
		pprojclip.h = pprojimage->h * pprojsource->h / 100;
		pprojclip.x = pprojimage->w * pprojsource->x / 100;
		pprojclip.y = pprojimage->h * pprojsource->y / 100;
	}

	if( sprojsource == NULL ){
		SDL_Rect temp = {0, 0, sprojimage->w, sprojimage->h};
		sprojclip = temp;
	}else{
		sprojclip.w = sprojimage->w * sprojsource->w / 100;
		sprojclip.h = sprojimage->h * sprojsource->h / 100;
		sprojclip.x = sprojimage->w * sprojsource->x / 100;
		sprojclip.y = sprojimage->h * sprojsource->y / 100;
	}

	SDL_FreeSurface( bodyimage );									//Free surfaces
	SDL_FreeSurface( turretimage );
	SDL_FreeSurface( barrelimage );
	SDL_FreeSurface( pprojimage );
	SDL_FreeSurface( sprojimage );
}

void Tank::render(){											//Render tank at current position with set dimentions
//	std::cout << "Rendering at " << x << ", " << y << "\n";
	SDL_Rect temp = {x-width/2, y-length/2, width, length};						//x-width/2 is used, so that the x and y coordinates refer to the center of the tank
	SDL_RenderCopyEx( gRenderer, body, &bodyclip, &temp, tankrot+90, NULL, SDL_FLIP_NONE);

	temp = {x-turretradius, y-turretradius, turretradius*2, turretradius*2};
	SDL_RenderCopyEx( gRenderer, turret, &turretclip, &temp, tankrot+turretrot+90, NULL, SDL_FLIP_NONE);

	SDL_Point center = {barrelwidth/2, 1-turretradius};
	temp = {x-barrelwidth/2, y+turretradius, barrelwidth, barrellength};
	SDL_RenderCopyEx( gRenderer, barrel, &barrelclip, &temp, tankrot+turretrot+90, &center, SDL_FLIP_NONE);
}
//{those methods

int Tank::getIndex(){
    return index;
}

void Tank::setIndex(int i){
    index = i;
}

std::string Tank::getName(){
    return name;
}

void Tank::setName(std::string nname){
    name = nname;
}

AngleRect Tank::getAngleRect(){
    AngleRect* temp = new AngleRect;
    AngleRect t = *temp;
    double tankrotrad = tankrot * PI / 180;
    t.x1 = x+cos(tankrotrad)*length/2+sin(tankrotrad)*width/2;    //front left
    t.y1 = y+sin(tankrotrad)*length/2-cos(tankrotrad)*width/2;
    t.x2 = x+cos(tankrotrad)*length/2-sin(tankrotrad)*width/2;    //front right
    t.y2 = y+sin(tankrotrad)*length/2+cos(tankrotrad)*width/2;
    t.x3 = x-cos(tankrotrad)*length/2+sin(tankrotrad)*width/2;    //back left
    t.y3 = y-sin(tankrotrad)*length/2-cos(tankrotrad)*width/2;
    t.x4 = x-cos(tankrotrad)*length/2-sin(tankrotrad)*width/2;    //back right
    t.y4 = y-sin(tankrotrad)*length/2+cos(tankrotrad)*width/2;
//    std::cout << "getAngleRect used\n";
    return t;
}


void Tank::getCorners(double* xs, double* ys){
    xs;// = new double[4];
    ys;// = new double[4];
    double tankrotrad = tankrot * PI / 180;
    xs[0] = x+cos(tankrotrad)*length/2+sin(tankrotrad)*width/2;    //front left
    ys[0] = y+sin(tankrotrad)*length/2-cos(tankrotrad)*width/2;
    xs[1] = x+cos(tankrotrad)*length/2-sin(tankrotrad)*width/2;    //front right
    ys[1] = y+sin(tankrotrad)*length/2+cos(tankrotrad)*width/2;
    xs[2] = x-cos(tankrotrad)*length/2+sin(tankrotrad)*width/2;    //back left
    ys[2] = y-sin(tankrotrad)*length/2-cos(tankrotrad)*width/2;
    xs[3] = x-cos(tankrotrad)*length/2-sin(tankrotrad)*width/2;    //back right
    ys[3] = y-sin(tankrotrad)*length/2+cos(tankrotrad)*width/2;
}

int Tank::getPburst(){
    return pburst;
}

int Tank::getSburst(){
    return sburst;
}
/*double Tank::getTurretradius(){
    return turretradius;
}

double Tank::getBarrellength(){
    return barrellength;
}*/

double Tank::getX(){
	return x;
}

double Tank::getY(){
	return y;
}

double Tank::getXvel(){
    return xvel;
}

double Tank::getYvel(){
    return yvel;
}

double Tank::getRot(){
	return tankrot;
}

double Tank::getTurretrot(){
	return turretrot;
}

double Tank::getTurretvel(){
	return turretvel;
}

void Tank::accelerate(double percent = 100){
	commands[ACCELERATE] = percent;
}

void Tank::brake(double percent = 100){
	commands[BRAKE] = percent;
}

void Tank::turn(double diff = 0){
	commands[TURN] = diff;
}

void Tank::turnturret(double percent = 0){
	commands[TURNTURRET] = percent;
}

void Tank::braketurret(double percent = 100){
	commands[BRAKETURRET] = percent;
}

bool Tank::primaryReloaded(){
    return primaryReload > primaryROF;
}

bool Tank::secondaryReloaded(){
    return secondaryReload > secondaryROF;
}
//}
//}

//{METHODS
void init(){
	SDL_Init( SDL_INIT_VIDEO );
	SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" );
	gWindow = SDL_CreateWindow( "Game", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
	gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED );
	SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	IMG_Init( IMG_INIT_PNG );

	TICKS_PER_FRAME = 1000/MAX_FPS;
	stepSize = 2/MAX_FPS;
}

void close(){
	SDL_DestroyRenderer( gRenderer );
	SDL_DestroyWindow( gWindow );
	gWindow = NULL;
	gRenderer = NULL;
	IMG_Quit();
	SDL_Quit();
}

std::vector<AngleRect> walls;
std::vector<Tank*> tanks;

int main( int argc, char* args[] ){
	init();
	//loadMedia();
	//{Initialize Tanks
	Tank* t = new Tank("player", 100, 100, -45);
	t->setDimensions(20, 10, 2, 4, 10, 3, 8, 2, 2, 2);
	t->setMass(100, 50, 10);
	t->setOther(10000, 2000, 10, 10, 0.8, 0.5);
	t->setWeapons(5000, 500, 500, 10, 1, 3, 3, 10);
	t->loadLook0();
    tanks.push_back(t);
	t = new Tank("enemy", SCREEN_WIDTH-100, SCREEN_HEIGHT-100, 135);
	t->setDimensions(20, 10, 2, 4, 10, 3, 8, 2, 2, 2);
	t->setMass(100, 50, 10);
	t->setOther(1000, 1245.69, 10, 10, 0.8, 0.5);
	t->setWeapons(500, 250, 500, 10, 0, 20, 1, 10);
	t->loadLook0();
	tanks.push_back(t);
	t = NULL;
	for(int i=0;i<tanks.size();i++){
        tanks[i]->setIndex(i);
	}
    //}

    AngleRect* temp1 = new AngleRect;
    AngleRect walltemp = *temp1;
    walltemp.x1 = 200.0;
    walltemp.y1 = 200.0;
    walltemp.x2 = 1080.0;
    walltemp.y2 = 200.0;
    walltemp.x3 = 200.0;
    walltemp.y3 = 760.0;
    walltemp.x4 = 1080.0;
    walltemp.y4 = 760.0;
    walls.push_back(walltemp);

    int frameTimer;

	bool quit = false;
	SDL_Event e;
	std::vector<Shape*> shapes;
    std::vector<Projectile*> projectiles;
    std::vector<double> frames;

    double x1[4] = {200, 1080, 200, 1080};
    double y1[4] = {200, 200, 760, 760};
    sortpoints(x1, y1, 4);
    Shape shape1;
    shape1.x = x1;
    shape1.y = y1;
    shape1.points = 4;
    double x[6] = {50, 100, 100, 0, 50, 0};
    double y[6] = {0, 50, 100, 10, 150, 75};
    sortpoints(x, y, 6);
    Shape shape;
    shape.x = x;
    shape.y = y;
    shape.points = 6;
    isInShape(tanks[0]->getX(), tanks[0]->getY(), shape);
//    std::cout << "1: " << x[0] << ", " << y[0] << "\n2: " << x[1] << ", " << y[1] << "\n3: " << x[2] << ", " << y[2] << "\n4: " << x[3] << ", " << y[3] << "\n";
	while( !quit ){      //DONT FORGET - NEEDS TO BE !quit

        frameTimer = SDL_GetTicks();
            isInShape(tanks[0]->getX(), tanks[0]->getY(), shape);
        //{Calculate FPS
        frames.push_back(SDL_GetTicks());
        for(int i=0;i<frames.size();i++){
            if(frames[i] + 1000 < SDL_GetTicks()){
                frames.erase(frames.begin()+i);
            }
        }
//        std::cout << "FPS: " << frames.size() << "\n";
        FPS = frames.size();
        stepSize = 2/(FPS);
        //}
        //{Handle input
        LmouseClick = 0;
        RmouseClick = 0;
		while( SDL_PollEvent( &e ) != 0 ){
			if( e.type == SDL_QUIT ){
				quit = true;
			} else if( e.type == SDL_MOUSEMOTION ){
                SDL_GetMouseState( &mouseX, &mouseY );
            } else if( e.type == SDL_MOUSEBUTTONDOWN ){
                if(e.button.button == SDL_BUTTON_LEFT){
                    LmouseState = 1;
                    LmouseClick = 1;
                } else if( e.button.button == SDL_BUTTON_RIGHT){
                    RmouseState = 1;
                    RmouseClick = 1;
                }
            } else if( e.type == SDL_MOUSEBUTTONUP ){
                if(e.button.button == SDL_BUTTON_LEFT)
                    LmouseState = 0;
                else if(e.button.button == SDL_BUTTON_RIGHT)
                    RmouseState = 0;
            }
		}
		handleInput(tanks[0]);
		tanks[0]->pointTo( mouseX*1.0, mouseY*1.0 );
		tanks[0]->step();
		fire(LmouseState, RmouseState, tanks[0], &projectiles);
        //}
        //enemy AI
        for(int i=1; i<tanks.size(); i++){
            tanks[i]->pointTo( tanks[0]->getX(), tanks[0]->getY() );
            tanks[i]->step();
            fire(true, false, tanks[i], &projectiles);
        }
        SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
        SDL_RenderDrawLine( gRenderer, walltemp.x1, walltemp.y1, walltemp.x2, walltemp.y2 );
        SDL_RenderDrawLine( gRenderer, walltemp.x3, walltemp.y3, walltemp.x4, walltemp.y4 );
        SDL_RenderDrawLine( gRenderer, walltemp.x1, walltemp.y1, walltemp.x3, walltemp.y3 );
        SDL_RenderDrawLine( gRenderer, walltemp.x2, walltemp.y2, walltemp.x4, walltemp.y4 );
        SDL_RenderDrawLine( gRenderer, x[5], y[5], x[0], y[0] );
        for(int i=0;i<5;i++)
            SDL_RenderDrawLine( gRenderer, x[i], y[i], x[i+1], y[i+1] );

//		if(collision(0)){
//            std::cout << "Collision Detected..." << frameTimer << "\n";
//		}

        //render
//		SDL_RenderClear( gRenderer );
		for(int i = 0; i < projectiles.size(); i++){
            bool remove = stepProjectile(projectiles[i]);
            if(remove){
                projectiles.erase(projectiles.begin()+i);
            }
        }
//        std::cout << "Projectile count: " << projectiles.size() << "\n";
		for(int i=0; i<tanks.size(); i++){
            tanks[i]->render();
		}
		SDL_RenderPresent( gRenderer );
		SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
		SDL_RenderClear( gRenderer );
		if(SDL_GetTicks()-frameTimer < TICKS_PER_FRAME)
            SDL_Delay(TICKS_PER_FRAME-(SDL_GetTicks()-frameTimer));
		//SDL_Delay(100);
	}
	close();
	return 0;
}

void fire(bool pfire, bool sfire, Tank* tank, std::vector<Projectile*>* projectiles){
    if(pfire && tank->primaryReloaded()){
        for(int i=0; i<tank->getPburst(); i++){
            Projectile* t = new Projectile;
            projectiles->push_back(t);
            tank->shootPrimary(t);
        }
    }
   	if(sfire && tank->secondaryReloaded()){
        for(int i=0; i<tank->getSburst(); i++){
            Projectile* t = new Projectile;
            projectiles->push_back(t);
            tank->shootSecondary(t);
        }
    }
}

bool stepProjectile(Projectile* proj){
    //render line
    bool ret = false;
    if(proj->timer < 0xFF){
        SDL_SetRenderDrawColor( gRenderer, 0xFF, proj->timer, proj->timer, 0xFF );
        SDL_RenderDrawLine( gRenderer, proj->tx, proj->ty, proj->x, proj->y );
        proj->timer+=200.0/FPS;
    }
    //render projectile
    SDL_Rect renderposition = {proj->x-proj->width/2, proj->y-proj->length/2, proj->width, proj->length};						//x-width/2 is used, so that the x and y coordinates refer to the center of the tank
	SDL_RenderCopyEx( gRenderer, proj->look, &proj->clip, &renderposition, proj->angle+90, NULL, SDL_FLIP_NONE);
	for(int i=0; i<tanks.size(); i++){
        double tx[4];
        double ty[4];
        tanks[i]->getCorners(tx, ty);
        if(passedovershape(proj->x, proj->y, proj->x+proj->xvel*stepSize, proj->y+proj->yvel*stepSize, tx, ty, 4)){
            std::cout << "Tank " << i << " hit by a projectile\n";
            return true;
        }
	}
/*	AngleRect temp = tanks[0]->getAngleRect();
	double tx[4] = {temp.x1, temp.x2, temp.x3, temp.x4};
	double ty[4] = {temp.y1, temp.y2, temp.y3, temp.y4};
	if(passedovershape(proj->x, proj->y, proj->x+proj->xvel*stepSize, proj->y+proj->yvel*stepSize, tx, ty, 4)){
        std::cout << "Hit by projectile\n";
        return true;
    }
    temp = tanks[1]->getAngleRect();
	tx[0] = temp.x1; tx[1] = temp.x2; tx[2] = temp.x3; tx[3] = temp.x4;
	ty[0] = temp.y1; ty[1] = temp.y2; ty[2] = temp.y3; ty[3] = temp.y4;
	if(passedovershape(proj->x, proj->y, proj->x+proj->xvel*stepSize, proj->y+proj->yvel*stepSize, tx, ty, 4)){
        std::cout << "Hit by projectile\n";
        return true;
    }*/
//    std::cout << "x1: " << proj->x << "\ny1: " << proj->y;
    proj->x += proj->xvel*stepSize;
    proj->y += proj->yvel*stepSize;
//    std::cout << "\nx2: " << proj->x << "\ny2: " << proj->y << "\n";
    //drag
//    proj->xvel > 0 ? proj->xvel *= 1-stepSize/10 : proj->xvel *= 1-stepSize/10;
//    proj->yvel > 0 ? proj->yvel *= 1-stepSize/10 : proj->yvel *= 1-stepSize/10;
    //maths to find if projectile will cross screen
    //if not (going to cross screen and going towards screen
    if(proj->x < 0 || proj->x > SCREEN_WIDTH || proj->y < 0 || proj->y > SCREEN_HEIGHT){    //off the screen
        if(proj->y > SCREEN_HEIGHT/2){  //below centre of the screen
            if(proj->x > SCREEN_WIDTH/2){   //bottom right
                if(!(yfromx(SCREEN_WIDTH, proj->x, proj->y, proj->xvel, proj->yvel) > 0 && yfromx(0, proj->x, proj->y, proj->xvel, proj->yvel) < SCREEN_HEIGHT && proj->xvel < 0 && proj->yvel < 0))
                    ret = true;
            }else{  //bottom left
                if(!(yfromx(SCREEN_WIDTH, proj->x, proj->y, proj->xvel, proj->yvel) < SCREEN_HEIGHT && yfromx(0, proj->x, proj->y, proj->xvel, proj->yvel) > 0 && proj->xvel > 0 && proj->yvel < 0))
                    ret = true;
            }
        }else{  //above centre of the screen
            if(proj->x > SCREEN_WIDTH/2){   //top right
                if(!(yfromx(SCREEN_WIDTH, proj->x, proj->y, proj->xvel, proj->yvel) < SCREEN_HEIGHT && yfromx(0, proj->x, proj->y, proj->xvel, proj->yvel) > 0 && proj->xvel < 0 && proj->yvel > 0))
                    ret = true;
            }else{  //top left
                if(!(yfromx(SCREEN_WIDTH, proj->x, proj->y, proj->xvel, proj->yvel) > 0 && yfromx(0, proj->x, proj->y, proj->xvel, proj->yvel) < SCREEN_HEIGHT && proj->xvel > 0 && proj->yvel > 0))
                    ret = true;
            }
        }
    }
    if(proj->timer < 0xFF)
        return false;
    return ret;
}

double yfromx(double x, double x1, double y1, double xvel, double yvel){    //return y coordinate on a line at given x coordinate
    return x*yvel/xvel + y1 - x1*yvel/xvel;
}

double yfromxcoords(double x, double x1, double y1, double x2, double y2){
    double m = (y1-y2)/(x1-x2);
    double c = y1 - m * x1;
    return m*x + c;
}

bool checkcollision(AngleRect shape, double x, double y){   //USE THIS TO CHECK IF LINE PASSED OVER DOT; ITS THE SAME THING FFS !!!
    bool b12and34 = false;  //between lines 12 and 34
    bool b13and24 = false;
    double y12 = yfromxcoords(x, shape.x1, shape.y1, shape.x2, shape.y2);
    double y34 = yfromxcoords(x, shape.x3, shape.y3, shape.x4, shape.y4);
    double y13 = yfromxcoords(x, shape.x1, shape.y1, shape.x3, shape.y3);
    double y24 = yfromxcoords(x, shape.x2, shape.y2, shape.x4, shape.y4);
    if(y12 != y12 || y34 != y34){   //if y12 or y34 are NAN swap x and y axis
        y12 = yfromxcoords(y, shape.y1, shape.x1, shape.y2, shape.x2);
        y34 = yfromxcoords(y, shape.y3, shape.x3, shape.y4, shape.x4);
        if((x < y12) == (x > y34))
            b12and34 = true;
    }else{
        if((y < y12) == (y > y34))  //if point is between the two lines
            b12and34 = true;
    }

    if(y24 != y24 || y13 != y13){
        y13 = yfromxcoords(y, shape.y1, shape.x1, shape.y3, shape.x3);
        y24 = yfromxcoords(y, shape.y2, shape.x2, shape.y4, shape.x4);
        if((x < y13) == (x > y24))
            b13and24 = true;
    }else{
        if((y < y13) == (y > y24))
            b13and24 = true;
    }

    return b12and34 && b13and24;    //is point between all four lines
}

bool checkcollision(AngleRect shape1, AngleRect shape2){
    return checkcollision(shape1, shape2.x1, shape2.y1) || checkcollision(shape1, shape2.x2, shape2.y2) || checkcollision(shape1, shape2.x3, shape2.y3) || checkcollision(shape1, shape2.x4, shape2.y4)
    || checkcollision(shape2, shape1.x1, shape1.y1) || checkcollision(shape2, shape1.x2, shape1.y2) || checkcollision(shape2, shape1.x3, shape1.y3) || checkcollision(shape2, shape1.x4, shape1.y4);
}

bool passedovershape(double xa, double ya, double xb, double yb, double* xs, double* ys, int values){
    for(int i=0;i<values-1;i++){
        for(int j=i+1;j<values;j++){
                if(passedover(xa, ya, xb, yb, xs[i], ys[i], xs[j], ys[j])){
                    return true;
                }
        }
    }
    return false;
}

bool passedover(double xa, double ya, double xb, double yb, double x1, double y1, double x2, double y2){
    double ma = (yb-ya)/(xb-xa);
    double m1 = (y2-y1)/(x2-x1);
    double ca = ya - ma * xa;
    double c1 = y1 - m1 * x1;

    double x = (ca-c1)/(m1-ma);
    double y = m1 * x + c1;
    return ((x<xa != x<xb) && (x<x1 != x<x2));// && ((y<ya != y<yb) && (y<y1 != y<y2));
}

void handleInput(Tank* player){
	const Uint8* keys = SDL_GetKeyboardState( NULL );
	if( keys[ SDL_SCANCODE_W ] ){
		player->accelerate(100);
	}else if( keys[ SDL_SCANCODE_S ] ){
		player->accelerate(-100);
	}else{
		player->accelerate(0);
	}
	if( !( keys[ SDL_SCANCODE_A ] && keys[ SDL_SCANCODE_D ] ) ){
		if( keys[ SDL_SCANCODE_A ] ){
			player->turn(-100);
		}else if( keys[ SDL_SCANCODE_D ] ){
			player->turn(100);
		}else{
			player->turn(0);
		}
	}
	if( keys[ SDL_SCANCODE_LSHIFT ] ){
		player->brake(100);
	}else{
		player->brake(0);
	}
    //secret hack
    if( keys[ SDL_SCANCODE_E ] && keys[ SDL_SCANCODE_Q]){
        player->accelerate(10000);
    } else if( keys[ SDL_SCANCODE_Z ] && keys[ SDL_SCANCODE_C]){
        player->accelerate(-10000);
    }
}

bool collision(int index, double* xindexo, double* yindexo){
    for(int i=0;i<tanks.size();i++){
        if(! i==index){
            if(checkcollision(tanks[i]->getAngleRect(), tanks[index]->getAngleRect()))
                return true;
        }
    }
    for(int i=0;i<walls.size();i++){
        if(checkcollision(walls[i], tanks[index]->getAngleRect())){
            std::cout << "Tank: " << index << "\n";
            return true;
        }
    }
    return false;

/*    for(int i=0;i<tanks.size();i++){
        if(! i==index){
            double xindex[4];
            double yindex[4];
            tanks[index]->getCorners(xindex, yindex);
            double xi[4];
            double yi[4];
            tanks[i]->getCorners(xi, yi);
            if(shapescrossed(xindexo, yindexo, xindex, yindex, 4, xi, yi, 4))
                return true;
        }
    }*/
    for(int i=0;i<walls.size();i++){
        if(checkcollision(walls[i], tanks[index]->getAngleRect())){
            std::cout << "Tank: " << index << "\n";
            return true;
        }
    }
    return false;
}

void sortpoints(double* xs, double* ys, int length){
    double x[length];
    double y[length];
    bool mark[length];
    double x0 = 0;
    double y0 = 0;
    for(int i=0;i<length;i++){
        x0+=xs[i];
        y0+=ys[i];
        mark[i] = true;
    }
    x0/=length; //middle point
    y0/=length;
    int i = 0;
    bool endloop = false;
    while(!endloop){
//        std::cout << "i: " << i << "\n";
        int smallest = -1;
        for(int j=0;j<length;j++){
            if(mark[j]){
                if(xs[j] >= x0){
//                    std::cout << "angle of " << j << ": " << atan((ys[j]-y0)/(xs[j]-x0)) << "\n";
                    if(smallest == -1)
                        smallest = j;
                    else
                        if(atan((ys[j]-y0)/(xs[j]-x0)) < atan((ys[smallest]-y0)/(xs[smallest]-x0)))
                            smallest = j;
                }
            }
        }
//        std::cout << "Smallest: " << smallest << "\n";
        if(smallest == -1)
            endloop = true;
        else{
            x[i] = xs[smallest];
            y[i] = ys[smallest];
            mark[smallest] = false;
            i++;
        }
    }
//    std::cout << "Part 2\n";
    for(i;i<length;i++){
//        std::cout << "i: " << i << "\n";
        int smallest = -1;
        for(int j=0;j<length;j++){
            if(mark[j]){
//                std::cout << "J: " << j << "\n";
                if(xs[j] < x0){
//                    std::cout << "angle of " << j << ": " << atan((ys[j]-y0)/(xs[j]-x0)) << "\n";
                    if(smallest == -1)
                        smallest = j;
                    else
                        if(atan((ys[j]-y0)/(xs[j]-x0)) < atan((ys[smallest]-y0)/(xs[smallest]-x0)))
                            smallest = j;
                }
            }
        }
        x[i] = xs[smallest];
        y[i] = ys[smallest];
        mark[smallest] = false;
    }
    for(int i=0;i<length;i++){
        xs[i] = x[i];
        ys[i] = y[i];
    }
}

bool isInShape(double x, double y, Shape shape){
    bool isin = true;
//    std::cout << "x: " << x << "\ny: " << y << "\n";
//    std::cout << "x0: " << shape.x[0] << "\ny0: " << shape.y[0] << "\n";
//    std::cout << "x1: " << shape.x[1] << "\ny1: " << shape.y[1] << "\n";
//    std::cout << "xE: " << shape.x[shape.points-1] << "\nyE: " << shape.y[shape.points-1] << "\n";
//    double oE = atan2(shape.x[0], shape.y[0], shape.x[shape.points-1], shape.y[shape.points-1]);
//    double o1 = atan2(shape.x[0], shape.y[0], shape.x[1], shape.y[1]);
//    double oT = atan2(shape.x[0], shape.y[0], x, y);
//    std::cout << "0E " << oE << "\n01 " << o1 << "\n0T " << oT << "\n";
//    if(Ip - Im > PI)
//        Ip -= 2.0 * PI;
//    if(oT < oE == oT < o1)
//        isin = false;

//    double E0 = atan2(shape.x[shape.points-1], shape.y[shape.points-1], shape.x[0], shape.y[0]);
//    double E1 = atan2(shape.x[shape.points-1], shape.y[shape.points-1], shape.x[shape.points-2], shape.y[shape.points-2]);
//    double ET = atan2(shape.x[shape.points-1], shape.y[shape.points-1], x, y);
//    std::cout << "E0 " << E0 << "\nE1 " << E1 << "\nET " << ET << "\n";
//    if(Ip - Im > PI)
//        Ip -= 2.0 * PI;
//    if(ET < E0 == ET < E1)
//        isin = false;

    for(int i=0; i<shape.points; i++){
        int im = i-1;
        if(im<0)
            im=shape.points-1;
        int ip = i+1;
        if(ip==shape.points)
            ip=0;
        double Im = atan2(shape.x[i], shape.y[i], shape.x[im], shape.y[im]);
        double Ip = atan2(shape.x[i], shape.y[i], shape.x[ip], shape.y[ip]);
        double IT = atan2(shape.x[i], shape.y[i], x, y);
        if(Ip - Im > PI)
            Ip -= 2.0 * PI;
        if((IT < Im == IT < Ip) && (IT-2.0*PI < Im == IT-2.0*PI < Ip)){
            isin = false;
            std::cout << "Im " << Im << "\nIp " << Ip << "\nIT " << IT << "\n";
        }
    }
    if(isin)
        std::cout << "collision? maybe?\n";
    return isin;
}

double atan2(double x1, double y1, double x2, double y2){
    double t = atan((x1-x2)/(y1-y2));
    if(x2 > x1)
        if(y2 > y1)
            t = PI - t;
        else
            t = 0 - t;
    else
        if(y2 > y1)
            t = PI - t;
        else
            t = 2.0 * PI - t;
    return t;
}
//}

//}
