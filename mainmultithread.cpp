#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
//#include <stdlib.h>
#include <sstream>
//#include <memory>
//#include <pthread.h>


/*TODO:
pathfinding
 - 2d array of booleans representing small parts of the map, then use dead end filling  - more efficient maybe???
 - recursive algorithm, saving all combinations, delete ones that dont work, use shortest one
AI - suicidally tracks down player

character struct:
    names of unlocked weapons


weapon struct: - loaded from file (this is a template)
    name
    look
    behaviour
disable projectile stepping while frozen
use std::shared_ptr
minimap, and fullscreen
upgrade system with dope menu
weapon selection - choose two weapons at once, each weapon has own upgrades + overall upgrades
when levelling up - get a few upgrade points. One upgrade available is weapon systems, gives one upgrade point for each weapon,
so all weapons remain useful in later game.

IDEAS:
    rooms - every object has an additional number indicating the room
    enemy AI only runs for enemies in the same room, but projectiles and traileffects run
    exits - defined as two points on the edge of two rooms, map edge is modified so you can travel through them
    teleport locations - purchased with "materials", can teleport to these
    can only use an exit/teleport if no "heat" - all enemies are at the last seen position (add in check for enemies that clears position if velocity is too low - if obstructed)
        reason - don't want to carry heat through exit / don't want the enemy to know you have access to teleporting
    create GUI element showing if there is "heat"
    materials are gained by killing enemies - two amounts, onperson, and safe. onperson is added to safe upon clearing heat, onperson is lost if killed.
    upon respawning, materials some materials are spent, if not enough, can't respawn in vehicle mode, must play person mode until enough materials are gathered
    long range teleporters at end of each area - cost significant materials to unlock, plus possibly special items
    upon killing enemies xp is gained, xp also gained on death as "what not to do next time"
    respawning as person is free
    why can respawning happen (in game explanation):
        tank control is well protected, if tank is "killed", player simply teleports out, and enemies don't care cos they think you're dead
        after body is disposed of / people leave the area, you are teleported back to base and revived
    enemies have maximum distance they can see, and broadcast. If you are in vision range they broadcast your location to all tanks within broadcast range
    all enemies have a respawn timer value - set when killed, if player is in different room and timer is 0, enemy respawns
    don't load player from a template, load from character file, which is completely independent of the map, so if map/tank template is modded/changed, player can be loaded
    load tank from template by ID as well as name
    multithreading?
    fix simulation speed relative changes
    fix no response to exit signal
*/

//{DECLARATIONS

//{CONSTANTS & VARIABLES
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 960;
const double PI = 3.14159265358979323846;

int mapwidth;
int mapheight;
int mouseX = 0;
int mouseY = 0;
int mouseXfreeze = 0;
int mouseYfreeze = 0;
double camX = 0;
double camY = 0;
bool LmouseState = 0;
bool LmouseClick = 0;
bool RmouseState = 0;
bool RmouseClick = 0;
SDL_Window* gWindow;
SDL_Renderer* gRenderer = NULL;
TTF_Font *gFont = NULL;
double gamespeed = 2;   //actual speed of game
double stepSize = 0.01; //speed of game - adjusted based on framerate
double mul = 2;     //scale of drawn graphics
double TICKS_PER_FRAME;
double MAX_FPS = 60;    //maximum framerate
double FPS = MAX_FPS;
double TICKS_PER_FRAME2;
double MAX_FPS2 = 60;
double FPS2 = MAX_FPS2;
double streakrenderres = 0.01;  //how accurately the streak left behind a projectile will be rendered close to a hit target - smaller = more accurate
float slomospeed = 1;
float maxslomospeed = 16;
int physics = 3;
double scanangleincrement = 0.01; //increment when searching for a line of sight to a target, larger = lower performance
double circleres = 10;  //number of points generated in a circular shape, higher = more accurate collision detection, lower performance
bool particleEffects = false;
bool pause = false;
const Uint8* keys = SDL_GetKeyboardState( NULL );
uint32_t frameCount = 0;
double traileffectprescision = 15;
uint32_t effectIDs = 0;
bool freeze = false;
double slomo=100;
bool aiminglaser = true;
int fontSize = 28;
bool showSecondary = false;//display secondary weapon stats in quick menu
bool graphicsthreadrun = true;
bool graphicsthreadpause = false;
uint32_t lastpause;
uint32_t pauseduration;
bool recentpause = false;

//bool damagefx = false;
//double walldrag = 0.9;
//{controls:
SDL_Scancode fowardsKey = SDL_SCANCODE_W;
SDL_Scancode backwardsKey = SDL_SCANCODE_S;
SDL_Scancode leftKey = SDL_SCANCODE_A;
SDL_Scancode rightKey = SDL_SCANCODE_D;
SDL_Scancode brakeKey = SDL_SCANCODE_SPACE;
SDL_Scancode slomoKey = SDL_SCANCODE_LSHIFT;
SDL_Scancode quickMenuKey = SDL_SCANCODE_LCTRL;
SDL_Scancode pauseKey = SDL_SCANCODE_ESCAPE;
SDL_Scancode primaryWeaponAssistKey = SDL_SCANCODE_LALT;
bool primaryWeaponAssistKeyState = false;
bool primaryWeaponAssistToggle = false;

//}

//}

//{STRUCTS
struct PercentRect{ //used for clipping images
	double x;
	double y;
	double w;
	double h;
};

struct ParticleLine{
    double x1;
    double y1;
    double x2;
    double y2;
    double r = 0xFF;
    double g = 0;
    double b = 0;
    double a = 0xFF;
    double t = 0;
    double dr = 0;
    double dg = 0;
    double db = 0;
    double da = -0.05;
    double dt = 0.05;
    uint32_t id=0;
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
    SDL_Rect* clip;              //clip of look to be rendered
    double damage;
    bool passesShield=false;
    ParticleLine* traileffect;

    int dist=0;//steps travelled on current trail effect segment
    uint32_t lastid=0;

    int ownerID;
    bool primaryWeapon;
};

struct Shape{
    std::vector<double> x;
    std::vector<double> y;
};

struct threaddata {
	std::vector<Projectile*>* projectiles;
};
//}

//{ENUMERATIONS
enum command { ACCELERATE, BRAKE, TURN, TURNTURRET, BRAKETURRET };
//}

//{CLASSES
class Tank{
	public:
        Tank();
		Tank(std::string, double, double, double);
		void positionReset(std::string, double, double, double, bool);
		~Tank();
		void step();
		void render();
		void free();
		void loadLook0();
		void loadLook(std::string, std::string, std::string, std::string, std::string, PercentRect*, PercentRect*, PercentRect*, PercentRect*, PercentRect*, SDL_Rect*, SDL_Rect*, SDL_Rect*, SDL_Rect*, SDL_Rect*);
		void setDimensions(double, double, double, double, double, double, double, double, double);
		void setOther(int, double, double, double, double, double, double);
		void setWeapons(double, double, double, double, int, int, double, double, int, int, ParticleLine, ParticleLine, bool, bool, double, double);
		bool primaryReloaded();
		bool secondaryReloaded();
		void setHealth(double, double, double, int);
		void takeDamage(double, bool);
		bool isDead();
		double getLength();
		int getID();

				//commands
        void settargetvel(double);
        void pointTo(double, double);
        void shootPrimary(Projectile*);
        void shootSecondary(Projectile*);
        void accelerate(double);	//accelerate with power
		void brake(double);		//brake with force
		void turn(double);		//set power distribution between tracks (to steer)
				//accessors
        bool hasseentarget();
        double getHealth();
        double getMaxHealth();
        double getShield();
        double getMaxShield();
        int getShieldDelay();
        double getShieldRegen();
		double getX();			//get x position
		double getY();			//get y position
		double getXvel();
		double getYvel();
		double getVel();
		double getRot();		//get rotation of tank
		double getTurretrot();		//get rotation of turret
		int getPburst();    //projectiles fired per shot
		int getSburst();
		std::string getName();
		void setName(std::string);
		int getIndex();
		void setIndex(int);
		Shape getShape();
		void setIsCircle(bool);
		bool isACircle();
		int getPrimaryROF();     //reload time in game steps
		int getSecondaryROF();
		double getPrimaryReload();  //time spent reloading
		double getSecondaryReload();
		double getPrimaryDamage();
		double getSecondaryDamage();
		double getPspread();
		double getSspread();
		double getMaxVel();
		double getAcceleration();
		bool getpPassesShield();
		bool getsPassesShield();
		double getEnergy();
		double getMaxEnergy();
		double getEnergyGen();
		double getPEnergy();
		double getSEnergy();
		void getProjectileLook(SDL_Texture**, SDL_Rect**, ParticleLine**, bool);
		//saving
		void save(std::ofstream*);
		void load(std::ifstream*);
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
		//double tankrotvel;		//rotational velocity of tank
		double turretrot;		//rotation of turret relative to tank
		//double turretvel;		//rotational velocity of turret relative to tank
		double targetX = 0;
		double targetY = 0;
		Shape shape;
		double targetvel;
				//specs
					//mass
		double mass;			//mass of tank body
					//size
        bool iscircle = false;
		double length;			//length of body
		double width;			//width of body
		//double trackwidth;		//width of tracks
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
		double maxvel;          //maximum velocity
		double energy;
		double maxEnergy;
		double energyGen;
                    //weapons
        double primaryDamage;
        double secondaryDamage;
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
		ParticleLine Peffect;
		ParticleLine Seffect;
		bool pPassesShield=false;
		bool sPassesShield=false;
        double pEnergy;
        double sEnergy;

        //health
        double maxHealth;
        double maxShield;
        double shieldRegen;
        int shieldDelay;
        double health;
        double shield;
        double shieldCount=0;
        bool dead=false;

        int ID; //ID assigned in tanktemplates file
        int index;  //index of tank in
        std::string name;
		double commands [5] = {0, 0, 0, 0, 0};		//array containing values of commands passed to tank
};
//}

//{METHODS
void init();
void loadMedia();
bool loadSettings(char*);
void loadTankTemplates(char*);
void loadTanks(char*);
void setMap(char*);
int* loadMap(std::vector<Shape*>*, char*);
uint8_t mainMenu();
void addButton(std::vector<Shape*>*, double, double, double, double);
uint8_t runGame(std::string*);
void close();
bool updateMouse();
void handleInput(Tank*);	//Reads keyboard input, and sends controls to the Tank passed in
bool stepProjectile(Projectile*);
bool cansee(int, int, double*, double*);
double yfromx(double, double, double, double, double);
double yfromx(double, double, double, double, double);
void fire(bool, bool, Tank*, std::vector<Projectile*>*);
bool collision(int, double*, bool*);
bool passedovershape(double, double, double, double, Shape);
bool passedover(double, double, double, double, double, double, double, double);
void sortpoints(std::vector<double>*, std::vector<double>*);
bool isInShape(double, double, Shape);
double atan2(double, double, double, double);
bool checkcollision(Shape, Shape, double*, bool*);
int findSide(double, double, Shape);
double nearestpoint(double, double, double, double, double, double);
double distanceto(double, double, double, int, double, double);
void impactcoords(double, double, double, int, double, double, double*, double*);
int SCROLL_RenderDrawLine(SDL_Renderer*, int, int, int, int);
uint8_t pauseMenu(std::string*);
void saveMenu(std::string*);
std::string loadMenu();
void basicText(std::string, SDL_Color, int, int, double, double, bool, uint8_t, uint8_t, uint8_t, uint8_t);

void saveParticleLine(ParticleLine, std::ofstream*);
ParticleLine loadParticleLine(std::ifstream*);
void saveShape(Shape, std::ofstream*);
Shape loadShape(std::ifstream*);
void saveProjectile(Projectile, std::ofstream*);
Projectile loadProjectile(std::ifstream*);
void saveTank(Tank, std::ofstream*);
Tank loadTank(std::ifstream*);

void *thread(void*);
//}

//{CONSTANTS AND VARIABLES RELYING ON THE ABOVE
Tank* showDetailsFor = NULL;
std::vector<Tank*> tanks;
std::vector<Tank> tankTemplates;
std::vector<Shape*> shapes;
std::vector<ParticleLine> particlelines;
//}

//}

//{DEFINITIONS

//{TANK

void Tank::step(){
    if(!dead){
    bool gradset = false;
    bool* gradsetp = &gradset;
    double grad;
    double* collisiongrad = &grad;
    if(iscircle){
        double dist = sqrt((targetX-x)*(targetX-x) + (targetY-y)*(targetY-y));
        if(targetvel==0){
            if(dist < length/2 && vel > targetvel){
                vel = 0;
                commands[BRAKE] = 0;
                commands[ACCELERATE] = 0;
            }
        }else{
            if(dist < (length/2+50)){// && vel > targetvel){
                commands[BRAKE] = 100;
                commands[ACCELERATE] = 0;
            }
        }
    }
    if(primaryReload<primaryROF+1)
        primaryReload+=30*200.0*stepSize;
    if(secondaryReload<secondaryROF+1)
        secondaryReload+=30*200.0*stepSize;
    if(shieldCount<=shieldDelay)
        shieldCount+=stepSize;
    if(shieldCount>shieldDelay)
        shield+=shieldRegen*stepSize;
    if(shield>maxShield)
        shield=maxShield;
    if(energy<maxEnergy)
        energy+=energyGen*stepSize;
    else
        energy=maxEnergy;
    if(iscircle){
        tankrot = (atan((targetY - y)/(targetX - x))*180/PI);
        targetX < x ? tankrot += 180 : tankrot;
    }
    turretrot = (atan((targetY - y)/(targetX - x))*180/PI) - tankrot;
    targetX > x ? turretrot += 180 : turretrot;
	tankrot += commands[TURN] * stepSize * sqrt(fabs(vel)+1) / 10;
	if(collision(index, collisiongrad, gradsetp)){
        tankrot -= commands[TURN] * stepSize * sqrt(abs(vel)+1) / 10;
        std::cout << "Collision Detected...\n";
	}
	tankrot > 360 ? tankrot -= 360 : (tankrot < 0 ? tankrot += 360 : tankrot = tankrot);
	double tankrotrad = (tankrot) * PI / 180;    //convert from visual rotation to rotation for calculation
	double usedpower = commands[ACCELERATE] * power / 100;  //calculate power used
	double drag = vel * vel / 10;   //drag so that tank can coast to a stop
	double braking = commands[BRAKE] * brakeforce / 100 + 10 + drag;   //strength of brakes being applied
	double thrust = usedpower / ( abs(vel) + 0.1 );	//calculate thrust at current velocity
	thrust > usedpower/10 ? thrust = usedpower/10 : thrust = thrust;
	thrust < usedpower/10 ? thrust = usedpower/10 : thrust = thrust;
	double dvel;
											//calculate acceleration provided to tracks by engine and brakes
	if( abs( thrust ) > braking ){							//tracks are powered by the engine
		vel > 0 ? thrust -= braking : thrust += braking;			//calculate net thrust
		dvel = stepSize * thrust / mass;				//calculate acceleration
	}else{										//tracks are braking
		thrust = braking - abs(thrust);						//calculate net braking
		vel > 0 ? dvel = -1 * stepSize * thrust / mass : dvel = stepSize * thrust / mass;				//calculate acceleration
	}
    vel += dvel;
    if(vel > maxvel)
        vel = maxvel;
    else if(fabs(vel) > maxvel)
        vel = -maxvel;
    xvel = cos(tankrotrad)*vel;
    yvel = sin(tankrotrad)*vel;
    x+=stepSize*xvel;
    y+=stepSize*yvel;
    if(collision(index, collisiongrad, gradsetp)){
            y-=stepSize*yvel;
            x-=stepSize*xvel;
        bool solved = false;
        std::cout << "Collision Detected...\n";
        double maxangle;
        iscircle ? maxangle = PI : maxangle = PI/2.0;
        for(double i=0.1; i<maxangle; i+=0.1){
            x+=stepSize*cos(tankrotrad+i)*vel;
            y+=stepSize*sin(tankrotrad+i)*vel;
            if(collision(index, collisiongrad, gradsetp)){
                x-=stepSize*cos(tankrotrad+i)*vel;
                y-=stepSize*sin(tankrotrad+i)*vel;
                x+=stepSize*cos(tankrotrad-i)*vel;
                y+=stepSize*sin(tankrotrad-i)*vel;
                if(collision(index, collisiongrad, gradsetp)){
                    x-=stepSize*cos(tankrotrad-i)*vel;
                    y-=stepSize*sin(tankrotrad-i)*vel;
                }else{
                    solved = true;
                    x-=stepSize*cos(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                    y-=stepSize*sin(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                    if(collision(index, collisiongrad, gradsetp)){
                        x+=stepSize*cos(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                        y+=stepSize*sin(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                    }
                    if(!iscircle)
                        vel*=sqrt(fabs(cos(i)));
                    i=PI;
                }
            }else{
                solved = true;
                x-=stepSize*cos(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                y-=stepSize*sin(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                if(collision(index, collisiongrad, gradsetp)){
                    x+=stepSize*cos(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                    y+=stepSize*sin(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                }
                if(!iscircle)
                    vel*=sqrt(fabs(cos(i)));
                i=PI;
            }
        }
        if(!solved)
            vel/=2;
	}
}}

void Tank::pointTo(double targx, double targy){
    targetX = targx;
    targetY = targy;
}

void Tank::shootPrimary(Projectile* p){
    primaryReload = 0;
    energy-=pEnergy;
    if(energy<0)
        energy=0;
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
    p->clip = &pprojclip;

    p->traileffect = &Peffect;
    p->damage = primaryDamage;
    p->passesShield = pPassesShield;

    p->ownerID=ID;
    p->primaryWeapon=true;
}

void Tank::shootSecondary(Projectile* p){
    secondaryReload = 0;
    energy-=sEnergy;
    if(energy<0)
        energy=0;
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
    p->clip = &sprojclip;

    p->traileffect = &Seffect;
    p->damage = secondaryDamage;
    p->passesShield = sPassesShield;

    p->ownerID=ID;
    p->primaryWeapon=false;
}

Tank::Tank(){};

Tank::Tank(std::string nname, double xpos, double ypos, double rot){
    if(nname==""){
        nname="unnamed";
    }
	positionReset(nname, xpos, ypos, rot, true);
}

void Tank::positionReset(std::string nname, double xpos, double ypos, double rot, bool reset){
	if(nname!="")
        name = nname;
	if(xpos>=0)
        x = xpos;
	if(ypos>=0)
        y = ypos;
	if(rot>=0)
        tankrot = rot;
    if(reset){
        turretrot = 0;
        xvel = 0;
        yvel = 0;
        vel = 0;
	}
}

Tank::~Tank(){
	//free();
}

void Tank::setOther(int nID, double npower, double nbrakeforce, double nmaxvel, double nmass, double nmaxEnergy, double nenergyGen){
	if( npower > 0 )
		power = npower;
	if( nbrakeforce > 0 )
		brakeforce = nbrakeforce;
	if( nmaxvel > 0 )
        maxvel = nmaxvel;
    if( nmass > 0)
        mass = nmass;
    if( nmaxEnergy > 0)
        maxEnergy = nmaxEnergy;
    energy = maxEnergy;
    if( nenergyGen > 0)
        energyGen = nenergyGen;
    ID=nID;
}

void Tank::setWeapons(double nPdamage, double nSdamage, double nPprojectileSpeed, double nSprojectileSpeed, int nprimaryROF, int nsecondaryROF, double npspread, double nsspread, int npburst, int nsburst, ParticleLine nPeffect, ParticleLine nSeffect, bool npPassesShield, bool nsPassesShield, double npEnergy, double nsEnergy){
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
    Peffect = nPeffect;
    Seffect = nSeffect;
    primaryDamage=nPdamage;
    secondaryDamage=nSdamage;
    pPassesShield=npPassesShield;
    sPassesShield=nsPassesShield;
    pEnergy=npEnergy;
    sEnergy=nsEnergy;
}

void Tank::setDimensions(double nlength, double nwidth, double nturretradius, double nbarrellength, double nbarrelwidth, double npprojlength, double npprojwidth, double nsprojlength, double nsprojwidth){
	if( nlength != 0 )
		length = nlength*mul;
	if( nwidth != 0 )
		width = nwidth*mul;
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

void Tank::setHealth(double nmaxHealth, double nmaxShield, double nshieldRegen, int nshieldDelay){
    maxHealth=nmaxHealth;
    maxShield=nmaxShield;
    shieldRegen=nshieldRegen;
    shieldDelay=nshieldDelay;
    health=maxHealth;
    shield=maxShield;
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
    loadLook("data/graphics/body.png", "data/graphics/turret.png", "data/graphics/barrel.png", "data/graphics/primaryprojectile.png", "data/graphics/secondaryprojectile.png", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}

void Tank::loadLook(std::string bodypath = "data/graphics/body.png", std::string turretpath = "data/graphics/turret.png", std::string barrelpath = "data/graphics/barrel.png", std::string pprojpath = "data/graphics/projectile.png", std::string sprojpath = "data/graphics/secondaryprojectile.png", PercentRect* bodysource = NULL, PercentRect* turretsource = NULL, PercentRect* barrelsource = NULL, PercentRect* pprojsource = NULL, PercentRect* sprojsource = NULL, SDL_Rect* bodystretchn = NULL, SDL_Rect* turretstretchn = NULL, SDL_Rect* barrelstretchn = NULL, SDL_Rect* pprojstretchn = NULL, SDL_Rect* sprojstretchn = NULL){
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
	SDL_Rect temp = {x-width/2-camX, y-length/2-camY, width, length};						//x-width/2 is used, so that the x and y coordinates refer to the center of the tank
	SDL_RenderCopyEx( gRenderer, body, &bodyclip, &temp, tankrot+90, NULL, SDL_FLIP_NONE);

	temp = {x-turretradius-camX, y-turretradius-camY, turretradius*2, turretradius*2};
	SDL_RenderCopyEx( gRenderer, turret, &turretclip, &temp, tankrot+turretrot+90, NULL, SDL_FLIP_NONE);

	SDL_Point center = {barrelwidth/2, 1-turretradius};
	temp = {x-barrelwidth/2-camX, y+turretradius-camY, barrelwidth, barrellength};
	SDL_RenderCopyEx( gRenderer, barrel, &barrelclip, &temp, tankrot+turretrot+90, &center, SDL_FLIP_NONE);
}

void Tank::takeDamage(double damage, bool shieldpass){
    if(!shieldpass){
        if(shield>0)
            shieldCount=0;
        if(shield>damage)
            shield-=damage;
        else{
            damage-=shield;
            shield=0;
            if(health>damage)
                health-=damage;
            else{
                dead=true;
                health=0;
            }    //call death animation
        }
    }else{
        if(health>damage)
            health-=damage;
        else{
            dead=true;
            health=0;
            shield=0;
        }    //call death animation
    }
}

void Tank::load(std::ifstream* file){
	std::string line;
	getline(*file,line);
	x=atof(line.c_str());
	getline(*file,line);
	y=atof(line.c_str());
	getline(*file,line);
	xvel=atof(line.c_str());
	getline(*file,line);
	yvel=atof(line.c_str());
	getline(*file,line);
	vel=atof(line.c_str());
	getline(*file,line);
	tankrot=atof(line.c_str());
	getline(*file,line);
	turretrot=atof(line.c_str());
	getline(*file,line);
	targetX=atof(line.c_str());
	getline(*file,line);
	targetY=atof(line.c_str());
	getline(*file,line);
	targetvel=atof(line.c_str());
	getline(*file,line);
	energy=atof(line.c_str());
	getline(*file,line);
	primaryReload=atof(line.c_str());
	getline(*file,line);
	secondaryReload=atof(line.c_str());
	getline(*file,line);
	health=atof(line.c_str());
	getline(*file,line);
	shield=atof(line.c_str());
	getline(*file,line);
	shieldCount=atof(line.c_str());
	getline(*file,line);
	if(line.compare("t")==0)
		dead=true;
	else
		dead=false;
	getline(*file,line);
	index=atoi(line.c_str());
}

void Tank::save(std::ofstream* file){
    std::ostringstream strs;
    strs << x << "\n";
    strs << y << "\n";
    strs << xvel << "\n";
    strs << yvel << "\n";
    strs << vel << "\n";
    strs << tankrot << "\n";
    strs << turretrot << "\n";
    strs << targetX << "\n";
    strs << targetY << "\n";
    strs << targetvel << "\n";
    strs << energy << "\n";
    strs << primaryReload << "\n";
    strs << secondaryReload << "\n";
    strs << health << "\n";
    strs << shield << "\n";
    strs << shieldCount << "\n";
    if(dead)
        strs << "t\n";
    else
        strs << "f\n";
    strs << index << "\n";
    *file << strs.str();
}

//{those methods

void Tank::getProjectileLook(SDL_Texture** look, SDL_Rect** clip, ParticleLine** traileffect, bool primary){
    if(primary){
        *look = pproj;
        *clip = &pprojclip;
        *traileffect = &Peffect;
    }else{
        *look = sproj;
        *clip = &sprojclip;
        *traileffect = &Seffect;
    }
}

int Tank::getID(){
    return ID;
}

double Tank::getPEnergy(){
    return pEnergy;
}

double Tank::getSEnergy(){
    return sEnergy;
}

double Tank::getEnergyGen(){
    return energyGen;
}

double Tank::getEnergy(){
    return energy;
}

double Tank::getMaxEnergy(){
    return maxEnergy;
}

bool Tank::getpPassesShield(){
    return pPassesShield;
}

bool Tank::getsPassesShield(){
    return sPassesShield;
}

double Tank::getMaxVel(){
    return maxvel;
}

double Tank::getAcceleration(){
    return power/mass;
}

double Tank::getPspread(){
    return pspread;
}

double Tank::getSspread(){
    return sspread;
}

double Tank::getPrimaryDamage(){
    return primaryDamage;
}
double Tank::getSecondaryDamage(){
    return secondaryDamage;
}

int Tank::getShieldDelay(){
    return shieldDelay;
}

double Tank::getShieldRegen(){
    return shieldRegen;
}

int Tank::getPrimaryROF(){     //reload time in game steps
    return primaryROF;
}
int Tank::getSecondaryROF(){
    return secondaryROF;
}
double Tank::getPrimaryReload(){  //time spent reloading
    return primaryReload;
}
double Tank::getSecondaryReload(){
    return secondaryReload;
}

double Tank::getLength(){
    return length;
}

bool Tank::isDead(){
    return dead;
}

double Tank::getHealth(){
    return health;
}
double Tank::getMaxHealth(){
    return maxHealth;
}
double Tank::getShield(){
    return shield;
}
double Tank::getMaxShield(){
    return maxShield;
}

bool Tank::hasseentarget(){
    return targetX!=0 && targetY!=0;
}

void Tank::settargetvel(double n){
    targetvel = n;
}

double Tank::getVel(){
    return vel;
}

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

void Tank::setIsCircle(bool n){
    iscircle = n;
}

bool Tank::isACircle(void){
    return iscircle;
}

Shape Tank::getShape(){
    Shape temp;
    std::vector<double> xvec;
    std::vector<double> yvec;
    if(!iscircle){
    double tankrotrad = tankrot * PI / 180;
    xvec.push_back(x+cos(tankrotrad)*length/2+sin(tankrotrad)*width/2);    //front left
    yvec.push_back(y+sin(tankrotrad)*length/2-cos(tankrotrad)*width/2);
    xvec.push_back(x+cos(tankrotrad)*length/2-sin(tankrotrad)*width/2);    //front right
    yvec.push_back(y+sin(tankrotrad)*length/2+cos(tankrotrad)*width/2);
    xvec.push_back(x-cos(tankrotrad)*length/2+sin(tankrotrad)*width/2);    //back left
    yvec.push_back(y-sin(tankrotrad)*length/2-cos(tankrotrad)*width/2);
    xvec.push_back(x-cos(tankrotrad)*length/2-sin(tankrotrad)*width/2);    //back right
    yvec.push_back(y-sin(tankrotrad)*length/2+cos(tankrotrad)*width/2);
    }else{
    for(double i = 0; i<1; i+=1.0/circleres){
        xvec.push_back(x + cos(2*PI*i)*length/2);
        yvec.push_back(y + sin(2*PI*i)*length/2);
    }
    }
    sortpoints(&xvec, &yvec);
    temp.x = xvec;
    temp.y = yvec;
    return temp;
}

int Tank::getPburst(){
    return pburst;
}

int Tank::getSburst(){
    return sburst;
}

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

void Tank::accelerate(double percent = 100){
	commands[ACCELERATE] = percent;
}

void Tank::brake(double percent = 100){
	commands[BRAKE] = percent;
}

void Tank::turn(double diff = 0){
	commands[TURN] = diff;
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
    delete gWindow;
    delete gRenderer;
	SDL_Init( SDL_INIT_VIDEO );
	SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" );
	gWindow = SDL_CreateWindow( "Game", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
	gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED );
	SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	IMG_Init( IMG_INIT_PNG );
	SDL_SetRenderDrawBlendMode(gRenderer, SDL_BLENDMODE_BLEND);
    TTF_Init();
	TICKS_PER_FRAME = 1000/MAX_FPS;
	TICKS_PER_FRAME2 = 1000/MAX_FPS2;
	stepSize = gamespeed/(MAX_FPS*physics);
    delete gFont;
	gFont = TTF_OpenFont( "data/graphics/font.ttf", fontSize );
	if( gFont == NULL ){
        std::cout << "error\n";
        std::cout << TTF_GetError();
        printf( "Failed to load lazy font! SDL_ttf Error: %s\n", TTF_GetError() );
        SDL_Delay(10000);
    }

}

void close(){
	SDL_DestroyRenderer( gRenderer );
	SDL_DestroyWindow( gWindow );
	gWindow = NULL;
	gRenderer = NULL;
	IMG_Quit();
	SDL_Quit();
}

int main( int argc, char* args[] ){
	init();
	uint8_t option = 1;
	if(loadSettings("data/game/settings.txt")){//if no menu skip, show menu
        loadTankTemplates("data/game/tanktemplates.txt");
        loadTanks("data/game/tanks.txt");
        setMap("data/game/map1.txt");
        runGame(NULL);
        close();
        return 0;
    }
    while(true){
        switch (option){
            case 0:
                close();
                return 0;
            break;
            case 1:
                option = mainMenu();
            break;
            case 2:
                option = runGame(NULL);
            break;
            case 3:
                std::string load = loadMenu();
                option = runGame(&load);
            break;
        }
    }
	close();
	return 0;
}

uint8_t runGame(std::string* loadfrom){
	std::vector<Projectile*> projectiles;
    uint32_t frameTimer;
	bool quit = false;
	SDL_Event e;
	std::vector<uint32_t> frames;
    pthread_t thread1;	//set up multithreading
    pthread_attr_t attr;
	pthread_attr_init(&attr); //initialise and set thread joinable
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	threaddata threaddata1;
    for(double i=0; i<1000; i+=1000/MAX_FPS){
        frames.push_back(SDL_GetTicks()-1000+i);
    }
	lastpause = 0;
	pauseduration = 0;
	recentpause = false;
    if(loadfrom!=NULL){
        std::string line;
        std::ifstream loadFile (*loadfrom);
        if (loadFile.is_open()){
            while ( getline (loadFile,line) ){
                if(line.compare("newparticleline")==0)
                    particlelines.push_back(loadParticleLine(&loadFile));
                else if(line.compare("newprojectile")==0){
                    Projectile* temp = new Projectile();
                    *temp = loadProjectile(&loadFile);
                    projectiles.push_back(temp);
                }else if(line.compare("newtank")==0){
                    Tank* temp = new Tank();
                    *temp = loadTank(&loadFile);
                    tanks.push_back(temp);
                }else if(line.compare("newshape")==0){
                    Shape* temp = new Shape();
                    *temp = loadShape(&loadFile);
                    shapes.push_back(temp);
                }
                std::cout << line << '\n';
            }
        }else
            std::cout << "Unable to open file\n";
    }
    //set camera before game runs to prevent a "jump"
    mouseXfreeze=mouseX;//only mouseX is kept up to date whilst outside of game loop
    mouseYfreeze=mouseY;
    camX=((tanks[0]->getX()+mouseXfreeze)/2)-SCREEN_WIDTH/2.0;
    camY=((tanks[0]->getY()+mouseYfreeze)/2)-SCREEN_HEIGHT/2.0;
    if(camX<-100)
        camX=-100;
    else if (camX>mapwidth+100-SCREEN_WIDTH)
        camX=mapwidth+100-SCREEN_WIDTH;
    if(camY<-100)
        camY=-100;
    else if (camY>mapheight+100-SCREEN_HEIGHT)
        camY=mapheight+100-SCREEN_HEIGHT;

	threaddata1.projectiles = &projectiles;
	pthread_create(&thread1, &attr, thread, (void *)&threaddata1);
	while( !quit ){      //DONT FORGET - NEEDS TO BE !quit
        frameTimer = SDL_GetTicks();
        //Shape* t = shapes[1];
        //Calculate FPS
        frames.push_back(SDL_GetTicks());
        for(int i=0;i<frames.size();i++){
            if(frames[i] + 1000 < SDL_GetTicks()){
                if(recentpause){
                    if(SDL_GetTicks()-lastpause<1000){//check again if recently paused
                        if(frames[i]+1000+pauseduration<SDL_GetTicks())
                            frames.erase(frames.begin()+i);
                    }else{
                        recentpause=false;
                        frames.erase(frames.begin()+i);
                    }
                }else{
                    frames.erase(frames.begin()+i);
                }
            }
        }
        FPS = frames.size();
        stepSize = gamespeed/(FPS*physics*slomospeed);
        std::cout << "STEPSIZE " << stepSize << "\n";
        if(freeze)
            stepSize=0;
        int tanknum = -1;
        for(int phys = 0; phys < physics; phys++){
            //{Handle input
            quit = updateMouse();
            handleInput(tanks[0]);
            if(!freeze&&!tanks[0]->isDead()){//unfreeze mouse variable, mousefreeze is always up to date, mouseX is only updated whilst game is running
                mouseX=mouseXfreeze;
                mouseY=mouseYfreeze;
            }
            if(pause){
                pause = false;
                pauseduration = SDL_GetTicks();
                std::string* saveto = new std::string("");
                switch (pauseMenu(saveto)){//pause menu returns 0 if returning to main menu, 1 if saving, otherwise continue
                    case 0:
                        for(int i=0;i<tanks.size();i++){
                            delete tanks.at(i);
                        }
                        tanks.clear();
                        for(int i=0;i<shapes.size();i++){
                            delete shapes.at(i);
                        }
                        shapes.clear();
                        particlelines.clear();
                        delete saveto;
                        return 1;
                    break;
                    case 1:
                        std::ofstream savefile (*saveto);
                        if (savefile.is_open()){
                            std::cout <<"SAVING TO " << *saveto << "\n";
                            for(int i=0;i<particlelines.size();i++){
                                savefile<<"newparticleline\n";
                                saveParticleLine(particlelines.at(i), &savefile);
                            }
                            for(int i=0;i<projectiles.size();i++){
                                savefile<<"newprojectile\n";
                                saveProjectile(*projectiles.at(i), &savefile);
                            }
                            for(int i=0;i<tanks.size();i++){
                                savefile<<"newtank\n";
                                saveTank(*tanks.at(i), &savefile);
                            }
                            /*for(int i=0;i<shapes.size();i++){
                                savefile<<"newshape\n";
                                saveShape(*shapes.at(i), &savefile);
                            }*///Dont save shapes, it's a static map
                        }else
                            std::cout << "Unable to open file\n";
                        pause=true;
                        //implement methods which write a given struct to a given file, then write multiple loops that write out all vectors, then write methods which allow reading from that file
                        //when reading, reads values in order until next element flag is read, repeat until next data type flag is read.
                    break;
                }
                lastpause = SDL_GetTicks();
                pauseduration = lastpause - pauseduration;
                recentpause = true;
                delete saveto;
            }
            if(!freeze)
                tanks[0]->pointTo( mouseX*1.0, mouseY*1.0 );
            tanks[0]->step();
            //}
            for(int i=1;i<tanks.size();i++){
                if(isInShape(mouseX, mouseY, tanks[i]->getShape())&&!tanks[i]->isDead()){
                    tanknum = i;
                }
                tanks[i]->step();
            }
			//step projectiles
			for(int i = 0; i < projectiles.size(); i++){
				bool remove = stepProjectile(projectiles[i]);
				if(remove){
					delete projectiles[i];
					projectiles.erase(projectiles.begin()+i);
				}
			}
        }
        //update mouse coordinates
        SDL_GetMouseState( &mouseXfreeze, &mouseYfreeze);
        mouseXfreeze+=camX;
        mouseYfreeze+=camY;
        //separate mousecoordinates so aiming laser doesn't move while frozen
        //{Set camera coordinates
        camX=((tanks[0]->getX()+mouseXfreeze)/2)-SCREEN_WIDTH/2.0;
        camY=((tanks[0]->getY()+mouseYfreeze)/2)-SCREEN_HEIGHT/2.0;
        if(camX<-100)
            camX=-100;
        else if (camX>mapwidth+100-SCREEN_WIDTH)
            camX=mapwidth+100-SCREEN_WIDTH;
        if(camY<-100)
            camY=-100;
        else if (camY>mapheight+100-SCREEN_HEIGHT)
            camY=mapheight+100-SCREEN_HEIGHT;
        //}
        //render everything
        //aiming laser
        int tanknum2=-1;
        if(aiminglaser){
            double xcomp = (mouseX-tanks[0]->getX())*10/(sqrt(pow(mouseX-tanks[0]->getX(), 2)+pow(mouseY-tanks[0]->getY(), 2)));
            double ycomp = (mouseY-tanks[0]->getY())*10/(sqrt(pow(mouseX-tanks[0]->getX(), 2)+pow(mouseY-tanks[0]->getY(), 2)));
            double x2 = tanks[0]->getX();
            double y2 = tanks[0]->getY();
            bool ret = false;
            while(x2<camX+SCREEN_WIDTH && y2<camY+SCREEN_HEIGHT && x2>camX && y2>camY && ret==false){
            for(int i=1; i<tanks.size(); i++){
                Shape temp = tanks[i]->getShape();
                if((ret == false)&&(passedovershape(x2, y2, x2+xcomp, y2+ycomp, temp) || isInShape(x2, y2, temp))){
                    if(!tanks[i]->isDead())
                        tanknum2=i;
                    ret = true;
                    double xt = x2;
                    double yt = y2;
                    while(!(passedovershape(x2, y2, xt, yt, temp) || isInShape(xt, yt, temp))){
                        xt += xcomp/10;
                        yt += ycomp/10;
                    }
                }
            }
            for(int i=0; i<shapes.size(); i++){
                Shape temp = *shapes[i];
                if((ret == false)&&(passedovershape(x2, y2, x2+xcomp, y2+ycomp, temp) || isInShape(x2, y2, temp))){
                    ret = true;
                    double xt = x2;
                    double yt = y2;
                    while(!(passedovershape(x2, y2, xt, yt, temp) || isInShape(xt, yt, temp))){
                        xt += xcomp/10;
                        yt += ycomp/10;
                    }
                }
            }
            x2+=xcomp;
            y2+=ycomp;
            }
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0xFF, 0x00, 0xFF );
            SCROLL_RenderDrawLine(gRenderer, tanks[0]->getX(), tanks[0]->getY(), x2, y2);
        }
        bool hasAShot=false;    //aiming assist, disables primary weapon firing if doesn't have a shot or can't hit enemy mouse is over
        if(primaryWeaponAssistToggle){
            if(tanknum!=-1){
                if(tanknum==tanknum2){
                    hasAShot=true;
                }
            }else if(tanknum2!=-1){
                hasAShot=true;
            }
        }else{
            hasAShot=true;
        }
        if(!(freeze||tanks[0]->isDead()))
            fire(LmouseState&&hasAShot, RmouseState, tanks[0], &projectiles);
        //draw shape outlines
        SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
        for(int i=0;i<shapes.size();i++){
            SCROLL_RenderDrawLine( gRenderer, shapes[i]->x[shapes[i]->x.size()-1], shapes[i]->y[shapes[i]->x.size()-1], shapes[i]->x[0], shapes[i]->y[0] );
            for(int j=0;j<shapes[i]->x.size()-1;j++)
                SCROLL_RenderDrawLine( gRenderer, shapes[i]->x[j], shapes[i]->y[j], shapes[i]->x[j+1], shapes[i]->y[j+1] );
        }
        if(particleEffects){
        for(int i = 0; i < particlelines.size(); i++){
            particlelines[i];
            SDL_SetRenderDrawColor( gRenderer, particlelines[i].r, particlelines[i].g, particlelines[i].b, particlelines[i].a);
            SCROLL_RenderDrawLine( gRenderer, particlelines[i].x1, particlelines[i].y1, particlelines[i].x2, particlelines[i].y2);
			uint16_t temp = 3000*physics*stepSize;
            particlelines[i].r += particlelines[i].dr*temp;
            particlelines[i].g += particlelines[i].dg*temp;
            particlelines[i].b += particlelines[i].db*temp;
            particlelines[i].a += particlelines[i].da*temp;
            particlelines[i].t += particlelines[i].dt*temp;
            if(particlelines[i].t >= 0xFF)
                particlelines.erase(particlelines.begin()+i);
        }
        }else{
            particlelines.clear();
        }
        //render tanks
		for(int i=0; i<tanks.size(); i++){
            tanks[i]->render();
		}
		//render headsup
		//health and shield
		SDL_Rect healthRect = { 25, 25, 300, 10 };//grey back
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
		SDL_RenderFillRect( gRenderer, &healthRect );
		healthRect = { 25, 27, 300.0*(tanks[0]->getHealth()/tanks[0]->getMaxHealth()), 6 };//health
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xAF, 0xAF );
		SDL_RenderFillRect( gRenderer, &healthRect );
        healthRect = { 25, 25, 300.0*(tanks[0]->getShield()/tanks[0]->getMaxShield()), 10 };//shield
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xFF, 0x6F );
		SDL_RenderFillRect( gRenderer, &healthRect );
		//show reloading
		healthRect = { 350, 25, 100, 10 };//grey back
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
		SDL_RenderFillRect( gRenderer, &healthRect );
        healthRect = { 350, 25, 100.0*(tanks[0]->getPrimaryReload()/tanks[0]->getPrimaryROF()), 10 };//primary
        if(hasAShot)
            SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, 0xAF );
        else
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xAF );
		SDL_RenderFillRect( gRenderer, &healthRect );
		//show slomo
		healthRect = { 475, 25, 100, 10 };//grey back
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
		SDL_RenderFillRect( gRenderer, &healthRect );
        healthRect = { 475, 25, slomo, 10 };//slomo
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0xFF, 0x00, 0xAF );
		SDL_RenderFillRect( gRenderer, &healthRect );
		//show energy
		healthRect = { 475, 35, 100, 10 };//grey back
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
		SDL_RenderFillRect( gRenderer, &healthRect );
        healthRect = { 475, 35, tanks[0]->getEnergy()/tanks[0]->getMaxEnergy()*100, 10 };//slomo
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0xFF, 0x00, 0xAF );
		SDL_RenderFillRect( gRenderer, &healthRect );
		//render lil health bars
        for(int i=0;i<tanks.size();i++){
            SDL_Rect healthRect = { tanks[i]->getX()-camX-50, tanks[i]->getY()-camY-50, 100, 5 };//grey back
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
            SDL_RenderFillRect( gRenderer, &healthRect );
            healthRect = { tanks[i]->getX()-camX-50, tanks[i]->getY()-camY-49, 100.0*(tanks[i]->getHealth()/tanks[i]->getMaxHealth()), 3 };//health
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xAF, 0xAF );
            SDL_RenderFillRect( gRenderer, &healthRect );
            healthRect = { tanks[i]->getX()-camX-50, tanks[i]->getY()-camY-50, 100.0*(tanks[i]->getShield()/tanks[i]->getMaxShield()), 5 };//shield
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xFF, 0x6F );
            SDL_RenderFillRect( gRenderer, &healthRect );
        }
        std::ostringstream strs;
        strs << FPS << " " << FPS2;
        std::string text = strs.str();
        SDL_Color textColour = {0, 0, 0};
        SDL_Surface* textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
        SDL_Texture* mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
        SDL_Rect temp = {5, 5, textSurface->w/2, textSurface->h/2};
        SDL_FreeSurface( textSurface );
        SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
        SDL_DestroyTexture(mTexture);
        if(freeze){//extra HUD while frozen
            if(showDetailsFor!=NULL){
                if(RmouseClick)
                    showSecondary=!showSecondary;
                //show advanced details
                basicText("Name: "+showDetailsFor->getName(), {0, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4, 0.8, 0.8, true, 255, 255, 255, 127);
                std::stringstream stream;
                stream << "Health: "<<round(showDetailsFor->getHealth())<<"/"<<round(showDetailsFor->getMaxHealth());
                basicText(stream.str(), {0, 0, 175}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8, 0.8, 0.8, true, 255, 255, 255, 127);
                stream.str("");
                stream << "Shield: "<<round(showDetailsFor->getShield())<<"/"<<round(showDetailsFor->getMaxShield());
                basicText(stream.str(), {0, 0, 255}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*2, 0.8, 0.8, true, 255, 255, 255, 127);
                stream.str("");
                stream << "Shield Delay: "<<showDetailsFor->getShieldDelay();
                basicText(stream.str(), {0, 0, 255}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*3, 0.8, 0.8, true, 255, 255, 255, 127);
                stream.str("");
                stream << "Shield Regen: "<<showDetailsFor->getShieldRegen();
                basicText(stream.str(), {0, 0, 255}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*4, 0.8, 0.8, true, 255, 255, 255, 127);
                stream.str("");
                if(!showSecondary){
                    basicText("Primary Weapon:", {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*5, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream << "Damage: "<<showDetailsFor->getPrimaryDamage();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*6, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Reload time: "<<showDetailsFor->getPrimaryROF();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*7, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Rounds per shot: "<<showDetailsFor->getPburst();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*8, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Spread: "<<showDetailsFor->getPspread();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*9, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Resultant Damage: "<<round(10000*showDetailsFor->getPburst()*showDetailsFor->getPrimaryDamage()/showDetailsFor->getPrimaryROF())/100;
                }else{
                    basicText("Secondary Weapon:", {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*5, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream << "Damage: "<<showDetailsFor->getSecondaryDamage();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*6, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Reload time: "<<showDetailsFor->getSecondaryROF();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*7, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Rounds per shot: "<<showDetailsFor->getSburst();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*8, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Spread: "<<showDetailsFor->getSspread();
                    basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*9, 0.8, 0.8, true, 255, 255, 255, 127);
                    stream.str("");
                    stream << "Resultant Damage: "<<round(10000*showDetailsFor->getSburst()*showDetailsFor->getSecondaryDamage()/showDetailsFor->getSecondaryROF())/100;
                }
                basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*10, 0.8, 0.8, true, 255, 255, 255, 127);
                stream.str("");
                stream << "Total Damage: "<<round(10000*(showDetailsFor->getSburst()*showDetailsFor->getSecondaryDamage()/showDetailsFor->getSecondaryROF()+showDetailsFor->getPburst()*showDetailsFor->getPrimaryDamage()/showDetailsFor->getPrimaryROF()))/100;
                basicText(stream.str(), {255, 0, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*11, 0.8, 0.8, true, 255, 255, 255, 127);
                stream.str("");
                stream << "Top speed: "<<showDetailsFor->getMaxVel();
                basicText(stream.str(), {0, 255, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*12, 0.8, 0.8, true, 255, 255, 255, 127);
                stream.str("");
                stream << "Acceleration: "<<round(showDetailsFor->getAcceleration());
                basicText(stream.str(), {0, 255, 0}, SCREEN_WIDTH/2, SCREEN_HEIGHT/4+fontSize*0.8*13, 0.8, 0.8, true, 255, 255, 255, 127);


            }
            if(LmouseClick)
                showDetailsFor=NULL;

                for(int i=0;i<tanks.size();i++){
                if(isInShape(mouseXfreeze, mouseYfreeze, tanks[i]->getShape())){//display info about enememy mouse is over while frozren
                    if(LmouseClick)//set tank to show advanced details for
                        showDetailsFor=tanks[i];
                    else if(showDetailsFor==NULL){//if not showing advanced details show quick details
                        if(tanks[i]->isDead()){
                            //say tank is dead + name
                            basicText("DEAD "+tanks[i]->getName(), {0, 0, 0}, tanks[i]->getX()-camX, tanks[i]->getY()-camY+50, 0.5, 0.5, true, 255, 255, 255, 127);
                        }else{
                            //show some stats
                            basicText(tanks[i]->getName(), {0, 0, 0}, tanks[i]->getX()-camX, tanks[i]->getY()-camY+50, 0.5, 0.5, true, 255, 255, 255, 127);
                            std::stringstream stream;
                            stream << round(tanks[i]->getHealth())<<"/"<<round(tanks[i]->getMaxHealth());
                            basicText(stream.str(), {0, 0, 175}, tanks[i]->getX()-camX, tanks[i]->getY()-camY+50+fontSize/2, 0.5, 0.5, true, 255, 255, 255, 127);
                            stream.str("");
                            stream << round(tanks[i]->getShield())<<"/"<<round(tanks[i]->getMaxShield());
                            basicText(stream.str(), {0, 0, 255}, tanks[i]->getX()-camX, tanks[i]->getY()-camY+50+fontSize, 0.5, 0.5, true, 255, 255, 255, 127);
                            stream.str("");
                            stream << round(10000*(tanks[i]->getSburst()*tanks[i]->getSecondaryDamage()/tanks[i]->getSecondaryROF()+tanks[i]->getPburst()*tanks[i]->getPrimaryDamage()/tanks[i]->getPrimaryROF()))/100;
                            basicText(stream.str(), {255, 0, 0}, tanks[i]->getX()-camX, tanks[i]->getY()-camY+50+fontSize*1.5, 0.5, 0.5, true, 255, 255, 255, 127);
                        }
                    }
                }
            }
        }else{
            showDetailsFor=NULL;
            showSecondary=false;
        }
		//render to screen
		SDL_RenderPresent( gRenderer );
		//clear
        SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
		SDL_RenderClear( gRenderer );
        //stay under framrate cap
		if(SDL_GetTicks()-frameTimer < TICKS_PER_FRAME)
            SDL_Delay(TICKS_PER_FRAME-(SDL_GetTicks()-frameTimer));
        //console log
        std::cout << "FPS: " << frames.size() << "\n";
        frameCount++;
        std::cout << "Frame count: " << frameCount << "\n";
        std::cout << "Particles: " << particlelines.size() << "\n";

	}
	return 0;
}

void *thread(void* in){
	uint32_t frameTimer;
	std::vector<uint32_t> frames;
	for(double i=0; i<1000; i+=1000/MAX_FPS){
		frames.push_back(SDL_GetTicks()-1000+i);
	}
	threaddata *data;
	data = (threaddata *) in;
	while(graphicsthreadrun){
			if(!graphicsthreadpause){
				frameTimer = SDL_GetTicks();
				frames.push_back(SDL_GetTicks());
				for(int i=0;i<frames.size();i++){
					if(frames[i] + 1000 < SDL_GetTicks()){
						if(recentpause){
							if(SDL_GetTicks()-lastpause<1000){//check again if recently paused
								if(frames[i]+1000+pauseduration<SDL_GetTicks())
									frames.erase(frames.begin()+i);
							}else{
								recentpause=false;
								frames.erase(frames.begin()+i);
							}
						}else{
							frames.erase(frames.begin()+i);
						}
					}
				}
				FPS2 = frames.size();
				//enemy AI
            if(!freeze){
            for(int i=1; i<tanks.size(); i++){
                double targx = tanks[0]->getX();
                double targy = tanks[0]->getY();
                if(cansee(i, 0, &targx, &targy)){
                    tanks[i]->settargetvel(tanks[0]->getVel());
                    tanks[i]->pointTo( targx, targy );
                    if(sqrt(pow((targx-tanks[i]->getX()), 2)+pow((targy-tanks[i]->getY()), 2))>100){
                        tanks[i]->accelerate(100);
                        tanks[i]->brake(0);
                    }else{
                        tanks[i]->accelerate(0);
                        tanks[i]->brake(100);
                    }
                    fire(true, true, tanks[i], data->projectiles);
                } else {
                    tanks[i]->settargetvel(0.0);
                    if(tanks[i]->hasseentarget()){
                        tanks[i]->accelerate(100);
                        tanks[i]->brake(0);
                    } else {
                        tanks[i]->accelerate(0);
                        tanks[i]->brake(100);
                    }
                }
//                std::cout << "VISIBLE!!!\n";
            }}
//				TICKS_PER_FRAME2 = 1000/FPS;
				if(SDL_GetTicks()-frameTimer < TICKS_PER_FRAME2)
					SDL_Delay(TICKS_PER_FRAME2-(SDL_GetTicks()-frameTimer));
				std::cout << "FPS2: " << frames.size() << "\n";
			}
	}
}

void addButton(std::vector<Shape*>* toadd, double xpos, double ypos, double width, double height){
    Shape* shape = new Shape;
    std::vector<double>* x = new std::vector<double>;
    std::vector<double>* y = new std::vector<double>;
    x->push_back(xpos-width/2);
    x->push_back(xpos-width/2);
    x->push_back(xpos+width/2);
    x->push_back(xpos+width/2);
    y->push_back(ypos-height/2);
    y->push_back(ypos+height/2);
    y->push_back(ypos-height/2);
    y->push_back(ypos+height/2);
    sortpoints(x, y);
    shape->x = *x;
    shape->y = *y;
    toadd->push_back(shape);
    shape = NULL;
}

uint8_t mainMenu(){
    //Add menu
    std::cout << "showing menu\n";
    std::vector<Shape*> buttons;
    bool atMenu = true;
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT/4, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT*5/12, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT*7/12, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT*3/4, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    //{render buttons
    SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	SDL_RenderClear( gRenderer );
    SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
    for(int i=0;i<buttons.size();i++){
        SDL_RenderDrawLine( gRenderer, buttons[i]->x[buttons[i]->x.size()-1], buttons[i]->y[buttons[i]->x.size()-1], buttons[i]->x[0], buttons[i]->y[0] );
        for(int j=0;j<buttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, buttons[i]->x[j], buttons[i]->y[j], buttons[i]->x[j+1], buttons[i]->y[j+1] );
    }
    std::string text = "Play Game";
    SDL_Color textColour = {0, 0, 0};
    SDL_Surface* textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    SDL_Texture* mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    SDL_Rect temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT/4-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);

	text = "Load Game";
    textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT*5/12-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);

	text = "Settings";
    textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT*7/12-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);

	text = "Quit";
    textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT*3/4-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
    SDL_RenderPresent( gRenderer );
    //}
    bool escstate = false;
    while(atMenu){
        if(updateMouse())//updateMouse returns true if quit signal is sent
            return 0;
        SDL_GetMouseState( &mouseX, &mouseY );
        if(LmouseClick)
            for(int i=0; i<buttons.size();i++){
                if(isInShape(mouseX, mouseY, *buttons[i])){
                    switch (i){
                        case 0://new game
                            atMenu=false;
                        break;
                        case 1://load game
                            //remove some of once saving is implemented
                            loadSettings("data/game/settings.txt");
                            loadTankTemplates("data/game/tanktemplates.txt");//Load tank templates as it isn't saved
                            //loadTanks("data/game/tanks.txt");
                            setMap("data/game/map1.txt");//Load map, as it isn't saved
                            delete buttons.at(0);
                            delete buttons.at(1);
                            return 3;
                        break;
                        case 3://exit
                            return 0;
                        break;
                    }
                }
            }
        if(!keys[ pauseKey] && escstate)//when escape key released
                return 0;//quit game
        escstate=keys[ pauseKey];
        SDL_RenderPresent( gRenderer );//nothing moves, but will prevent screen from going glitchy when dragged
    }
    loadSettings("data/game/settings.txt");
    loadTankTemplates("data/game/tanktemplates.txt");
	loadTanks("data/game/tanks.txt");
	setMap("data/game/map1.txt");
	delete buttons.at(0);
	delete buttons.at(1);
	return 2;//go back and start game
}

uint8_t pauseMenu(std::string* data){
    bool loop = true;
    while(loop){
        SDL_PumpEvents();
        if(!keys[pauseKey])
            loop = false;
    }
    loop = true;
    bool escstate = false;

    std::vector<Shape*> buttons;
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT/4, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT*5/12, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT*7/12, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    addButton(&buttons, SCREEN_WIDTH/2, SCREEN_HEIGHT*3/4, SCREEN_WIDTH/2, SCREEN_HEIGHT/10);
    //{render buttons
    SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	SDL_RenderClear( gRenderer );
    SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
    for(int i=0;i<buttons.size();i++){
        SDL_RenderDrawLine( gRenderer, buttons[i]->x[buttons[i]->x.size()-1], buttons[i]->y[buttons[i]->x.size()-1], buttons[i]->x[0], buttons[i]->y[0] );
        for(int j=0;j<buttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, buttons[i]->x[j], buttons[i]->y[j], buttons[i]->x[j+1], buttons[i]->y[j+1] );
    }
    std::string text = "Return to Main Menu";
    SDL_Color textColour = {0, 0, 0};
    SDL_Surface* textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    SDL_Texture* mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    SDL_Rect temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT/4-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);

	text = "Save Game";
    textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT*5/12-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);

	text = "Settings";
    textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT*7/12-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);

	text = "Resume Game";
    //textColour = {0, 0, 0};
    textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    temp = {(SCREEN_WIDTH-textSurface->w)/2, SCREEN_HEIGHT*3/4-textSurface->h/2, textSurface->w, textSurface->h};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
    SDL_RenderPresent( gRenderer );
    SDL_DestroyTexture(mTexture);
    //}
    while(loop){
        if(updateMouse())//updateMouse returns true if quit signal is sent
            return true;
        SDL_GetMouseState( &mouseX, &mouseY );
        if(LmouseClick)
            for(int i=0; i<buttons.size();i++){
                if(isInShape(mouseX, mouseY, *buttons[i])){
                    switch (i){
                        case 0:
                            delete buttons.at(0);
                            delete buttons.at(1);
                            delete buttons.at(2);
                            delete buttons.at(3);
                            return 0;
                        break;
                        case 1:
                            saveMenu(data);
                            if(*data!="")
                                return 1;
                        break;
                        case 2:
                        break;
                        case 3:
                            delete buttons.at(0);
                            delete buttons.at(1);
                            delete buttons.at(2);
                            delete buttons.at(3);
                            return 2;
                        break;
                    }
                }
            }
        SDL_RenderPresent( gRenderer );
        if(!keys[ pauseKey] && escstate){//when escape key released
                    delete buttons.at(0);
                    delete buttons.at(1);
                    delete buttons.at(2);
                    delete buttons.at(3);
                    return 2;//continue game
        }
        escstate=keys[ pauseKey];
    }
}

void saveMenu(std::string* data){
    *data = "data/save/save1.game";
}

std::string loadMenu(){
    return "data/save/save1.game";
}

void saveParticleLine(ParticleLine save, std::ofstream* file){
    std::ostringstream strs;
    strs << save.x1 << "\n";
    strs << save.y1 << "\n";
    strs << save.x2 << "\n";
    strs << save.y2 << "\n";
    strs << save.r << "\n";
    strs << save.g << "\n";
    strs << save.b << "\n";
    strs << save.a << "\n";
    strs << save.t << "\n";
    strs << save.dr << "\n";
    strs << save.dg << "\n";
    strs << save.db << "\n";
    strs << save.da << "\n";
    strs << save.dt << "\n";
    strs << save.id << "\n";
    *file << strs.str();
}
ParticleLine loadParticleLine(std::ifstream* file){
    ParticleLine load;
    std::string line;
    getline(*file,line);
    load.x1=atof(line.c_str());
    getline(*file,line);
    load.y1=atof(line.c_str());
    getline(*file,line);
    load.x2=atof(line.c_str());
    getline(*file,line);
    load.y2=atof(line.c_str());
    getline(*file,line);
    load.r=atof(line.c_str());
    getline(*file,line);
    load.g=atof(line.c_str());
    getline(*file,line);
    load.b=atof(line.c_str());
    getline(*file,line);
    load.a=atof(line.c_str());
    getline(*file,line);
    load.t=atof(line.c_str());
    getline(*file,line);
    load.dr=atof(line.c_str());
    getline(*file,line);
    load.dg=atof(line.c_str());
    getline(*file,line);
    load.db=atof(line.c_str());
    getline(*file,line);
    load.da=atof(line.c_str());
    getline(*file,line);
    load.dt=atof(line.c_str());
    getline(*file,line);
    load.id=atoi(line.c_str());
    return load;
}
void saveShape(Shape save, std::ofstream* file){
    std::ostringstream strs;
    for(int i=0;i<save.x.size();i++)
        strs << save.x.at(i) << "\n";
    strs << "ENDX\n";
    for(int i=0;i<save.y.size();i++)
        strs << save.y.at(i) << "\n";
    strs << "ENDSHAPE\n";
    *file << strs.str();
}
Shape loadShape(std::ifstream* file){
    Shape load;
    std::string line;
    getline(*file,line);
    while(line.compare("ENDX")!=0){
        load.x.push_back(atof(line.c_str()));
        getline(*file,line);
    }
    getline(*file,line);
    while(line.compare("ENDSHAPE")!=0){
        load.y.push_back(atof(line.c_str()));
        getline(*file,line);
    }
    return load;
}
void saveProjectile(Projectile save, std::ofstream* file){
    std::ostringstream strs;
    strs << save.x << "\n";
    strs << save.y << "\n";
    strs << save.xvel << "\n";
    strs << save.yvel << "\n";
    strs << save.angle << "\n";
    strs << save.width << "\n";
    strs << save.length << "\n";
    strs << save.damage << "\n";
    if(save.passesShield){
        strs << "t\n";
    }else{
        strs << "f\n";
    }
    strs << save.dist << "\n";
    strs << save.lastid << "\n";
    strs << save.ownerID << "\n";
    if(save.primaryWeapon){
        strs << "t\n";
    }else{
        strs << "f\n";
    }
    *file << strs.str();
}
Projectile loadProjectile(std::ifstream* file){
    Projectile load;
    std::string line;
    getline(*file,line);
    load.x=atof(line.c_str());
    getline(*file,line);
    load.y=atof(line.c_str());
    getline(*file,line);
    load.xvel=atof(line.c_str());
    getline(*file,line);
    load.yvel=atof(line.c_str());
    getline(*file,line);
    load.angle=atof(line.c_str());
    getline(*file,line);
    load.width=atoi(line.c_str());
    getline(*file,line);
    load.length=atoi(line.c_str());
    getline(*file,line);
    load.damage=atoi(line.c_str());
    getline(*file,line);
    if(line.compare("t")==0)
        load.passesShield=true;
    else
        load.passesShield=false;
    getline(*file,line);
    load.dist=atoi(line.c_str());
    getline(*file,line);
    load.lastid=atoi(line.c_str());
    getline(*file,line);
    load.ownerID=atoi(line.c_str());
    getline(*file,line);
    if(line.compare("t")==0)
        load.primaryWeapon=true;
    else
        load.primaryWeapon=false;
    for(int i=0;i<tankTemplates.size();i++){
        if(tankTemplates.at(i).getID()==load.ownerID){
            tankTemplates.at(i).getProjectileLook(&(load.look), &(load.clip), &(load.traileffect), load.primaryWeapon);
        }
    }
    return load;
}
void saveTank(Tank save, std::ofstream* file){
    *file << save.getID() << "\n";
    save.save(file);
}
Tank loadTank(std::ifstream* file){
    Tank load;
    std::string line;
    getline(*file, line);
    for(int i=0;i<tankTemplates.size();i++){
        if(tankTemplates[i].getID()==atoi(line.c_str())){
            load = tankTemplates[i];
        }
    }
    load.load(file);
    return load;
}

bool loadSettings(char* filename){
    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)){//read from file
        if(line.length()!=0){
            if(line.find("skipMenu")!=std::string::npos){
                if(line.substr(line.find("-")+1)=="true")
                    return true;
            }else if(line.find("particleEffects")!=std::string::npos){
                if(line.substr(line.find("-")+1)=="true")
                    particleEffects=true;
            }
        }
    }
    return false;
}

void loadTankTemplates(char* filename){
    Tank* temp = new Tank("unnamed", 0, 0, 0);
    Tank t = *temp;
    std::ifstream infile(filename);
    std::string line;
    //{temporary values for template loading
    double power;
	double brakeforce;
	double maxvel;
    double mass;
    double maxEnergy;
    double energyGen;

    double primaryProjectileSpeed;
    double secondaryProjectileSpeed;
    int primaryROF;
    int secondaryROF;
    double pspread;
    double sspread;
    int pburst;
    int sburst;
    double primaryDamage;
    double secondaryDamage;
    bool pPassesShield=false;
    bool sPassesShield=false;
    double pEnergy;
    double sEnergy;

    double length;
    double width;
	double turretradius;
	double barrellength;
	double barrelwidth;
    double pprojlength;
    double pprojwidth;
    double sprojlength;
    double sprojwidth;

    double maxHealth;
    double maxShield;
    double shieldRegen;
    int shieldDelay;
    double health;
    double shield;

    std::string bodypath = "data/graphics/body.png";
    std::string turretpath = "data/graphics/turret.png";
    std::string barrelpath = "data/graphics/barrel.png";
    std::string pprojpath = "data/graphics/primaryprojectile.png";
    std::string sprojpath = "data/graphics/secondaryprojectile.png";

    ParticleLine primary;
    ParticleLine secondary;

    int ID;
    //}
    while (std::getline(infile, line)){//read from file
        if(line.length()!=0){
            if(line.at(0)=='n'){//set name
                t.positionReset(line.substr(1), 0, 0, 0, true);
            }else if(line.at(0)=='a'){//add template and start a new one
                t.setDimensions(length, width, turretradius, barrellength, barrelwidth, pprojlength, pprojwidth, sprojlength, sprojwidth);
                t.setOther(ID, power, brakeforce, maxvel, mass, maxEnergy, energyGen);
                t.setWeapons(primaryDamage, secondaryDamage, primaryProjectileSpeed, secondaryProjectileSpeed, primaryROF, secondaryROF, pspread, sspread, pburst, sburst, primary, secondary, pPassesShield, sPassesShield, pEnergy, sEnergy);
                t.setHealth(maxHealth, maxShield, shieldRegen, shieldDelay);
                t.loadLook(bodypath, turretpath, barrelpath, pprojpath, sprojpath, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
                for(int i=0;i<tankTemplates.size();i++){
                    if(ID==tankTemplates.at(i).getID()){
                        std::cout << "ERROR DUPLICATE ID, MAY CAUSE ISSUES WITH SAVING AND LOADING\n";
                    }
                    if(t.getName()==tankTemplates.at(i).getName()){
                        std::cout << "ERROR DUPLICATE NAME, MAY CAUSE ISSUES WITH SAVING AND LOADING\n";
                    }
                }
                tankTemplates.push_back(t);
                t = *temp;
                power=0;//reset values
                brakeforce=0;
                maxvel=0;
                mass=0;
                maxEnergy=0;
                energyGen=0;
                primaryProjectileSpeed=0;
                secondaryProjectileSpeed=0;
                primaryROF=0;
                secondaryROF=0;
                pspread=0;
                sspread=0;
                pburst=0;
                sburst=0;
                primaryDamage=0;
                secondaryDamage=0;
                pPassesShield=false;
                sPassesShield=false;
                pEnergy=0;
                sEnergy=0;
                length=0;
                width=0;
                turretradius=0;
                barrellength=0;
                barrelwidth=0;
                pprojlength=0;
                pprojwidth=0;
                sprojlength=0;
                sprojwidth=0;
                maxHealth=0;
                maxShield=0;
                shieldRegen=0;
                shieldDelay=0;
                health=0;
                shield=0;
                bodypath = "data/graphics/body.png";
                turretpath = "data/graphics/turret.png";
                barrelpath = "data/graphics/barrel.png";
                pprojpath = "data/graphics/primaryprojectile.png";
                sprojpath = "data/graphics/secondaryprojectile.png";
                primary = {};
                secondary = {};
            }else if(line.at(0)=='o'){
                if(line.at(1)=='p'){
                    power = atof(line.substr(2).c_str());
                }else if(line.at(1)=='b'){
                    brakeforce = atof(line.substr(2).c_str());
                }else if(line.at(1)=='v'){
                    maxvel = atof(line.substr(2).c_str());
                }else if(line.at(1)=='m'){
                    mass = atof(line.substr(2).c_str());
                }else if(line.at(1)=='e'){
                    maxEnergy = atof(line.substr(2).c_str());
                }else if(line.at(1)=='g'){
                    energyGen = atof(line.substr(2).c_str());
                }
            }else if(line.at(0)=='w'){
                if(line.at(1)=='p'){
                    if(line.at(2)=='s'){
                        primaryProjectileSpeed=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='r'){
                        primaryROF=atoi(line.substr(3).c_str());
                    }else if(line.at(2)=='a'){
                        pspread=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='b'){
                        pburst=atoi(line.substr(3).c_str());    //add in particle effect after here
                    }else if(line.at(2)=='d'){
                        primaryDamage=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='p'){
                        pPassesShield=true;
                    }else if(line.at(2)=='e'){
                        pEnergy=atof(line.substr(3).c_str());
                    }
                }else if(line.at(1)=='s'){
                    if(line.at(2)=='s'){
                        secondaryProjectileSpeed=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='r'){
                        secondaryROF=atoi(line.substr(3).c_str());
                    }else if(line.at(2)=='a'){
                        sspread=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='b'){
                        sburst=atoi(line.substr(3).c_str());    //add in particle effect after here
                    }else if(line.at(2)=='d'){
                        secondaryDamage=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='p'){
                        sPassesShield=true;
                    }else if(line.at(2)=='e'){
                        sEnergy=atof(line.substr(3).c_str());
                    }
                }
            }else if(line.at(0)=='d'){
                if(line.at(1)=='l'){
                    length=atof(line.substr(2).c_str());
                }else if(line.at(1)=='w'){
                    width=atof(line.substr(2).c_str());
                }else if(line.at(1)=='r'){
                    turretradius=atof(line.substr(2).c_str());
                }else if(line.at(1)=='b'){
                    if(line.at(2)=='l'){
                        barrellength=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='w'){
                        barrelwidth=atof(line.substr(3).c_str());
                    }
                }else if(line.at(1)=='p'){
                    if(line.at(2)=='l'){
                        pprojlength=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='w'){
                        pprojwidth=atof(line.substr(3).c_str());
                    }
                }else if(line.at(1)=='s'){
                    if(line.at(2)=='l'){
                        sprojlength=atof(line.substr(3).c_str());
                    }else if(line.at(2)=='w'){
                        sprojwidth=atof(line.substr(3).c_str());
                    }
                }
            }else if(line.at(0)=='h'){
                if(line.at(1)=='h'){
                    maxHealth=atof(line.substr(2).c_str());
                    health=maxHealth;
                }else if(line.at(1)=='s'){
                    maxShield=atof(line.substr(2).c_str());
                    shield=maxShield;
                }else if(line.at(1)=='r'){
                    shieldRegen=atof(line.substr(2).c_str());
                }else if(line.at(1)=='d'){
                    shieldDelay=atoi(line.substr(2).c_str());
                }
            }else if(line.at(0)=='l'){
                if(line.at(1)=='b'){
                    bodypath=line.substr(2);
                }else if(line.at(1)=='t'){
                    turretpath=line.substr(2);
                }else if(line.at(1)=='a'){
                    barrelpath=line.substr(2);
                }else if(line.at(1)=='p'){
                    pprojpath=line.substr(2);
                }else if(line.at(1)=='s'){
                    sprojpath=line.substr(2);
                }
            }else if(line.at(0)=='c'){
                t.setIsCircle(true);
            }else if(line.at(0)=='p'){//particle effect
                if(line.at(1)=='p'){//primary weapon
                    switch(line.at(2)){
                        case 'r':
                            primary.r=atof(line.substr(3).c_str());
                        break;
                        case 'g':
                            primary.g=atof(line.substr(3).c_str());
                        break;
                        case 'b':
                            primary.b=atof(line.substr(3).c_str());
                        break;
                        case 'a':
                            primary.a=atof(line.substr(3).c_str());
                        break;
                        case 't':
                            primary.t=atof(line.substr(3).c_str());
                        break;
                    }
                    if(line.at(2)=='d'){
                        switch(line.at(3)){
                            case 'r':
                                primary.dr=atof(line.substr(4).c_str());
                            break;
                            case 'g':
                                primary.dg=atof(line.substr(4).c_str());
                            break;
                            case 'b':
                                primary.db=atof(line.substr(4).c_str());
                            break;
                            case 'a':
                                primary.da=atof(line.substr(4).c_str());
                            break;
                            case 't':
                                primary.dt=atof(line.substr(4).c_str());
                            break;
                        }
                    }
                }else if(line.at(1)=='s'){
                    switch(line.at(2)){
                        case 'r':
                            secondary.r=atof(line.substr(3).c_str());
                        break;
                        case 'g':
                            secondary.g=atof(line.substr(3).c_str());
                        break;
                        case 'b':
                            secondary.b=atof(line.substr(3).c_str());
                        break;
                        case 'a':
                            secondary.a=atof(line.substr(3).c_str());
                        break;
                        case 't':
                            secondary.t=atof(line.substr(3).c_str());
                        break;
                    }
                    if(line.at(2)=='d'){
                        switch(line.at(3)){
                            case 'r':
                                secondary.dr=atof(line.substr(4).c_str());
                            break;
                            case 'g':
                                secondary.dg=atof(line.substr(4).c_str());
                            break;
                            case 'b':
                                secondary.db=atof(line.substr(4).c_str());
                            break;
                            case 'a':
                                secondary.da=atof(line.substr(4).c_str());
                            break;
                            case 't':
                                secondary.dt=atof(line.substr(4).c_str());
                            break;
                        }
                    }
                }
            }else if(line.at(0)=='i'){
                ID = atoi(line.substr(1).c_str());
            }
        }
    }
    delete temp;
}

void loadTanks(char* filename){
    std::ifstream infile(filename);
    std::string line;
    Tank* t = new Tank();
    double x;
    double y;
    double r=0;
    bool templated = false;
    while (std::getline(infile, line)){
        if(line.length()!=0){
            if(line.at(0)=='n'){
                for(int i=0;i<tankTemplates.size();i++){
                    if(tankTemplates[i].getName()==line.substr(1)){
                        if(templated){
                            std::cout << "ERROR DUPLICATE NAME, FIRST ONE WILL BE USED\n";
                        }else{
                            *t = tankTemplates[i];
                            templated = true;
                        }
                    }
                }
                if(!templated){
                    std::cout << "Name " << line.substr(1) << " not found!\n";
                }
            }else if(line.at(0)=='i'){
                for(int i=0;i<tankTemplates.size();i++){
                    if(tankTemplates[i].getID()==atoi(line.substr(1).c_str())){
                        if(templated){
                            std::cout << "ERROR DUPLICATE ID, FIRST ONE WILL BE USED\n";
                        }else{
                            *t = tankTemplates[i];
                            templated = true;
                        }
                    }
                }
                if(!templated){
                    std::cout << "ID " << line.substr(1) << " not found!\n";
                }
            }else if(line.at(0)=='a'){
                if(templated){
                    t->positionReset("", x, y, r, true);
                    tanks.push_back(t);
                }else{
                    delete t;
                    std::cout << "Error: No template loaded\n";
                }
                t = new Tank();
                templated=false;
            }else if(line.at(0)=='x'){
                x = atof(line.substr(1).c_str());
                if(x<0)
                    x = SCREEN_WIDTH + x;
            }else if(line.at(0)=='y'){
                y = atof(line.substr(1).c_str());
                if(y<0)
                    y = SCREEN_HEIGHT + y;
            }else if(line.at(0)=='r'){
                r = atof(line.substr(1).c_str());
            }
        }
    }
    for(int i=0;i<tanks.size();i++){
        tanks[i]->setIndex(i);
	}
}

void setMap(char* filename){
    int* dimensions = loadMap(&shapes, filename);
	mapwidth = dimensions[0];   //set map dimensions
	mapheight = dimensions[1];
	delete dimensions;
	//set barriers
	Shape* shape = new Shape;
	std::vector<double>* x = new std::vector<double>;
	std::vector<double>* y = new std::vector<double>;
	//top barrier
    x->push_back(-250);
    x->push_back(mapwidth+250);
    x->push_back(mapwidth+250);
    x->push_back(-250);
    y->push_back(-250);
    y->push_back(-250);
    y->push_back(0);
    y->push_back(0);
    sortpoints(x, y);
    shape->x = *x;
    shape->y = *y;
    shapes.push_back(shape);
    shape = new Shape;
    //bottom barrier
    delete x;
    delete y;
    x = new std::vector<double>;
    y = new std::vector<double>;
    x->push_back(-250);
    x->push_back(mapwidth+250);
    x->push_back(mapwidth+250);
    x->push_back(-250);
    y->push_back(mapheight+250);
    y->push_back(mapheight+250);
    y->push_back(mapheight);
    y->push_back(mapheight);
    sortpoints(x, y);
    shape->x = *x;
    shape->y = *y;
    shapes.push_back(shape);
    shape = new Shape;
    //left barrier
    delete x;
    delete y;
    x = new std::vector<double>;
    y = new std::vector<double>;
    x->push_back(-250);
    x->push_back(0);
    x->push_back(0);
    x->push_back(-250);
    y->push_back(-250);
    y->push_back(-250);
    y->push_back(mapheight+250);
    y->push_back(mapheight+250);
    sortpoints(x, y);
    shape->x = *x;
    shape->y = *y;
    shapes.push_back(shape);
    shape = new Shape;
    //left barrier
    delete x;
    delete y;
    x = new std::vector<double>;
    y = new std::vector<double>;
    x->push_back(mapwidth);
    x->push_back(mapwidth+250);
    x->push_back(mapwidth+250);
    x->push_back(mapwidth);
    y->push_back(-250);
    y->push_back(-250);
    y->push_back(mapheight+250);
    y->push_back(mapheight+250);
    sortpoints(x, y);
    shape->x = *x;
    shape->y = *y;
    shapes.push_back(shape);
    delete x;
    delete y;
}

int* loadMap(std::vector<Shape*>* shapestemp, char* filename){
    int* dimensions = new int[2];
    std::vector<double>* x = new std::vector<double>;
    std::vector<double>* y = new std::vector<double>;
    Shape* shape;
    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)){//read from file
        if(line.length()!=0){
            if(line.at(0)=='w')//map width
                dimensions[0] = atoi( line.substr(1).c_str() );
            else if(line.at(0)=='h')//map height
                dimensions[1] = atoi( line.substr(1).c_str() );
            else if(line.at(0)=='x')//x coordinate of corner
                x->push_back(atoi(line.substr(1).c_str()));
            else if(line.at(0)=='y')//y coordinate of corner
                y->push_back(atoi(line.substr(1).c_str()));
            else if(line.at(0)=='s'){//new shape
                if(x->size()==y->size() && x->size()>1){
                sortpoints(x, y);
                shape = new Shape;
                shape->x = *x;
                shape->y = *y;
                shapestemp->push_back(shape);
                }
                delete x;
                delete y;
                x = new std::vector<double>;
                y = new std::vector<double>;
            }
        }
    }
    delete x;
    delete y;
    return dimensions;
}

void fire(bool pfire, bool sfire, Tank* tank, std::vector<Projectile*>* projectiles){
    if(pfire && tank->primaryReloaded() && tank->getEnergy()>=tank->getPEnergy()){
        for(int i=0; i<tank->getPburst(); i++){
            Projectile* t = new Projectile;
            projectiles->push_back(t);
            tank->shootPrimary(t);
        }
    }
   	if(sfire && tank->secondaryReloaded() && tank->getEnergy()>=tank->getSEnergy()){
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
    ParticleLine part = *(proj->traileffect);
    part.id=effectIDs;
    effectIDs++;
    if((proj->dist+1)*stepSize*sqrt(pow(proj->xvel, 2)+pow(proj->yvel, 2)) > traileffectprescision){
        proj->dist=0;
    }else{
        for(int i=particlelines.size()-1;i>=0;i--)
            if(particlelines[i].id==proj->lastid){
                particlelines.erase(particlelines.begin()+i);
                i=-1;
            }
        }
    if(particleEffects){
    part.x1 = proj->x-proj->dist*proj->xvel*stepSize;
    part.y1 = proj->y-proj->dist*proj->yvel*stepSize;
    part.x2 = proj->x+proj->xvel*stepSize;
    part.y2 = proj->y+proj->yvel*stepSize;
    }
    proj->dist++;
    //detect collision
	for(int i=0; i<tanks.size(); i++){
        Shape temp = tanks[i]->getShape();
        if((ret == false)&&(passedovershape(proj->x, proj->y, proj->x+proj->xvel*stepSize, proj->y+proj->yvel*stepSize, temp) || isInShape(proj->x, proj->y, temp))){
            std::cout << "Tank " << i << " hit by a projectile\n";
            tanks[i]->takeDamage(proj->damage, proj->passesShield);
            ret = true;
            double xt = proj->x;
            double yt = proj->y;
            while(!(passedovershape(proj->x, proj->y, xt, yt, temp) || isInShape(proj->x, proj->y, temp))){
                xt += proj->xvel*stepSize*streakrenderres;
                yt += proj->yvel*stepSize*streakrenderres;
            }
            if(particleEffects){
            part.x2 = xt;
            part.y2 = yt;
            }
        }
	}
    for(int i=0; i<shapes.size(); i++){
        Shape* temp = shapes[i];
        if((ret == false)&&(passedovershape(proj->x, proj->y, proj->x+proj->xvel*stepSize, proj->y+proj->yvel*stepSize, *temp) || isInShape(proj->x, proj->y, *temp))){
            std::cout << "Shape " << i << " hit by a projectile\n";
            ret = true;
            //find point where shape was hit
            double xt = proj->x;
            double yt = proj->y;
            while(!(passedovershape(proj->x, proj->y, xt, yt, *temp) || isInShape(proj->x, proj->y, *temp))){
                xt += proj->xvel*stepSize*streakrenderres;
                yt += proj->yvel*stepSize*streakrenderres;
            }
            part.x2 = xt;
            part.y2 = yt;
        }
	}
	if(!ret){
	//render projectile
    SDL_Rect renderposition = {proj->x+proj->xvel*stepSize-proj->width/2-camX, proj->y+proj->yvel*stepSize-proj->length/2-camY, proj->width, proj->length};						//x-width/2 is used, so that the x and y coordinates refer to the center of the tank
	SDL_RenderCopyEx( gRenderer, proj->look, proj->clip, &renderposition, proj->angle+90, NULL, SDL_FLIP_NONE);
	}
    proj->x += proj->xvel*stepSize;
    proj->y += proj->yvel*stepSize;
    //maths to find if projectile will cross screen
    //if not (going to cross screen and going towards screen
    /*int visualX=proj->x-camX;
    int visualY=proj->y-camY;
    if(visualX < 0 || visualX > SCREEN_WIDTH || visualY < 0 || visualY > SCREEN_HEIGHT){    //off the screen
        if(visualY > SCREEN_HEIGHT/2){  //below centre of the screen
            if(visualX > SCREEN_WIDTH/2){   //bottom right
                if(!(yfromx(SCREEN_WIDTH, visualX, visualY, proj->xvel, proj->yvel) > 0 && yfromx(0, visualX, visualY, proj->xvel, proj->yvel) < SCREEN_HEIGHT && proj->xvel < 0 && proj->yvel < 0))
                    ret = true;
            }else{  //bottom left
                if(!(yfromx(SCREEN_WIDTH, visualX, visualY, proj->xvel, proj->yvel) < SCREEN_HEIGHT && yfromx(0, visualX, visualY, proj->xvel, proj->yvel) > 0 && proj->xvel > 0 && proj->yvel < 0))
                    ret = true;
            }
        }else{  //above centre of the screen
            if(proj->x > SCREEN_WIDTH/2){   //top right
                if(!(yfromx(SCREEN_WIDTH, visualX, visualY, proj->xvel, proj->yvel) < SCREEN_HEIGHT && yfromx(0, visualX, visualY, proj->xvel, proj->yvel) > 0 && proj->xvel < 0 && proj->yvel > 0))
                    ret = true;
            }else{  //top left
                if(!(yfromx(SCREEN_WIDTH, visualX, visualY, proj->xvel, proj->yvel) > 0 && yfromx(0, visualX, visualY, proj->xvel, proj->yvel) < SCREEN_HEIGHT && proj->xvel > 0 && proj->yvel > 0))
                    ret = true;
            }
        }
    }*/
    if(particleEffects)
        particlelines.push_back(part);
    proj->lastid=part.id;
    return ret;
}

bool cansee(int tank1, int tankn, double* targx, double* targy){
    //Find angle and distance to tank
    //Rotate clockwise from angle and check if collision with given tank and nothing else
    //same but clockwise
    double x = tanks[tank1]->getX();    //coordinates of tank
    double y = tanks[tank1]->getY();
    double tx = tanks[tankn]->getX();   //coordinates of target
    double ty = tanks[tankn]->getY();
//   	bool visible = false;
   	bool tvisible = true;   //keeps track of whether the target is visible
   	for(int i=0; i<tanks.size(); i++){  //check through all shapes that are not the tank or target; if none are blocking the view tvisible is left true
        if(i!=tankn && i!=tank1){
            Shape temp = tanks[i]->getShape();
            if((tvisible == true)&&(passedovershape(x, y, tx, ty, temp) || isInShape(x, y, temp))){
                tvisible = false;
            }
        }
	}
    for(int i=0; i<shapes.size(); i++){
        Shape* temp = shapes[i];
        if((tvisible == true)&&(passedovershape(x, y, tx, ty, *temp) || isInShape(x, y, *temp))){
            tvisible = false;
        }
	}
	double angle = atan2(y, x, ty, tx); //set angle to target
	if(tvisible == true){   //if target is visible return true
        return true;
	}
	double tangle = angle;  //angle changed to check if target is visible
	bool run = true;    //variable for while loop
	double dist;    //distance to the target at tangle
	int checks = 0; //track the number of angles checked
	double acdist = distanceto(x, y, angle, tankn, 0, 10000);   //last value is the maximum range
	while(run){
        checks++;   //increment the counter
        tangle+=scanangleincrement; //increment tangle
        dist = distanceto(x, y, tangle, tankn, acdist-30, acdist+30);   //find exact distance to target
        double c = 0 - cos(tangle); //set cos and sin of angle
        double s = sin(tangle);
        tx = x + c * dist;  //coordinates of edge of target at tangle from the tank
        ty = y + s * dist;
//        std::cout << "acdist: " << acdist << "\ndistance: " << dist << "\nangle: " << tangle << "\nx: " << tx << "\ny: " << ty << "\n";
        if(dist == 0)   //if distance is 0, target is not found at angle tangle from the tank within the given range
            run = false;
        else{
        tvisible = true;    //perform the same check as before but with a different angle
           	for(int i=0; i<tanks.size(); i++){
                if(i!=tankn && i!=tank1){
                    Shape temp = tanks[i]->getShape();
                    if((tvisible == true)&&(passedovershape(x, y, tx, ty, temp) || isInShape(x, y, temp))){
                        tvisible = false;
                    }
                }
            }
            for(int i=0; i<shapes.size(); i++){
                Shape* temp = shapes[i];
                if((tvisible == true)&&(passedovershape(x, y, tx, ty, *temp) || isInShape(x, y, *temp))){
                    tvisible = false;
                }
            }
        if(tvisible == true)
            run = false;    //break the loop
        }
	}
	double xpos;
	double ypos;
	double ang1 = tangle;
	if(tvisible){
        impactcoords(x, y, tangle, tankn, acdist-30, acdist+30, &xpos, &ypos);  //save coordinates of target that are visible
	}
	bool vispos = tvisible;
	tangle = angle; //reset tangle
	run = true; //repeat the above but in the opposite direction (clockwise as opposed to anti clockwise)
	while(run){
        checks++;
        tangle-=scanangleincrement;
        dist = distanceto(x, y, tangle, tankn, acdist-30, acdist+30);
        double c = 0 - cos(tangle);
        double s = sin(tangle);
        tx = x + c * dist;
        ty = y + s * dist;
//        std::cout << "distance: " << dist << "\nangle: " << tangle << "\nx: " << tx << "\ny: " << ty << "\n";
        if(dist == 0)
            run = false;
        else{
        tvisible = true;
           	for(int i=0; i<tanks.size(); i++){
                if(i!=tankn && i!=tank1){
                    Shape temp = tanks[i]->getShape();
                    if((tvisible == true)&&(passedovershape(x, y, tx, ty, temp) || isInShape(x, y, temp))){
                        tvisible = false;
                    }
                }
            }
            for(int i=0; i<shapes.size(); i++){
                Shape* temp = shapes[i];
                if((tvisible == true)&&(passedovershape(x, y, tx, ty, *temp) || isInShape(x, y, *temp))){
                    tvisible = false;
                }
            }
        if(tvisible == true)
            run = false;
        }
	}
	double xneg;
	double yneg;
	if(tvisible){
        impactcoords(x, y, tangle, tankn, acdist-30, acdist+30, &xneg, &yneg);
	}
	bool visneg = tvisible;
	if(vispos && visneg){   //select coordinates based on which are closer to the target
        if(ang1-angle < angle-tangle){
            *targx = xpos;
            *targy = ypos;
        }else{
            *targx = xneg;
            *targy = yneg;
        }
	}else if(vispos){
        *targx = xpos;
        *targy = ypos;
	}else if(visneg){
        *targx = xneg;
        *targy = yneg;
	}
//	std::cout << "CHECKS: " << checks << "\n";
	if(vispos || visneg){   //if target is visible return true
        return true;
    }
    return false;
}

double distanceto(double x, double y, double angle, int tank, double mini, double maxi){
    double dist = mini; //set minimum scanning distance
    double xt = x - cos(angle)*mini;    //set initial x and y coords
    double yt = y + sin(angle)*mini;
    Shape temp = tanks[tank]->getShape();
    while(!(passedovershape(x, y, xt, yt, temp) || isInShape(x, y, temp)) && dist < maxi){  //scan in given direction until past the maximum, or the target is encountered
        dist += 1;
        xt -= cos(angle);
        yt += sin(angle);
    }
    if(dist >= maxi)    //if target was not found return 0, otherwise return the distance
        return 0;
    return dist;
}

void impactcoords(double x, double y, double angle, int tank, double mini, double maxi, double* xpos, double* ypos){
    double dist = mini; //same as distanceto method, but with no return value, and the coordinates of the target if found are assigned to the given pointers
    double xt = x - cos(angle)*mini;
    double yt = y + sin(angle)*mini;
    Shape temp = tanks[tank]->getShape();
    while(!(passedovershape(x, y, xt, yt, temp) || isInShape(x, y, temp)) && dist < maxi){
        dist += 1;
        xt -= cos(angle);
        yt += sin(angle);
    }
    *xpos = xt;
    *ypos = yt;
}

double yfromx(double x, double x1, double y1, double xvel, double yvel){    //return y coordinate on a line at given x coordinate
    return x*yvel/xvel + y1 - x1*yvel/xvel;
}

double yfromxcoords(double x, double x1, double y1, double x2, double y2){
    double m = (y1-y2)/(x1-x2);
    double c = y1 - m * x1;
    return m*x + c;
}

bool passedovershape(double xa, double ya, double xb, double yb, Shape shape){
    for(int i=0;i<shape.x.size();i++){
        int j;
        i == shape.x.size()-1 ? j = 0 : j = i+1;
        if(passedover(xa, ya, xb, yb, shape.x[i], shape.y[i], shape.x[j], shape.y[j])){
            return true;
        }else if(shape.x[i] == shape.x[j]){
            if(passedover(ya, xa, yb, xb, shape.y[i], shape.x[i], shape.y[j], shape.x[j]))
                return true;
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
    return ((x<xa != x<xb) && (x<x1 != x<x2));
}

void sortpoints(std::vector<double>* xs, std::vector<double>* ys){
//    std::cout << "sortpoints\n";
    int length = xs->size();
    std::vector<double>* x = new std::vector<double>;
    std::vector<double>* y = new std::vector<double>;
    bool mark[length];
    double x0 = 0;
    double y0 = 0;
    for(int i=0;i<length;i++){
        x0+=xs->at(i);
        y0+=ys->at(i);
        mark[i] = true;

    }
    x0/=length; //middle point
    y0/=length;
    int i = 0;
    bool endloop = false;
    while(!endloop){
        int smallest = -1;
        for(int j=0;j<length;j++){
            if(mark[j]){
                if(xs->at(j) >= x0){
                    if(smallest == -1)
                        smallest = j;
                    else
                        if(atan((ys->at(j)-y0)/(xs->at(j)-x0)) < atan((ys->at(smallest)-y0)/(xs->at(smallest)-x0)))
                            smallest = j;
                }
            }
        }
        if(smallest == -1)
            endloop = true;
        else{
            x->push_back(xs->at(smallest));
            y->push_back(ys->at(smallest));
            mark[smallest] = false;
            i++;
        }
    }
    for(i;i<length;i++){
        int smallest = -1;
        for(int j=0;j<length;j++){
            if(mark[j]){
                if(xs->at(j) < x0){
                    if(smallest == -1)
                        smallest = j;
                    else
                        if(atan((ys->at(j)-y0)/(xs->at(j)-x0)) < atan((ys->at(smallest)-y0)/(xs->at(smallest)-x0)))
                            smallest = j;
                }
            }
        }
            x->push_back(xs->at(smallest));
            y->push_back(ys->at(smallest));
            mark[smallest] = false;
    }
    *xs = *x;
    *ys = *y;
    delete x;
    delete y;
}

bool collision(int index, double* gradient, bool* setgradient){
    for(int i=0;i<tanks.size();i++){
        if(i==index)
            ;
        else{
            if(checkcollision(tanks[i]->getShape(), tanks[index]->getShape(), gradient, setgradient))
                return true;
        }
    }
    for(int i=0;i<shapes.size();i++){
        Shape temp = *shapes[i];
        if(checkcollision(temp, tanks[index]->getShape(), gradient, setgradient)){
            return true;
        }
    }
    return false;
}

bool checkcollision(Shape shape1, Shape shape2, double* gradient, bool* setgradient){
    for(int i=0;i<shape1.x.size();i++){
        if(isInShape(shape1.x[i], shape1.y[i], shape2))
            return true;
    }
    for(int i=0;i<shape2.x.size();i++){
        if(isInShape(shape2.x[i], shape2.y[i], shape1)){
            int point1 = findSide(shape2.x[i], shape2.y[i], shape1);
            int point2;
            point1 == shape1.x.size()-1 ? point2 = 0 : point2 = point1 + 1;
            *setgradient = true;
            *gradient = (shape1.y[point1]-shape1.y[point2])/(shape1.x[point1]-shape1.x[point2]);
            return true;
        }
    }
    return false;
}

bool isInShape(double x, double y, Shape shape){
    bool isin = true;
    for(int i=0; i<shape.x.size(); i++){
        int im = i-1;
        if(im<0)
            im=shape.x.size()-1;
        int ip = i+1;
        if(ip==shape.x.size())
            ip=0;
        double Im = atan2(shape.x[i], shape.y[i], shape.x[im], shape.y[im]);
        double Ip = atan2(shape.x[i], shape.y[i], shape.x[ip], shape.y[ip]);
        double IT = atan2(shape.x[i], shape.y[i], x, y);
        if(Ip - Im > PI)
            Ip -= 2.0 * PI;
        if((IT < Im == IT < Ip) && (IT-2.0*PI < Im == IT-2.0*PI < Ip)){ //Sketchy maths that actually works
            isin = false;
        }
    }
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

int findSide(double x, double y, Shape shape){
    int length = shape.x.size();
    double nearest = nearestpoint(x, y, shape.x[length-1], shape.y[length-1], shape.x[0], shape.y[0]);
    int point = length-1;
    for(int i=0; i<length-1; i++){
        double temp = nearestpoint(x, y, shape.x[i], shape.y[i], shape.x[i+1], shape.y[i+1]);
        if(temp < nearest){
            nearest = temp;
            point = i;
        }
    }
    return point;
}

double nearestpoint(double xp, double yp, double x1, double y1, double x2, double y2){
    if(x1 == x2)
        return abs(xp-x1);
    if(y1 == y2)
        return abs(yp-y1);
    double as = abs(yp-yfromxcoords(xp, x1, y1, x2, y2));
    double dist;
    double ang = tan((x1-x2)/(y1-y2));
    dist = abs(as / sin(ang));
    return dist;
}

bool updateMouse(){
    SDL_Event e;
    bool quit = false;
    //{Handle input
    LmouseClick = 0;
    RmouseClick = 0;
    while( SDL_PollEvent( &e ) != 0 ){
        if( e.type == SDL_QUIT){
            quit = true;
        }else if( e.type == SDL_MOUSEBUTTONDOWN ){
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
    //}
    return quit;
}

void handleInput(Tank* player){
	if( keys[ fowardsKey ] ){
		player->accelerate(100);
	}else if( keys[ backwardsKey ] ){
		player->accelerate(-100);
	}else{
		player->accelerate(0);
	}
	if( !( keys[ leftKey ] && keys[ rightKey ] ) ){
		if( keys[ leftKey ] ){
			player->turn(-100);
		}else if( keys[ rightKey ] ){
			player->turn(100);
		}else{
			player->turn(0);
		}
	}
	if( keys[ brakeKey ] ){
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
    if( keys[ slomoKey] && slomo>0){
        slomo-=4.25*30*stepSize;
        if(slomospeed<maxslomospeed){
            slomospeed*=1.1;
        }else{
            slomospeed=maxslomospeed;
        }
    }else if(slomospeed>1){
        slomospeed/=1.1;
    }else{
        slomospeed=1;
    }
    if(slomo<100){
        slomo+=0.25*30*stepSize;
        if(player->getVel()<5)
            slomo+=0.25*30*stepSize;
    }
    if( keys[ quickMenuKey]){
        freeze = true;
    }else{
        freeze = false;
    }
    if( keys[ pauseKey])
        pause = true;
    //set key and toggle states:
    if(keys[primaryWeaponAssistKey]==true && primaryWeaponAssistKeyState==false)
        primaryWeaponAssistToggle=!primaryWeaponAssistToggle;
    primaryWeaponAssistKeyState=keys[primaryWeaponAssistKey];
}

int SCROLL_RenderDrawLine(SDL_Renderer* renderer, int x1, int y1, int x2, int y2){
    return SDL_RenderDrawLine(renderer, x1-camX, y1-camY, x2-camX, y2-camY);
}

void basicText(std::string text, SDL_Color textColour, int x, int y, double w, double h, bool background, uint8_t r, uint8_t g, uint8_t b, uint8_t a){
    SDL_Surface* textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    SDL_Texture* mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    SDL_Rect temp = {x-textSurface->w*w/2, y-textSurface->h*h/2, textSurface->w*w, textSurface->h*h};
    SDL_FreeSurface( textSurface );
    if(background){
        SDL_SetRenderDrawColor( gRenderer, r, g, b, a );
        SDL_RenderFillRect( gRenderer, &temp );
    }
    SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
    SDL_DestroyTexture(mTexture);
}

//}

