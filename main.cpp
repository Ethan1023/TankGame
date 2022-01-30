#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

/*GAME INFO:
	autosaving - lvl 1, save with heat going to 0 - lvl 2, save entering new room with no heat - lvl 3, save with no velocity and heat and frame%50=0
	can only use a teleporter if no heat
        reason - don't want the enemy to know you have access to teleporting
    materials are gained by touching dead enemies - two amounts, onperson, and safe. onperson is added to safe upon clearing heat, onperson is lost if killed.
*/

/*DOCUMENTATION:
	Ha good luck understanding anything
*/

/*TODO:
show name of weapon in upgrade weapon menu
in weapon menu make it easier to see current equipped weapons
label columns in upgrade menus`

save and load these from the tank class:
timeSinceSeen
chaseTime
xPath
yPath
roomPath
toprocess
patrolling

implement processing path and patrols

possibly add maxradius and outsiderectangle to shape - if outside one of those there is no collision
add in precalcs - memory usage of program is still very low, but cpu usage is a bit higher - complete static calculations beforehand
double check checkcollision running, see if there is any way to optimise
add in more timers to find what chews up what time

possibly only run enemy AI once every couple of frames

add semiautomatic weapons

add weapon levelling - spend materials to boost base stats significantly, same for other stats possibly (maybe with special item), or something else to make upgrades work past level 30

add in portal unlock cost and state, the state is saved and loaded with the id

when dead, pay materials to respawn or load previous save - covered I think

disable projectile stepping while frozen
minimap, and fullscreen map

IDEAS:
    enemy AI only runs for enemies in the same room, but projectiles and traileffects run
    long range teleporters at end of each area - cost significant materials to unlock, plus possibly special items
    xp gained on death as "what not to do next time"
    respawning as person is free
    why can respawning happen (in game explanation):
        tank control is well protected, if tank is "killed", player simply teleports out, and enemies don't care cos they think you're dead
        after body is disposed of / people leave the area, you are teleported back to base and revived
    fix no response to exit signal

BUGS:
    crashes after loading game and then returning to menu and starting new game
	turning and moving at the same time - glitch into wall - maybe not too bad
	enemies dont go back through portals
	screen shake when at minimum zoom and tankx-camx = -1*(mousex-camx)
	stuck on corners - checkcollision possibly
*/

/*
long debug - tanks saving coordinates as a path, when stuck on wall whilst chase timer overflows targetX = x and targetY = y, leading to undefined rotation atan((targetY-y)/(targetX-x))
leading to undefined cos and sin of angle, leading to undefined xvel and yvel and undefined x and y. calls collision method, uses getshape on tank, undefined x and y pushed on to vector
sortpoints called with undefined coordinates, all values are nan, so smallest doesn't get changed from -1, so it is attempted to access the coordinate -1 of a vector causing an error saying
value 18446744073709551615 is out of range for a vector of size 8
*/

//{DECLARATIONS

double globaltemp;
int shapecollide;

//{timing
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;
const bool timing = true;
std::vector<uint32_t> AIruntimes;
std::vector<uint32_t> otherruntimes;
std::vector<uint32_t> stepruntimes;
//}

//{CONSTANTS & VARIABLES
//constants
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 960;
const double PI = 3.14159265358979323846;

//mouse and camera
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

//rendering
SDL_Window* gWindow;
SDL_Renderer* gRenderer = NULL;
TTF_Font *gFont = NULL;
int fontSize = 28;
double xzoom=1;//two variables makes it easier to have fun later on
double yzoom=1;

//game speed and framerate
double stepSize = 0.01; //speed of game - adjusted based on framerate
double TICKS_PER_FRAME;
double FPS = 60;
double slomospeed = 1;
double maxslomospeed = 16;
double slomo=100;
double slomoregen=1;
double slomousage=1;
double slomoresponse=1;
uint32_t frameCount = 0;
uint32_t lastpause;
uint32_t pauseduration;
bool recentpause = false;

//calculation performance
double scanangleincrement = 0.001; //increment when searching for a line of sight to a target, larger = lower vision precision, better performance
uint32_t precisevisionfrequency = 30; //how often precise vision is used in cansee for a much higher cpu cost - can be quite high as it realistically may take enemies a second to spot player if they can only see the very corner
//cansee costs 1/(precise*scan) = time cost ratio
double circleres = 8;  //number of points generated in a circular shape, higher = more accurate collision detection, lower performance

//graphics
double traileffectprescision = 15;
uint32_t effectIDs = 0;

//menu and keys
bool pause = false;
bool menu = false;
const Uint8* keys = SDL_GetKeyboardState( NULL );

//game
bool freeze = false;
bool aiminglaser = true;
bool heat = false;
double heatnum = 0;
int maxrepairs=5;//reset from character file
const int pointsperlevel=5;//constant

//cheats
bool theflash=false;

//saving
uint8_t saved=0;
uint32_t savetime=0;

//AI
bool processingAvailable=true;

//other
bool showSecondary = false;//display secondary weapon stats in quick menu

//settings
double MAX_FPS = 60;    //maximum framerate
bool particleEffects = false;
double streakrenderres = 0.01;  //how accurately the streak left behind a projectile will be rendered close to a hit target - smaller = more accurate
int physics = 3;	//physics loops per graphical loop
double mul = 2;     //scale of tanks and projectiles physical sizes
double gamespeed = 2;   //actual speed of game
double firstup=1.1;//potency of first upgrade
double deterioration=0.9;//deterioration of each sequential upgrade
int savelevel=2;

//bool damagefx = false;
//{controls:
SDL_Scancode fowardsKey = SDL_SCANCODE_W;
SDL_Scancode backwardsKey = SDL_SCANCODE_S;
SDL_Scancode leftKey = SDL_SCANCODE_A;
SDL_Scancode rightKey = SDL_SCANCODE_D;
SDL_Scancode brakeKey = SDL_SCANCODE_SPACE;
SDL_Scancode slomoKey = SDL_SCANCODE_LSHIFT;
SDL_Scancode quickMenuKey = SDL_SCANCODE_LCTRL;
SDL_Scancode pauseKey = SDL_SCANCODE_ESCAPE;
SDL_Scancode menuKey = SDL_SCANCODE_TAB;
SDL_Scancode primaryWeaponAssistKey = SDL_SCANCODE_LALT;
SDL_Scancode portalKey = SDL_SCANCODE_F;
SDL_Scancode zoomKey = SDL_SCANCODE_GRAVE;
bool primaryWeaponAssistKeyState = false;
bool primaryWeaponAssistToggle = false;

bool portalKeyState = false;
bool portalKeyPress = false;
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
    uint16_t room=0;
};

struct Projectile{
	uint16_t room=0;
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
    double range=0;
    bool passesShield=false;
    ParticleLine* traileffect;

	double totaldist=0;//total distance
    int dist=0;//steps travelled on current trail effect segment
    uint32_t lastid=0;

    int ownerID;
    bool primaryWeapon;
};

struct Shape{
    std::vector<double> x;
    std::vector<double> y;
    bool isRectangle=false;
    bool isCircle=false;
    double x0 = 0;
    double y0 = 0;
    double radius = 0;
    uint16_t room=0;
    std::vector<double> prevangle;
    std::vector<double> nextangle;
};

struct Weapon{
    std::string name;
    uint32_t ID;
    double speed;
    uint32_t range;
    int ROF;
    double spread;
    int burst;
    double damage;
    bool passesshields;
    double energy;
    bool primary;
    uint32_t cost;
};

struct WeaponUpgrades{//each upgrade increases value as described in struct Character, if value is an int round down to nearest worst whole value
	int ID;
	int points;
	int speed=0;
	int ROF=0;
	int spread=0;
	int burst=0;
	int damage=0;
	int energy=0;
	int range=0;
};

struct Character{
	std::string name;
	uint32_t materials;
	uint32_t safematerials;
	uint32_t exp;//store exp, every level needs 1000 exp, but enemies give lower rewards the more exp you have
	int repairs;

	double power;
	double brake;
	double maxVelocity;
	double mass;
	double energy;
	double generation;
	double health;
	double shield;
	double regen;
	double delay;
	double slomomax;
	double slomoregen;
	double slomousage;
	double slomoresponse;
	int maxrepairs;

	int powerup;//power = power*for(i in powerup) 1.1^(0.9^i)...powerup=3, power=power*1.1*1.1^0.9*(1.1^0.9)^0.9
	int brakeup;//primitive, but causes upgrades to gradually become less effective so other upgrades become more worthwhile
	int maxVelocityup;
	int massdown;
	int energyup;
	int generationup;
	int healthup;
	int shieldup;
	int regenup;
	int delayup;
	int slomomaxup;
	int slomoregenup;
	int slomousageup;
	int slomoresponseup;
	int maxrepairsup;

	int upgradepoints;
	int totalweaponpoints;
	std::vector<int> weapons;//contains IDs
	std::vector<WeaponUpgrades> weaponUpgrades;
	int primaryID;
	int secondaryID;
};

struct Portal{
	int room;
	int x;
	int y;
	int w;
	int h;
	uint32_t id;
	uint32_t toid;
	bool inwall;
};

struct RoomDimension{
	int mapwidth;
	int mapheight;
	int room;
};
//}

//{ENUMERATIONS
enum command { ACCELERATE, BRAKE, TURN, TURNTURRET, BRAKETURRET };
//}

//{CLASSES
class Tank{
	public:
        Tank();
		Tank(std::string, double, double, double, uint16_t);
		void positionReset(std::string, double, double, double, uint16_t, bool);
		~Tank();
		void step();
		void patrol();
		void render();
		void free();
		void loadLook0();
		void loadLook(std::string, std::string, std::string, std::string, std::string, PercentRect*, PercentRect*, PercentRect*, PercentRect*, PercentRect*, SDL_Rect*, SDL_Rect*, SDL_Rect*, SDL_Rect*, SDL_Rect*);
		void setDimensions(double, double, double, double, double, double, double, double, double);
		void setOther(int, double, double, double, double, double, double, uint32_t, uint32_t);
		void setWeapons(double, double, double, double, int, int, double, double, int, int, ParticleLine, ParticleLine, bool, bool, double, double);
		bool primaryReloaded();
		bool secondaryReloaded();
		void setHealth(double, double, double, int);
		uint32_t takeDamage(double, bool);
		bool isDead();
		double getLength();
		int getID();
		uint32_t getexp();
		uint32_t getmaterials();
		void strip();
		bool isStripped();
		double getViewDistance(bool);
		void broadcast();
		void boostView();
				//commands
        void settargetvel(double);
        void pointTo(double, double);
        void shootPrimary(Projectile*);
        void shootSecondary(Projectile*);
        void accelerate(double);	//accelerate with power
		void brake(double);		//brake with force
		void turn(double);		//set power distribution between tracks (to steer)
				//accessors
		double getShieldTimer();
		double getShieldDelay();
        bool hasseentarget();
        void getTarget(double**, double**);
        double getHealth();
        double getMaxHealth();
        double getShield();
        double getMaxShield();
        double getShieldRegen();
		double getX();			//get x position
		double getY();			//get y position
		double getOldX();
		double getOldY();
		int getCurrentRoom();
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
		void loadfromcharacter();
		//AI
		void resetTimeSinceSeen();
		void incrementTimeSinceSeen(double);
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
		uint8_t stuckness=0; //value to track how long stuck for
		double x;			//x position
		double y;			//y position
		double oldx;
		double oldy;
		double xvel;			//x velocity
		double yvel;			//y velocity
		double vel;             //velocity
		double tankrot;			//rotation of tank
		uint16_t currentRoom;
		uint32_t markedPortal;
		double turretrot;		//rotation of turret relative to tank
		double targetX = -1000;
		double targetY = -1000;
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
        uint32_t primaryRange;
        uint32_t secondaryRange;
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
        double shieldDelay;
        double health;
        double shield;
        double shieldCount=0;
        bool dead=false;
        bool stripped=false;//has player scavenged materials(increases respawn time)

        //AI
        double viewDistance=500;
        double boostedViewDistance=500;
        double broadcastDistance=500;
        double chaseTime=10;
		double timeSinceSeen=chaseTime;
		std::vector<double> xPath;
		std::vector<double> yPath;
		std::vector<uint16_t> roomPath;
		bool toprocess=false;
		bool patrolling=true;

        uint32_t expgiven=0;
        uint32_t materialsgiven=0;

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
Character loadCharacter(char*);
Character updateCharacter(char*, Character);
std::vector<Weapon> loadWeapons(char*);
void setMap(char*);
void loadMap(std::vector<Shape*>*, char*);
uint8_t mainMenu();
uint8_t gameMenu();
uint8_t selectWeaponMenu(Character*);
uint8_t upgradeWeaponMenu(WeaponUpgrades*, uint32_t*, Character*);
void reloadCharacter();
void addButton(std::vector<Shape*>*, double, double, double, double);
uint8_t runGame(std::string*);
void save(std::string*);
void close();
bool updateMouse(bool capzoom=false);
void handleInput(Tank*);	//Reads keyboard input, and sends controls to the Tank passed in
bool stepProjectile(Projectile*);
bool cansee(int, int, double*, double*);

bool process(std::vector<double>*, std::vector<double>*, std::vector<uint16_t>*);
bool noobstacle(double, double, double, double, uint16_t);

double yfromx(double, double, double, double, double);
void fire(bool, bool, Tank*, std::vector<Projectile*>*);
Portal portalEnter(int);
char sideof(Portal);
Shape portalToShape(Portal);
RoomDimension getRoomDimension(int);
bool collision(int);
bool passedovershape(double, double, double, double, Shape);
bool passedover(double, double, double, double, double, double, double, double);
void sortpoints(std::vector<double>*, std::vector<double>*);
bool isInShape(double, double, Shape);
void preLoadAngles(Shape*);
double atan2(double, double, double, double);
bool checkcollision(Shape, Shape);//, double*, bool*);
bool checkcollision(Shape, Shape, double, double);
double findCollisionAngle(double, double, double, double, Shape);
int findSide(double, double, Shape);
double nearestpoint(double, double, double, double, double, double);
double distanceto(double, double, double, int, double, double);
void impactcoords(double, double, double, int, double, double, double*, double*);
double xToDispX(double);
double yToDispY(double);
int SCROLL_RenderDrawLine(SDL_Renderer*, int, int, int, int);
uint8_t pauseMenu(std::string*);
void saveMenu(std::string*);
std::string loadMenu();
void basicText(std::string, SDL_Color, int, int, double, double, bool, uint8_t, uint8_t, uint8_t, uint8_t);
void verybasicText(std::string, int, int, double);

void saveParticleLine(ParticleLine, std::ofstream*);
ParticleLine loadParticleLine(std::ifstream*);
void saveShape(Shape, std::ofstream*);
Shape loadShape(std::ifstream*);
void saveProjectile(Projectile, std::ofstream*);
Projectile loadProjectile(std::ifstream*);
void saveTank(Tank, std::ofstream*);
Tank loadTank(std::ifstream*);
void saveWeapon(Weapon, std::ofstream*);
Weapon loadWeapon(std::ifstream*);
void saveCharacter(Character, std::ofstream*);
Character loadCharacter(std::ifstream*);
void saveWeaponUpgrades(WeaponUpgrades, std::ofstream*);
WeaponUpgrades loadWeaponUpgrades(std::ifstream*);
void saveStatus(std::ofstream*);
void loadStatus(std::ifstream*);

//}

//{CONSTANTS AND VARIABLES RELYING ON THE ABOVE
Tank* showDetailsFor = NULL;
std::vector<Tank*> tanks;
std::vector<Tank> tankTemplates;
std::vector<Shape*> shapes;
std::vector<ParticleLine> particlelines;
Character character;
std::vector<Weapon> weapons;
std::vector<Portal> portals;
std::vector<RoomDimension> roomDimensions;
std::vector<Projectile*> projectiles;
//}

//}

//{DEFINITIONS

//{TANK

void Tank::step(){
	auto t1step = Clock::now();
	oldx=x;
	oldy=y;
	if(theflash && index==0)//the flash cheat - move full speed during slomo
		stepSize*=slomospeed;
	if(!dead){
    if(index!=0){
		if(vel<5 && primaryReloaded() && secondaryReloaded())
			health+=0.000001*(maxHealth-health)*300*stepSize*300/physics;
        double dist = sqrt((targetX-x)*(targetX-x) + (targetY-y)*(targetY-y));
        if(targetvel==0){
            if(dist < length/2 && vel > targetvel && toprocess){
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
		if((targetX!=-1000 || targetY!=-1000)&&!patrolling){
			if((frameCount+index)%100==0){
				xPath.push_back(x);
				yPath.push_back(y);
				roomPath.push_back(currentRoom);
				toprocess=true;
			}
        }
    }else{//health regen for player - occurs with no heat, full energy and almost complete stop
		if(heatnum==0){
			character.safematerials+=character.materials;
			character.materials=0;
			if(energy==maxEnergy && vel<5 && health<maxHealth)
				health+=0.000001*(maxHealth-health)*(maxHealth-health)*stepSize*300/physics;
		}
		if(health>maxHealth)
			health==maxHealth;
    }
    if(primaryReload<primaryROF-30*200.0*stepSize*3/physics)//reload weapons
        primaryReload+=30*200.0*stepSize*3/physics;
	else
		primaryReload=primaryROF;
    if(secondaryReload<secondaryROF-30*200.0*stepSize*3/physics)
        secondaryReload+=30*200.0*stepSize*3/physics;
	else
		secondaryReload=secondaryROF;
    if(shieldCount<=shieldDelay)//regenerate shield
        shieldCount+=stepSize*3/physics;
    if(shieldCount>=shieldDelay){
		shieldCount=shieldDelay;
        shield+=shieldRegen*stepSize*3/physics;
	}
    if(shield>maxShield)
        shield=maxShield;
    if(energy<maxEnergy)
        energy+=energyGen*stepSize*3/physics;
    else
        energy=maxEnergy;
	if(targetX!=-1000 || targetY!=-1000){
    if(index!=0){
		if(targetX==x){//avoid divide by zero error
			if(targetY<y)
				tankrot = 90;
			else
				tankrot = 270;
		}else{
			tankrot = (atan((targetY - y)/(targetX - x))*180/PI);
        }
        targetX < x ? tankrot += 180 : tankrot;
    }
    if(targetX==x){
		if(targetY<y)
			tankrot = 90-tankrot;
		else
			tankrot = 270-tankrot;
    }else{
		turretrot = (atan((targetY - y)/(targetX - x))*180/PI) - tankrot;
    }
    targetX >= x ? turretrot += 180 : turretrot;
	}
	}
  if(stuckness<100 || !(rand()%100)){//move
	if(!dead || (index==0 && health>=0)){
	tankrot += commands[TURN] * stepSize * sqrt(fabs(vel)+1) / 10;//try turning
	if(collision(index)){
        tankrot -= commands[TURN] * stepSize * sqrt(abs(vel)+1) / 10;
        //std::cout << "Collision Detected...\n";
	}
	}
	tankrot > 360 ? tankrot -= 360 : (tankrot < 0 ? tankrot += 360 : tankrot = tankrot);
	double tankrotrad = (tankrot) * PI / 180;    //convert from visual rotation to rotation for calculation
	double usedpower = commands[ACCELERATE] * power / 100;  //calculate power used
	double drag = vel * vel / 10;   //drag so that tank can coast to a stop
	double braking = commands[BRAKE] * brakeforce / 100 + 10 + drag;   //strength of brakes being applied
	if(dead){
		usedpower=0;
		braking=10+drag;
	}
	double thrust = usedpower / ( abs(vel) + 0.1 );	//calculate thrust at current velocity
	if(thrust>0)
		thrust > usedpower/10 ? thrust = usedpower/10 : thrust = thrust;
	else
		thrust < usedpower/10 ? thrust = usedpower/10 : thrust = thrust;
	double dvel;
											//calculate acceleration provided to tracks by engine and brakes
	if( abs( thrust ) > braking ){							//tracks are powered by the engine
		vel > 0 ? thrust -= braking : thrust += braking;			//calculate net thrust
		dvel = stepSize * thrust / mass;				//calculate acceleration
	}else{										//tracks are braking
		thrust = braking - abs(thrust);						//calculate net braking
		vel > 0 ? dvel = -1 * stepSize * thrust / mass : dvel = stepSize * thrust / mass;				//calculate acceleration
		if(fabs(dvel)>fabs(vel)){
			dvel=0;
			vel=0;
		}
	}
    vel += dvel;
    if(vel > maxvel)
        vel = maxvel;
    else if(fabs(vel) > maxvel)
        vel = -maxvel;
    xvel = cos(tankrotrad)*vel;
    yvel = sin(tankrotrad)*vel;

    x+=stepSize*xvel;//try moving, in the event of a collision, try moving at an angle to the rotation and add friction if not a circle
    y+=stepSize*yvel;
if(timing){
auto t2 = Clock::now();
otherruntimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1step).count());
if(otherruntimes.size()>100)
otherruntimes.erase(otherruntimes.begin()+0);
}
    if(collision(index)){//crash triggered from here
            y-=stepSize*yvel;
            x-=stepSize*xvel;
		double firstx=x;
		double firsty=y;
		int32_t circle=std::round(PI*1000000*2);
		int32_t op1=(int32_t)(std::round((globaltemp+PI/2.0-tankrotrad)*1000000)+circle/2)%circle-circle/2;
		int32_t op2=(int32_t)(std::round((globaltemp+PI/2.0+PI-tankrotrad)*1000000)+circle/2)%circle-circle/2;
		if(abs(op1)<abs(op2)){
			x+=stepSize*cos(globaltemp+PI/2.0)*(vel);
			y+=stepSize*sin(globaltemp+PI/2.0)*(vel);
		}else{
			x-=stepSize*cos(globaltemp+PI/2.0)*(vel);
			y-=stepSize*sin(globaltemp+PI/2.0)*(vel);
		}

		if(collision(index)){
			x=firstx;
			y=firsty;
			vel/=2;
			if(index!=0 && stuckness<100){
				stuckness++;
			}
		}else{
			stuckness=0;
			if(!iscircle || usedpower==0){
				if(abs(op1)<abs(op2)){
					vel*=pow(fabs(cos((double)op1/1000000.0)), 0.5);//wall drag - based on cosine of difference between forwards and wall
				}else{
					vel*=pow(fabs(cos((double)op2/1000000.0)), 0.5);
				}
			}
		}
/*legacy collision detection (brute force tactic)
		bool solved=false;
		bool maxangle=PI/2.0;
        for(double i=tankrotrad-globaltemp-0.1; i<maxangle; i+=0.1){//i=0.1
            x+=stepSize*cos(tankrotrad+i)*vel;
            y+=stepSize*sin(tankrotrad+i)*vel;
            if(collision(index)){
                x-=stepSize*cos(tankrotrad+i)*vel;
                y-=stepSize*sin(tankrotrad+i)*vel;
                x+=stepSize*cos(tankrotrad-i)*vel;
                y+=stepSize*sin(tankrotrad-i)*vel;
                if(collision(index)){
                    x-=stepSize*cos(tankrotrad-i)*vel;
                    y-=stepSize*sin(tankrotrad-i)*vel;
                }else{
                    solved = true;
                    x-=stepSize*cos(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                    y-=stepSize*sin(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                    if(collision(index)){
                        x+=stepSize*cos(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                        y+=stepSize*sin(tankrotrad-i)*vel*(1-cos(i))*(1-cos(i));
                    }
                    if(!iscircle)
                        vel*=sqrt(fabs(cos(i)));
					else if(usedpower==0)
						vel*=sqrt(sqrt(fabs(cos(i))));
//					std::cout << "Long: " << tankrotrad-i << "\n";
                    i=PI;
                }
            }else{
                solved = true;
                x-=stepSize*cos(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                y-=stepSize*sin(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                if(collision(index)){
                    x+=stepSize*cos(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                    y+=stepSize*sin(tankrotrad+i)*vel*(1-cos(i))*(1-cos(i));
                }
                if(!iscircle)
                    vel*=sqrt(fabs(cos(i)));
//				std::cout << "Long: " << tankrotrad+i << "\n";
                i=PI;
            }
        }*/
	}
	if(portalEnter(index).id!=0){//check if colliding with exit portal
		Portal exitPortal = portalEnter(index);
		if(exitPortal.inwall){
			markedPortal = exitPortal.id;
		}else if (portalKeyPress && heatnum==0 && index==0 && !dead){
			if(savelevel>=2 && (savetime==0 || (SDL_GetTicks()+500)<savetime)){
				savetime=SDL_GetTicks()+500;
			}
			uint32_t toid = exitPortal.toid;
			for(int i=0;i<portals.size();i++){
				if(portals[i].id==toid)
					exitPortal=portals[i];
			}
			currentRoom = exitPortal.room;
			x=exitPortal.x+exitPortal.w/2;
			y=exitPortal.y+exitPortal.h/2;
		}
	}
	if(y<-100 || y>getRoomDimension(currentRoom).mapheight+100 || x<-100 || x>getRoomDimension(currentRoom).mapwidth+100){
		Portal entryPortal = portals[0];
		for(int i=0;i<portals.size();i++){
			if(portals[i].id==markedPortal)
				entryPortal=portals[i];
		}
		Portal exitPortal = portals[0];
		for(int i=0;i<portals.size();i++){
			if(portals[i].id==entryPortal.toid)
				exitPortal=portals[i];
		}
		currentRoom = exitPortal.room;
		switch (sideof(entryPortal)){
			case 'n':
				switch (sideof(exitPortal)){
					case 'n':
						x=exitPortal.x+exitPortal.w-exitPortal.w*(x-entryPortal.x)/(entryPortal.w);
						y=-100;
                        tankrot+=180;
					break;
					case 'e':
						y=exitPortal.y+exitPortal.h-exitPortal.h*(x-entryPortal.x)/(entryPortal.w);
						x=getRoomDimension(currentRoom).mapwidth+100;
						tankrot-=90;
					break;
					case 's':
						x=exitPortal.x+exitPortal.w*(x-entryPortal.x)/(entryPortal.w);
						y=getRoomDimension(currentRoom).mapheight+100;
					break;
					case 'w':
						y=exitPortal.y+exitPortal.h*(x-entryPortal.x)/(entryPortal.w);
						x=-100;
						tankrot+=90;
					break;
					default:
						x=exitPortal.x+exitPortal.w/2;
						y=exitPortal.y+exitPortal.h/2;
						if(y<-100)
							y=-100;
						else if(y>getRoomDimension(currentRoom).mapheight+100)
							y=getRoomDimension(currentRoom).mapheight+100;
						if(x<-100)
							x=100;
						else if(x>getRoomDimension(currentRoom).mapwidth+100)
							x=getRoomDimension(currentRoom).mapwidth+100;
					break;
				}
			break;
			case 'e':
				switch (sideof(exitPortal)){
					case 'n':
						x=exitPortal.x+exitPortal.w-exitPortal.w*(y-entryPortal.y)/(entryPortal.h);
						y=-100;
						tankrot+=90;
					break;
					case 'e':
						y=exitPortal.y+exitPortal.h-exitPortal.h*(y-entryPortal.y)/(entryPortal.h);
						x=getRoomDimension(currentRoom).mapwidth+100;
						tankrot+=180;
					break;
					case 's':
						x=exitPortal.x+exitPortal.w*(y-entryPortal.y)/(entryPortal.h);
						y=getRoomDimension(currentRoom).mapheight+100;
						tankrot-=90;
					break;
					case 'w':
						y=exitPortal.y+exitPortal.h*(y-entryPortal.y)/(entryPortal.h);
						x=-100;
					break;
					default:
						x=exitPortal.x+exitPortal.w/2;
						y=exitPortal.y+exitPortal.h/2;
						if(y<-100)
							y=-100;
						else if(y>getRoomDimension(currentRoom).mapheight+100)
							y=getRoomDimension(currentRoom).mapheight+100;
						if(x<-100)
							x=100;
						else if(x>getRoomDimension(currentRoom).mapwidth+100)
							x=getRoomDimension(currentRoom).mapwidth+100;
					break;
				}
			break;
			case 's':
				switch (sideof(exitPortal)){
					case 'n':
						x=exitPortal.x+exitPortal.w*(x-entryPortal.x)/(entryPortal.w);
						y=-100;
					break;
					case 'e':
						y=exitPortal.y+exitPortal.h*(x-entryPortal.x)/(entryPortal.w);
						x=getRoomDimension(currentRoom).mapwidth+100;
						tankrot+=90;
					break;
					case 's':
						x=exitPortal.x+exitPortal.w-exitPortal.w*(x-entryPortal.x)/(entryPortal.w);
						y=getRoomDimension(currentRoom).mapheight+100;
						tankrot+=180;
					break;
					case 'w':
						y=exitPortal.y+exitPortal.h-exitPortal.h*(x-entryPortal.x)/(entryPortal.w);
						x=-100;
						tankrot-=90;
					break;
					default:
						x=exitPortal.x+exitPortal.w/2;
						y=exitPortal.y+exitPortal.h/2;
						if(y<-100)
							y=-100;
						else if(y>getRoomDimension(currentRoom).mapheight+100)
							y=getRoomDimension(currentRoom).mapheight+100;
						if(x<-100)
							x=100;
						else if(x>getRoomDimension(currentRoom).mapwidth+100)
							x=getRoomDimension(currentRoom).mapwidth+100;
					break;
				}
			break;
			case 'w':
				switch (sideof(exitPortal)){
					case 'n':
						x=exitPortal.x+exitPortal.w*(y-entryPortal.y)/(entryPortal.h);
						y=-100;
						tankrot-=90;
					break;
					case 'e':
						y=exitPortal.y+exitPortal.h*(y-entryPortal.y)/(entryPortal.h);
						x=getRoomDimension(currentRoom).mapwidth+100;
					break;
					case 's':
						x=exitPortal.x+exitPortal.w-exitPortal.w*(y-entryPortal.y)/(entryPortal.h);
						y=getRoomDimension(currentRoom).mapheight+100;
						tankrot+=90;
					break;
					case 'w':
						y=exitPortal.y+exitPortal.h-exitPortal.h*(y-entryPortal.y)/(entryPortal.h);
						x=-100;
						tankrot+=180;
					break;
					default:
						x=exitPortal.x+exitPortal.w/2;
						y=exitPortal.y+exitPortal.h/2;
						if(y<-100)
							y=-100;
						else if(y>getRoomDimension(currentRoom).mapheight+100)
							y=getRoomDimension(currentRoom).mapheight+100;
						if(x<-100)
							x=100;
						else if(x>getRoomDimension(currentRoom).mapwidth+100)
							x=getRoomDimension(currentRoom).mapwidth+100;
					break;
				}
			break;
			default:
			x=exitPortal.x+exitPortal.w/2;
			y=exitPortal.y+exitPortal.h/2;
			if(y<-100)
				y=-100;
			else if(y>getRoomDimension(currentRoom).mapheight+100)
				y=getRoomDimension(currentRoom).mapheight+100;
			if(x<-100)
				x=100;
			else if(x>getRoomDimension(currentRoom).mapwidth+100)
				x=getRoomDimension(currentRoom).mapwidth+100;
		}
		if(index!=0){
			if(x<0){
				targetX=50;
				targetY=y;
			}else if(x>getRoomDimension(currentRoom).mapwidth){
				targetX=getRoomDimension(currentRoom).mapwidth-50;
				targetY=y;
			}else if(y<0){
				targetY=50;
				targetX=x;
			}else if(y>getRoomDimension(currentRoom).mapheight){
				targetY=getRoomDimension(currentRoom).mapheight-50;
				targetX=x;
			}
		}else if(heatnum==0){
			if(savelevel>=2 && (savetime==0 || (SDL_GetTicks()+500)<savetime)){
				savetime=SDL_GetTicks()+500;
			}
		}
	}
  }
	if(dead){
		if(health>=maxHealth){
			dead=false;
			stripped=false;
			health=maxHealth;
		}else if(health>=0){
			if(index==0)
				health+=(stepSize/physics)*maxHealth/5;
			else if(stripped)
				health+=stepSize/physics*maxHealth/100;
			else
				health+=stepSize/physics*maxHealth/30;
		}
	}

	if(theflash && index==0)
		stepSize/=slomospeed;
	if(collision(index)){//automatically undo any remnant collision, causes infinite loop if no space for tank in room
		double i=1;
		while(i!=0){
			for(double j=0;j<2*PI;j+=0.1){
				x+=cos(j)*i;
				y+=sin(j)*i;
				if(collision(index) || x<-100 || x>roomDimensions[currentRoom].mapwidth+100 || y<-100 || y>roomDimensions[currentRoom].mapheight+100){
					x-=cos(j)*i;
					y-=cos(j)*i;
				}else{
					i=0;
					j=2*PI;
				}
			}
			i*=1.5;
		}
    }
    if(timing){
		auto t2step = Clock::now();
		stepruntimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t2step - t1step).count());
		if(stepruntimes.size()>100)
			stepruntimes.erase(stepruntimes.begin()+0);
	}
}

void Tank::patrol(){
	if(xPath.size()==0){
		brake(100);
		accelerate(100);
		//normal patrol
	} else if(toprocess && !process(&xPath, &yPath, &roomPath)){
		brake(100);
		accelerate(0);
	} else {
		toprocess=false;
		brake(0);
        accelerate(100);
        targetX=xPath[xPath.size()-1];
		targetY=yPath[yPath.size()-1];
        while(xPath.size()>0 && fabs(x-xPath[xPath.size()-1])<5 && fabs(y-yPath[yPath.size()-1])<5){
			xPath.erase(xPath.begin()+xPath.size()-1);
			yPath.erase(yPath.begin()+yPath.size()-1);
			roomPath.erase(roomPath.begin()+roomPath.size()-1);
        }
	}
}

void Tank::pointTo(double targx, double targy){
    targetX = targx;
    targetY = targy;
}

void Tank::shootPrimary(Projectile* p){
    primaryReload = 0;
    energy-=pEnergy;
    if(energy<0)
        energy=0;
    p->x = x-cos((tankrot+turretrot)*PI/180)*(turretradius + barrellength);
    p->y = y-sin((tankrot+turretrot)*PI/180)*(turretradius + barrellength);
    p->angle = turretrot+tankrot;
    if(pspread>0)
		if(rand()%2 == 0)
			p->angle += pow(((rand()%100)*1.0), 4)/(200000000.0)*pspread;
		else
			p->angle -= pow(((rand()%100)*1.0), 4)/(200000000.0)*pspread;
	double tmp = 0.8+(rand()%401)/1000.0;
    p->xvel = xvel-cos(p->angle*PI/180)*primaryProjectileSpeed*tmp;
    p->yvel = yvel-sin(p->angle*PI/180)*primaryProjectileSpeed*tmp;
    p->width = pprojwidth;
    p->length = pprojlength;
    p->look = pproj;
    p->clip = &pprojclip;

    p->traileffect = &Peffect;
    p->damage = primaryDamage;
    p->range = primaryRange*(0.9+(rand()%201)/1000.0);
    p->passesShield = pPassesShield;

    p->ownerID=ID;
    p->primaryWeapon=true;
    p->room = currentRoom;
}

void Tank::shootSecondary(Projectile* p){
    secondaryReload = 0;
    energy-=sEnergy;
    if(energy<0)
        energy=0;
    p->x = x-cos((tankrot+turretrot)*PI/180)*(turretradius + barrellength);
    p->y = y-sin((tankrot+turretrot)*PI/180)*(turretradius + barrellength);
    p->angle = turretrot+tankrot;
    if(sspread>0)
        p->angle += ((rand()%1000)*1.0)/(500/(sspread)) - sspread;
	double tmp = 0.8+(rand()%401)/1000.0;
    p->xvel = xvel-cos(p->angle*PI/180)*secondaryProjectileSpeed*tmp;
    p->yvel = yvel-sin(p->angle*PI/180)*secondaryProjectileSpeed*tmp;

    p->width = sprojwidth;
    p->length = sprojlength;
    p->look = sproj;
    p->clip = &sprojclip;

    p->traileffect = &Seffect;
    p->damage = secondaryDamage;
    p->range = secondaryRange*(0.8+(rand()%401)/1000.0);
    p->passesShield = sPassesShield;

    p->ownerID=ID;
    p->primaryWeapon=false;
    p->room=currentRoom;
}

Tank::Tank(){};

Tank::Tank(std::string nname, double xpos, double ypos, double rot, uint16_t room){
    stuckness=0;
    if(nname==""){
        nname="unnamed";
    }
	positionReset(nname, xpos, ypos, rot, room, true);
}

void Tank::positionReset(std::string nname, double xpos, double ypos, double rot, uint16_t room, bool reset){
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
	currentRoom = room;
}

Tank::~Tank(){
	//free();
}

void Tank::setOther(int nID, double npower, double nbrakeforce, double nmaxvel, double nmass, double nmaxEnergy, double nenergyGen, uint32_t exp, uint32_t materials){
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
    if(exp>0)
		expgiven=exp;
	if(materials>0)
		materialsgiven=materials;
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
	SDL_Rect temp = {xToDispX(x-width/2), yToDispY(y-length/2), width*xzoom, length*yzoom};						//x-width/2 is used, so that the x and y coordinates refer to the center of the tank
	SDL_RenderCopyEx( gRenderer, body, &bodyclip, &temp, tankrot+90, NULL, SDL_FLIP_NONE);

	temp = {xToDispX(x-turretradius), yToDispY(y-turretradius), turretradius*2*xzoom, turretradius*2*yzoom};
	SDL_RenderCopyEx( gRenderer, turret, &turretclip, &temp, tankrot+turretrot+90, NULL, SDL_FLIP_NONE);

	SDL_Point center = {barrelwidth*xzoom/2, 0-turretradius*yzoom};
	temp = {xToDispX(x-barrelwidth/2), yToDispY(y+turretradius), barrelwidth*xzoom, barrellength*yzoom};
	SDL_RenderCopyEx( gRenderer, barrel, &barrelclip, &temp, tankrot+turretrot+90, &center, SDL_FLIP_NONE);
}

uint32_t Tank::takeDamage(double damage, bool shieldpass){
    if(index!=0 && !dead){
		boostedViewDistance=viewDistance*1000;
		if(heatnum<(255-500*stepSize))
			heatnum+=500*stepSize;
		else
			heatnum=255;
    }
	if(!shieldpass){
            shieldCount=0;
        if(shield>damage)
            shield-=damage;
        else{
            damage-=shield;
            shield=0;
            if(health>damage)
                health-=damage;
            else{
                if(dead){
//					health=0;
					return 0;
                }else if(index==0){
					character.materials=0;
                }
				primaryReload=0;
				secondaryReload=0;
                energy=0;
                dead=true;
                if(index!=0)
					health=0;
				else if(character.repairs>0){
					health=0;
					character.repairs--;
				}else{
					health=-1;
				}
                return expgiven;
            }    //call death animation
        }
    }else{
        if(health>damage)
            health-=damage;
        else{
            if(dead){
				health=0;
				shield=0;
				return 0;
			}else if(index==0){
				character.materials=0;
			}
			primaryReload=0;
			secondaryReload=0;\
			shieldCount=0;
			energy=0;
            dead=true;
            if(index!=0)
				health=0;
			else if(character.repairs>0){
				health=0;
				character.repairs--;
			}else{
				health=-1;
			}
			shield=0;
            return expgiven;
        }    //call death animation
    }
    return 0;
}

void Tank::load(std::ifstream* file){
	std::string line;
	getline(*file,line);
	x=atof(line.c_str());
	getline(*file,line);
	y=atof(line.c_str());
	getline(*file,line);
	stuckness=atoi(line.c_str());
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
	if(line.compare("t")==0)
		stripped=true;
	else
		stripped=false;
	getline(*file,line);
	index=atoi(line.c_str());
	getline(*file,line);
	currentRoom=atoi(line.c_str());
}

void Tank::save(std::ofstream* file){
    std::ostringstream strs;
    strs << x << "\n";
    strs << y << "\n";
    strs << stuckness << "\n";
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
	if(stripped)
		strs << "t\n";
	else
		strs << "f\n";
    strs << index << "\n";
    strs << currentRoom << "\n";
    *file << strs.str();
}

void Tank::loadfromcharacter(){
	double mul=1;
	for(int i=0;i<character.brakeup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	brakeforce=character.brake*mul;
	mul=1;
	for(int i=0;i<character.delayup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	shieldDelay=character.delay/mul;
	mul=1;
	for(int i=0;i<character.energyup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	maxEnergy=character.energy*mul;
	mul=1;
	for(int i=0;i<character.generationup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	energyGen=character.generation*mul;
	mul=1;
	for(int i=0;i<character.healthup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	maxHealth=character.health*mul;
	mul=1;
	for(int i=0;i<character.massdown;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	mass=character.mass/mul;
	mul=1;
	for(int i=0;i<character.maxVelocityup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	maxvel=character.maxVelocity*mul;
	name=character.name;
	mul=1;
	for(int i=0;i<character.powerup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	power=character.power*mul;
	mul=1;
	for(int i=0;i<character.regenup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	shieldRegen=character.regen*mul;
	mul=1;
	for(int i=0;i<character.shieldup;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	maxShield=character.shield*mul;
	//Accuracy(spread), Burst(int), damage, energy, Id(int), Name, Passesshield(flag), 1primary(flag), Rof(int), Gspeed, Another(start next one)
	Weapon primary;
	Weapon secondary;
	for(int i=0;i<weapons.size();i++){
		if(weapons[i].ID==character.primaryID)
			primary=weapons[i];
		if(weapons[i].ID==character.secondaryID)
			secondary=weapons[i];
	}
	WeaponUpgrades prim;
	WeaponUpgrades second;
	for(int i=0;i<character.weaponUpgrades.size();i++){
		if(character.weaponUpgrades[i].ID==character.primaryID)
			prim=character.weaponUpgrades[i];
		if(character.weaponUpgrades[i].ID==character.secondaryID)
			second=character.weaponUpgrades[i];
	}
	mul=1;
	for(int i=0;i<prim.range;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	primaryRange=primary.range*mul;
	mul=1;
	for(int i=0;i<second.range;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	secondaryRange=secondary.range*mul;
	mul=1;
	mul=1;
	for(int i=0;i<prim.spread;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	pspread=primary.spread/mul;
	mul=1;
	for(int i=0;i<second.spread;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	sspread=secondary.spread/mul;
	mul=1;
	for(int i=0;i<prim.burst;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	pburst=primary.burst*floor(mul);
	mul=1;
	for(int i=0;i<second.burst;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	sburst=secondary.burst*floor(mul);
	mul=1;
	for(int i=0;i<prim.damage;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	primaryDamage=primary.damage*mul;
	mul=1;
	for(int i=0;i<second.damage;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	secondaryDamage=secondary.damage*mul;
	mul=1;
	for(int i=0;i<prim.energy;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	pEnergy=primary.energy/mul;
	mul=1;
	for(int i=0;i<second.energy;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	sEnergy=secondary.energy/mul;
	pPassesShield=primary.passesshields;
	sPassesShield=secondary.passesshields;
	mul=1;
	for(int i=0;i<prim.ROF;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	primaryROF=primary.ROF/mul;
	mul=1;
	for(int i=0;i<second.ROF;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	secondaryROF=secondary.ROF/mul;
	mul=1;
	for(int i=0;i<prim.speed;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	primaryProjectileSpeed=primary.speed*mul;
	mul=1;
	for(int i=0;i<second.speed;i++){
		mul*=pow(firstup, pow(deterioration, (i-1)));
	}
	secondaryProjectileSpeed=secondary.speed*mul;
	health=maxHealth;
	energy=maxEnergy;
	shield=maxShield;
}

void Tank::broadcast(){
	for(int i=1;i<tanks.size();i++){
		if(i!=index && sqrt(pow(tanks[i]->getX()-x, 2)+pow(tanks[i]->getY()-y, 2))<broadcastDistance){
			tanks[i]->pointTo(targetX, targetY);
			tanks[i]->resetTimeSinceSeen();
		}
	}
}


//{those methods

void Tank::resetTimeSinceSeen(){
	timeSinceSeen=0;
    patrolling=false;
}
void Tank::incrementTimeSinceSeen(double increment){
	timeSinceSeen+=increment;
}

double Tank::getShieldTimer(){
	return shieldDelay-shieldCount;
}

double Tank::getShieldDelay(){
	return shieldDelay;
}

void Tank::boostView(){
	boostedViewDistance=1000*viewDistance;
}

double Tank::getViewDistance(bool reset){
	double tmp=boostedViewDistance;
	if(reset)
		boostedViewDistance=viewDistance;
	return tmp;
}

bool Tank::isStripped(){
	return stripped;
}

void Tank::strip(){
	stripped=true;
}

uint32_t Tank::getmaterials(){
	return materialsgiven;
}

uint32_t Tank::getexp(){
	return expgiven;
}

int Tank::getCurrentRoom(){
	return currentRoom;
}

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

void Tank::getTarget(double** x, double** y){
	*x=&targetX;
	*y=&targetY;
}

bool Tank::hasseentarget(){
	if(tanks[0]->isDead() || timeSinceSeen>=chaseTime){
		//targetX=-1000;
		//targetY=-1000;
		timeSinceSeen=chaseTime;
		patrolling=true;
		return false;
	}
    patrolling = !(targetX!=-1000 || targetY!=-1000);
    return !patrolling;
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
		/*shape.isCircle=true;
		shape.x0=x;
		shape.y0=y;
		shape.radius=length/2.0;*/
    }
    sortpoints(&xvec, &yvec);
    temp.x = xvec;
    temp.y = yvec;
    shape.room=currentRoom;
    return temp;
}

int Tank::getPburst(){
    return pburst;
}

int Tank::getSburst(){
    return sburst;
}

double Tank::getX(){
//	std::cout << "X " << x << "\n";
	return x;
}

double Tank::getY(){
//	std::cout << "Y " << y << "\n";
	return y;
}

double Tank::getOldX(){
	return oldx;
}

double Tank::getOldY(){
	return oldy;
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
    return primaryReload >= primaryROF;
}

bool Tank::secondaryReloaded(){
    return secondaryReload >= secondaryROF;
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
        character = loadCharacter("data/game/defaultcharacter.txt");
        weapons = loadWeapons("data/game/weapons.txt");
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
	std::cout << "RUN GAME\n";
	saved=0;
	TICKS_PER_FRAME = 1000/MAX_FPS;
	heatnum=0;
	uint32_t frameTimer;
	bool quit = false;
	SDL_Event e;
	std::vector<uint32_t> frames;
    for(double i=0; i<1000; i+=1000/MAX_FPS){
        frames.push_back(SDL_GetTicks()-1000+i);
    }
	lastpause = 0;
	pauseduration = 0;
	recentpause = false;
	slomo=100;
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
                }else if(line.compare("charactersave-")==0){
					character=loadCharacter(&loadFile);
				}else if(line.compare("status")==0){
					loadStatus(&loadFile);
				}
                std::cout << line << '\n';
            }
        }else
            std::cout << "Unable to open file\n";
    }
    if(character.name!=""){
		reloadCharacter();
	}
	//set camera before game runs to prevent a "jump"
    mouseXfreeze=mouseX;//only mouseX is kept up to date whilst outside of game loop
    mouseYfreeze=mouseY;
    camX=((tanks[0]->getX()+mouseXfreeze)/2)-SCREEN_WIDTH/2.0;
    camY=((tanks[0]->getY()+mouseYfreeze)/2)-SCREEN_HEIGHT/2.0;
    if(camX<-100)
        camX=-100;
    else if (camX>getRoomDimension(tanks[0]->getCurrentRoom()).mapwidth+100-SCREEN_WIDTH)
        camX=getRoomDimension(tanks[0]->getCurrentRoom()).mapwidth+100-SCREEN_WIDTH;
    if(camY<-100)
        camY=-100;
    else if (camY>getRoomDimension(tanks[0]->getCurrentRoom()).mapheight+100-SCREEN_HEIGHT)
        camY=getRoomDimension(tanks[0]->getCurrentRoom()).mapheight+100-SCREEN_HEIGHT;
	std::cout << "REACHED GAME LOOP\n";
	while( !quit ){
		auto t1all=Clock::now();
        frameTimer = SDL_GetTicks();
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
        if(freeze)
            stepSize=0;
        int tanknum = -1;
        quit = updateMouse(true);
        auto t1physics = Clock::now();
        for(int phys = 0; phys < physics; phys++){
            //{Handle input
            handleInput(tanks[0]);
            if(!freeze&&!tanks[0]->isDead()){//unfreeze mouse variable, mousefreeze is always up to date, mouseX is only updated whilst game is running
                mouseX=mouseXfreeze;
                mouseY=mouseYfreeze;
            }
            if(pause){//pause menu
                pause = false;
                pauseduration = SDL_GetTicks();
                std::string* saveto = new std::string("");
                switch (pauseMenu(saveto)){//pause menu returns 0 if returning to main menu, 1 if saving, otherwise continue
                    case 0://return to main menu
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
                    case 1://save game
						save(saveto);
                        pause=true;
                        //implement methods which write a given struct to a given file, then write multiple loops that write out all vectors, then write methods which allow reading from that file
                        //when reading, reads values in order until next element flag is read, repeat until next data type flag is read.
                        //DONE
                    break;
                }
                lastpause = SDL_GetTicks();
                pauseduration = lastpause - pauseduration;
                recentpause = true;
                delete saveto;
            }
            if(menu){//game menu
				menu=false;
				pauseduration = SDL_GetTicks();
				gameMenu();
				lastpause = SDL_GetTicks();
                pauseduration = lastpause - pauseduration;
                recentpause = true;
            }

            if(!freeze){
                tanks[0]->pointTo( mouseX*1.0, mouseY*1.0 );
				tanks[0]->step();
				if(phys!=physics-1){
					for(int i=1;i<tanks.size();i++){
						tanks[i]->step();//crash triggered from here
					}
				}
            }
            //}
            for(int i=1;i<tanks.size();i++){
                if(isInShape(mouseX, mouseY, tanks[i]->getShape())&&!tanks[i]->isDead()){
                    tanknum = i;
                }
            }
			//step projectiles
			stepSize=3*stepSize/physics;
			for(int i = 0; i < projectiles.size(); i++){
				bool remove = stepProjectile(projectiles[i]);
				if(remove){
					delete projectiles[i];
					projectiles.erase(projectiles.begin()+i);
				}
			}
			stepSize=physics*stepSize/3;
        }
        auto t2physics = Clock::now();
		if(!freeze){
	        heat=false;
	        processingAvailable=true;
            for(int i=1; i<tanks.size(); i++){//enemy AI
				if(!tanks[i]->isDead()){
				auto t1 = Clock::now();
                double targx = tanks[0]->getX();
                double targy = tanks[0]->getY();
                if(cansee(i, 0, &targx, &targy)){
					tanks[i]->resetTimeSinceSeen();
					heat=true;
					if(heatnum<(255-50*stepSize))
						heatnum+=50*stepSize;
					else
						heatnum=255;
                    tanks[i]->settargetvel(tanks[0]->getVel());
                    tanks[i]->pointTo( targx, targy );
                    tanks[i]->broadcast();
                    tanks[i]->step();
                    if(sqrt(pow((targx-tanks[i]->getX()), 2)+pow((targy-tanks[i]->getY()), 2))>100){//only accelerate if more than 100 from target
                        tanks[i]->accelerate(100);
                        tanks[i]->brake(0);
                    }else{
                        tanks[i]->accelerate(0);
                        tanks[i]->brake(100);
                    }
                    fire(true, true, tanks[i], &projectiles);
                } else {//can't see player
					tanks[i]->incrementTimeSinceSeen(stepSize);
					tanks[i]->step();
                    tanks[i]->settargetvel(0.0);
                    double* x;
                    double* y;
                    tanks[i]->getTarget(&x, &y);//adjust target so enemy will follow through portals
                    if(*x<=0){
						*x=-200;
                    }else if(*x>=getRoomDimension(tanks[i]->getCurrentRoom()).mapwidth){
						*x=getRoomDimension(tanks[i]->getCurrentRoom()).mapwidth+200;
                    }
                    if(*y<=0){
						*y=-200;
                    }else if(*y>=getRoomDimension(tanks[i]->getCurrentRoom()).mapheight){
						*y=getRoomDimension(tanks[i]->getCurrentRoom()).mapheight+200;
                    }
                    if(tanks[i]->hasseentarget()){// && !tanks[0]->isDead()){
						tanks[i]->accelerate(100);
                        tanks[i]->brake(0);
                    } else {
						//return to patrol and follow it
						tanks[i]->patrol();
                        //tanks[i]->accelerate(0);
                        //tanks[i]->brake(100);
                    }
                }
                    if(timing){
						auto t2 = Clock::now();
						//std::cout << "AI execution time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " nanoseconds" << std::endl;
						AIruntimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count());
						if(AIruntimes.size()>100)
							AIruntimes.erase(AIruntimes.begin()+0);
					}
                }
            }
			if(!heat&&heatnum>0){
				if(heatnum<50*stepSize){
					if(savelevel>=1 && (savetime==0 || (SDL_GetTicks()+500)<savetime)){
						savetime=SDL_GetTicks()+5000;
					}
					heatnum=0;
				}else{
					savetime=0;
					heatnum-=50*stepSize;
				}
			}
			if(tanks[0]->getVel()==0 && heatnum==0){
				if(savelevel>=3 && frameCount%50==0 && (savetime==0 || (SDL_GetTicks()+500)<savetime)){
					savetime=SDL_GetTicks()+500;
				}
			}
		}
		//saving
		if(savetime!=0 && SDL_GetTicks()>=savetime && heatnum==0 && !tanks[0]->isDead()){
			std::string saveto = "data/save/save1.game";
			std::cout << "AUTOSAVE\n";
			save(&saveto);
			savetime=0;
		}
        //update mouse coordinates
        SDL_GetMouseState( &mouseXfreeze, &mouseYfreeze);
        //mouseXfreeze+=camX;
        //mouseYfreeze+=camY;
        //mouseXfreeze=camX+(mouseXfreeze)/xzoom;
        //mouseYfreeze=camY+(mouseYfreeze)/yzoom;
		mouseXfreeze=(mouseXfreeze-SCREEN_WIDTH/2)/xzoom+camX;//convert screen coordinates to game coordinates
		mouseYfreeze=(mouseYfreeze-SCREEN_HEIGHT/2)/yzoom+camY;
        //separate mousecoordinates so aiming laser doesn't move while frozen
        //{Set camera coordinates
        camX=((tanks[0]->getX()+mouseXfreeze)/2);//-SCREEN_WIDTH/(2.0*xzoom);//cam is the coordinates at the centre of the screen
        camY=((tanks[0]->getY()+mouseYfreeze)/2);
        if(camX-SCREEN_WIDTH/(2.0*xzoom)<-100)
            camX=-100+SCREEN_WIDTH/(2.0*xzoom);
        else if (camX+SCREEN_WIDTH/(2.0*xzoom)>getRoomDimension(tanks[0]->getCurrentRoom()).mapwidth+100)
            camX=getRoomDimension(tanks[0]->getCurrentRoom()).mapwidth+100-SCREEN_WIDTH/(2.0*xzoom);
        if(camY-SCREEN_HEIGHT/(2.0*yzoom)<-100)
            camY=-100+SCREEN_HEIGHT/(2.0*yzoom);
        else if (camY+SCREEN_HEIGHT/(2.0*yzoom)>getRoomDimension(tanks[0]->getCurrentRoom()).mapheight+100)
            camY=getRoomDimension(tanks[0]->getCurrentRoom()).mapheight+100-SCREEN_HEIGHT/(2.0*yzoom);
		if(camX+SCREEN_WIDTH/(2.0*xzoom)>getRoomDimension(tanks[0]->getCurrentRoom()).mapwidth+100)
			camX=getRoomDimension(tanks[0]->getCurrentRoom()).mapwidth/2.0;
		if(camY+SCREEN_HEIGHT/(2.0*yzoom)>getRoomDimension(tanks[0]->getCurrentRoom()).mapheight+100)
			camY=getRoomDimension(tanks[0]->getCurrentRoom()).mapheight/2.0;
		//}
        auto t1render=Clock::now();
        //render everything
        //aiming laser
        int tanknum2=-1;
        if(aiminglaser && !tanks[0]->isDead()){
            double xcomp = (mouseX-tanks[0]->getX())*10/(sqrt(pow(mouseX-tanks[0]->getX(), 2)+pow(mouseY-tanks[0]->getY(), 2)));//xcomponent of increment
            double ycomp = (mouseY-tanks[0]->getY())*10/(sqrt(pow(mouseX-tanks[0]->getX(), 2)+pow(mouseY-tanks[0]->getY(), 2)));
            double x2 = tanks[0]->getX();
            double y2 = tanks[0]->getY();
            bool ret = false;
            while(x2<camX+SCREEN_WIDTH/(2.0*xzoom) && y2<camY+SCREEN_HEIGHT/(2.0*yzoom) && x2>camX-SCREEN_WIDTH/(2.0*xzoom) && y2>camY-SCREEN_HEIGHT/(2.0*yzoom) && ret==false){
				for(int i=1; i<tanks.size(); i++){
					if(tanks[i]->getCurrentRoom()==tanks[0]->getCurrentRoom()){
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
				}
				for(int i=0; i<shapes.size(); i++){
					if(shapes[i]->room==tanks[0]->getCurrentRoom()){
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
        if(shapecollide==i)
			SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, 0xFF );
		else
			SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
		if(shapes[i]->room==tanks[0]->getCurrentRoom()){
            SCROLL_RenderDrawLine( gRenderer, shapes[i]->x[shapes[i]->x.size()-1], shapes[i]->y[shapes[i]->x.size()-1], shapes[i]->x[0], shapes[i]->y[0] );
            for(int j=0;j<shapes[i]->x.size()-1;j++)
                SCROLL_RenderDrawLine( gRenderer, shapes[i]->x[j], shapes[i]->y[j], shapes[i]->x[j+1], shapes[i]->y[j+1] );
        }}
        //draw portals
        SDL_SetRenderDrawColor( gRenderer, 0x00, 0xFF, 0x00, 0xFF );
        for(int i=0;i<portals.size();i++){
        if(portals[i].room==tanks[0]->getCurrentRoom()){
			int tx = portals[i].x;
			int ty = portals[i].y;
			int tx2 = tx+portals[i].w;
			int ty2 = ty+portals[i].h;
			SCROLL_RenderDrawLine( gRenderer, tx, ty, tx2, ty);
			SCROLL_RenderDrawLine( gRenderer, tx2, ty, tx2, ty2);
			SCROLL_RenderDrawLine( gRenderer, tx2, ty2, tx, ty2);
			SCROLL_RenderDrawLine( gRenderer, tx, ty2, tx, ty);
        }}
        if(particleEffects){
        for(int i = 0; i < particlelines.size(); i++){
			if(particlelines[i].room==tanks[0]->getCurrentRoom()){
				particlelines[i];
				SDL_SetRenderDrawColor( gRenderer, particlelines[i].r, particlelines[i].g, particlelines[i].b, particlelines[i].a);
				SCROLL_RenderDrawLine( gRenderer, particlelines[i].x1, particlelines[i].y1, particlelines[i].x2, particlelines[i].y2);
			}
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
            if(tanks[i]->getCurrentRoom()==tanks[0]->getCurrentRoom())
				tanks[i]->render();
		}
		//render headsup
		//health and shield
		SDL_Rect healthRect = { 25, 25, 300, 15 };//grey back
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
		SDL_RenderFillRect( gRenderer, &healthRect );
		if(tanks[0]->getHealth()>=0){
			healthRect = { 25, 27, 300.0*(tanks[0]->getHealth()/tanks[0]->getMaxHealth()), 6 };//health
			SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xAF, 0xAF );
			SDL_RenderFillRect( gRenderer, &healthRect );
		}
		healthRect = { 25, 20, 300.0*(1-tanks[0]->getShieldTimer()/tanks[0]->getShieldDelay()), 5 };//shieldDelay
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xFF, 0x6F );
		SDL_RenderFillRect( gRenderer, &healthRect );
        healthRect = { 25, 25, 300.0*(tanks[0]->getShield()/tanks[0]->getMaxShield()), 10 };//shield
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xFF, 0x6F );
		SDL_RenderFillRect( gRenderer, &healthRect );
		//exp
		healthRect = { 25, 35, 300.0*((character.exp%1000)/1000.0), 5};//exp
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0xFF, 0x00, 0xAF );
		SDL_RenderFillRect( gRenderer, &healthRect );
		//heat
		if(heatnum>0){
			healthRect = { 0, 0, SCREEN_WIDTH, 5 };
			SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, heatnum);
			SDL_RenderFillRect( gRenderer, &healthRect );
			healthRect = { 0, SCREEN_HEIGHT-5, SCREEN_WIDTH, 5 };
			SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, heatnum);
			SDL_RenderFillRect( gRenderer, &healthRect );
			healthRect = { 0, 5, 5, SCREEN_HEIGHT-10 };
			SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, heatnum);
			SDL_RenderFillRect( gRenderer, &healthRect );
			healthRect = { SCREEN_WIDTH-5, 5, 5, SCREEN_HEIGHT-10 };
			SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, heatnum);
			SDL_RenderFillRect( gRenderer, &healthRect );
		}
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
		healthRect = { 350, 35, 100, 10 };//grey back
		SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
		SDL_RenderFillRect( gRenderer, &healthRect );
        healthRect = { 350, 35, 100.0*(tanks[0]->getSecondaryReload()/tanks[0]->getSecondaryROF()), 10 };//primary
		SDL_SetRenderDrawColor( gRenderer, 0xFF, 0x00, 0x00, 0xAF );
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
        if(tanks[i]->getCurrentRoom()==tanks[0]->getCurrentRoom()){
            SDL_Rect healthRect = { xToDispX(tanks[i]->getX())-50*xzoom, yToDispY(tanks[i]->getY())-50*yzoom, 100*xzoom, 5*yzoom };//grey back
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0x1F );
            SDL_RenderFillRect( gRenderer, &healthRect );
            healthRect = { xToDispX(tanks[i]->getX())-50*xzoom, yToDispY(tanks[i]->getY())-49*yzoom, 100.0*(tanks[i]->getHealth()/tanks[i]->getMaxHealth())*xzoom, 3*yzoom };//health
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xAF, 0xAF );
            SDL_RenderFillRect( gRenderer, &healthRect );
            healthRect = { xToDispX(tanks[i]->getX())-50*xzoom, yToDispY(tanks[i]->getY())-50*yzoom, 100.0*(tanks[i]->getShield()/tanks[i]->getMaxShield())*xzoom, 5*yzoom };//shield
            SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0xFF, 0x6F );
            SDL_RenderFillRect( gRenderer, &healthRect );
        }}
		//show framerate
        std::ostringstream strs;
        strs << FPS;
        std::string text = strs.str();
        SDL_Color textColour = {0, 0, 0};
        SDL_Surface* textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
        SDL_Texture* mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
        SDL_Rect temp = {5, 5, textSurface->w/2, textSurface->h/2};
        SDL_FreeSurface( textSurface );
        SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
        SDL_DestroyTexture(mTexture);
        //show velocity
        strs.str("");
        strs << (round(tanks[0]->getVel()*100.0))/100.0;
		text = strs.str();
        textColour = {0, 0, 0};
        textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
        mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
        temp = {30, 5, textSurface->w/2, textSurface->h/2};
        SDL_FreeSurface( textSurface );
        SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
        SDL_DestroyTexture(mTexture);
        //show level
        strs.str("");
        strs << floor(character.exp/1000.0);
		text = strs.str();
        textColour = {0, 0, 0};
        textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
        mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
        temp = {5, 25, textSurface->w/2, textSurface->h/2};
        SDL_FreeSurface( textSurface );
        SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
        SDL_DestroyTexture(mTexture);
        //show materials
        strs.str("");
        strs << character.materials << " / " << character.safematerials;
		text = strs.str();
        textColour = {0, 0, 0};
        textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
        mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
        temp = {5, 45, textSurface->w/2, textSurface->h/2};
        SDL_FreeSurface( textSurface );
        SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
        SDL_DestroyTexture(mTexture);
        //show repairs
        strs.str("");
        strs << character.repairs << " / " << maxrepairs;
		text = strs.str();
        textColour = {0, 0, 0};
        textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
        mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
        temp = {5, 60, textSurface->w/2, textSurface->h/2};
        SDL_FreeSurface( textSurface );
        SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
        SDL_DestroyTexture(mTexture);
		//show saved
		if(saved>0){
			saved--;
			strs.str("");
			strs << "Game Saved!";
			text = strs.str();
			textColour = {0, 0, 0};
			textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
			mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
			temp = {1190, 940, textSurface->w/2, textSurface->h/2};
			SDL_FreeSurface( textSurface );
			SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
			SDL_DestroyTexture(mTexture);
		}
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
                if(isInShape(mouseXfreeze, mouseYfreeze, tanks[i]->getShape()) && tanks[i]->getCurrentRoom()==tanks[0]->getCurrentRoom()){//display info about enememy mouse is over while frozren
                    if(LmouseClick){//set tank to show advanced details for
                        showDetailsFor=tanks[i];
                    }else if(showDetailsFor==NULL){//if not showing advanced details show quick details
                        if(tanks[i]->isDead()){
                            //say tank is dead + name
                            if(tanks[i]->isStripped()){
								basicText("STRIPPED "+tanks[i]->getName(), {0, 0, 0}, xToDispX(tanks[i]->getX()), yToDispY(tanks[i]->getY())+50*yzoom, 0.5, 0.5, true, 255, 255, 255, 127);
                            }else{
								basicText("DEAD "+tanks[i]->getName(), {0, 0, 0}, xToDispX(tanks[i]->getX()), yToDispY(tanks[i]->getY())+50*yzoom, 0.5, 0.5, true, 255, 255, 255, 127);
							}
                        }else{
                            //show some stats
                            basicText(tanks[i]->getName(), {0, 0, 0}, xToDispX(tanks[i]->getX()), yToDispY(tanks[i]->getY())+50*yzoom, 0.5, 0.5, true, 255, 255, 255, 127);
                            std::stringstream stream;
                            stream << round(tanks[i]->getHealth())<<"/"<<round(tanks[i]->getMaxHealth());
                            basicText(stream.str(), {0, 0, 175}, xToDispX(tanks[i]->getX()), yToDispY(tanks[i]->getY())+50*yzoom+fontSize/2, 0.5, 0.5, true, 255, 255, 255, 127);
                            stream.str("");
                            stream << round(tanks[i]->getShield())<<"/"<<round(tanks[i]->getMaxShield());
                            basicText(stream.str(), {0, 0, 255}, xToDispX(tanks[i]->getX()), yToDispY(tanks[i]->getY())+50*yzoom+fontSize, 0.5, 0.5, true, 255, 255, 255, 127);
                            stream.str("");
                            stream << round(10000*(tanks[i]->getSburst()*tanks[i]->getSecondaryDamage()/tanks[i]->getSecondaryROF()+tanks[i]->getPburst()*tanks[i]->getPrimaryDamage()/tanks[i]->getPrimaryROF()))/100;
                            basicText(stream.str(), {255, 0, 0}, xToDispX(tanks[i]->getX()), yToDispY(tanks[i]->getY())+50*yzoom+fontSize*1.5, 0.5, 0.5, true, 255, 255, 255, 127);
                            stream.str("");
                            stream << tanks[i]->getVel();
							basicText(stream.str(), {0, 0, 0}, xToDispX(tanks[i]->getX()), yToDispY(tanks[i]->getY())+50*yzoom+fontSize*2, 0.5, 0.5, true, 255, 255, 255, 127);
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
		if(timing && !freeze){
			auto t2all = Clock::now();
			double frametime=std::chrono::duration_cast<std::chrono::nanoseconds>(t2all - t1all).count();
			frametime/=1000000;
			std::cout << "Frame time: " << frametime << " milliseconds out of " << TICKS_PER_FRAME << " allowed by framerate" << std::endl;
			frametime=std::chrono::duration_cast<std::chrono::nanoseconds>(t2all - t1render).count();
			frametime/=1000000;
			std::cout << "Time to render: " << frametime << " milliseconds\n";
			frametime=std::chrono::duration_cast<std::chrono::nanoseconds>(t2physics - t1physics).count();
			frametime/=1000000;
			std::cout << "Time to do physics: " << frametime << " milliseconds\n";
			double average=0;
			for(int j=0;j<AIruntimes.size();j++)
				average+=AIruntimes[j];
			average/=AIruntimes.size()*1000;
			std::cout << "Average AI execution time: " << average << " microseconds\n";
			average=0;
			for(int j=0;j<otherruntimes.size();j++)
				average+=otherruntimes[j];
			average/=otherruntimes.size()*1000;
			std::cout << "Other execution time: " << average << " microseconds. (size: " << otherruntimes.size() << ")\n";
			average=0;
			for(int j=0;j<stepruntimes.size();j++)
				average+=stepruntimes[j];
			average/=stepruntimes.size()*1000;
			std::cout << "Step execution time: " << average << " microseconds\n";
		}
        //stay under framrate cap
		if(SDL_GetTicks()-frameTimer < TICKS_PER_FRAME)
            SDL_Delay(TICKS_PER_FRAME-(SDL_GetTicks()-frameTimer));

		/*if(LmouseState){//Frame step - hack FPS to use properly
			updateMouse();
			while(!RmouseState){
				updateMouse();
			}
			while(RmouseState){
				updateMouse();
			}
		}*/
        //console log
//        std::cout << "FPS: " << frames.size() << "\n";
//			std::cout << "STEPSIZE: " << stepSize << "\n";

        frameCount++;
//        std::cout << "Frame count: " << frameCount << "\n";
//        std::cout << "Particles: " << particlelines.size() << "\n";

	}
	return 0;
}

void save(std::string* saveto){
	std::ofstream savefile (*saveto);
	if (savefile.is_open()){
		saved=128;
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
                            savefile<<"charactersave-\n";
                            saveCharacter(character, &savefile);
                            savefile<<"status\n";
                            saveStatus(&savefile);
                            /*for(int i=0;i<shapes.size();i++){
                                savefile<<"newshape\n";
                                saveShape(*shapes.at(i), &savefile);
                            }*///Dont save shapes or portals, it's a static map
                        }else
                            std::cout << "Unable to open file\n";
}

//{menus

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
        if(LmouseClick){
			while(LmouseState){//wait for mouse to be released before continuing, prevents firing projectile as soon as game starts.
				if(updateMouse())
					return 0;
			}
			SDL_GetMouseState( &mouseX, &mouseY );
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
                            weapons = loadWeapons("data/game/weapons.txt");
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
	character = loadCharacter("data/game/defaultcharacter.txt");
	weapons = loadWeapons("data/game/weapons.txt");
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
    std::string text = "Resume Game";
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

	text = "Return to Main Menu";
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
        if(updateMouse()){//updateMouse returns true if quit signal is sent
            delete buttons.at(0);
			delete buttons.at(1);
            delete buttons.at(2);
            delete buttons.at(3);
            return 0;
		}
        if(LmouseClick){
			while(LmouseState){//wait for mouse to be released before continuing, prevents firing projectile as soon as game starts.
				if(updateMouse()){
					delete buttons.at(0);
                    delete buttons.at(1);
                    delete buttons.at(2);
                    delete buttons.at(3);
					return 0;
				}
			}
			SDL_GetMouseState( &mouseX, &mouseY );
            for(int i=0; i<buttons.size();i++){
                if(isInShape(mouseX, mouseY, *buttons[i])){
                    switch (i){
                        case 0:
                            delete buttons.at(0);
                            delete buttons.at(1);
                            delete buttons.at(2);
                            delete buttons.at(3);
                            return 2;
                        break;
                        case 1:
                            saveMenu(data);
                            if(*data!=""){
								delete buttons.at(0);
								delete buttons.at(1);
								delete buttons.at(2);
								delete buttons.at(3);
								return 1;
                            }
                        break;
                        case 2:
                        break;
                        case 3:
                            delete buttons.at(0);
                            delete buttons.at(1);
                            delete buttons.at(2);
                            delete buttons.at(3);
                            return 0;
                        break;
                    }
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

uint8_t gameMenu(){//upgrade menu, show base values, current, and with one more upgrade.
	std::cout << "Game Menu\n";
	Character tempcharacter =  character;
	bool loop = true;
    while(loop){
        SDL_PumpEvents();
        if(!keys[menuKey])
            loop = false;
    }
    bool esckeystate = false;
    bool menukeystate = false;
  while (true){
	loop = true;
	//{render screen
	SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	SDL_RenderClear( gRenderer );
    SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
    std::vector<Shape*> menubuttons;
    //add menu buttons
	addButton(&menubuttons, 1*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	addButton(&menubuttons, 11.0/3.0*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	addButton(&menubuttons, 5*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	addButton(&menubuttons, 7.0/3.0*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	verybasicText("Reset - 100m", 1*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	verybasicText("Undo Changes", 11.0/3.0*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	verybasicText("Apply Changes", 5*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	verybasicText("Upgrade Weapons", 7.0/3.0*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
    //render menu buttons
    for(int i=0;i<menubuttons.size();i++){
        SDL_RenderDrawLine( gRenderer, menubuttons[i]->x[menubuttons[i]->x.size()-1], menubuttons[i]->y[menubuttons[i]->x.size()-1], menubuttons[i]->x[0], menubuttons[i]->y[0] );
        for(int j=0;j<menubuttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, menubuttons[i]->x[j], menubuttons[i]->y[j], menubuttons[i]->x[j+1], menubuttons[i]->y[j+1] );
    }
    verybasicText("Upgrade Menu", SCREEN_WIDTH/2, SCREEN_HEIGHT/8, 1);
	std::vector<std::string> upgradenames = {"Power:","Brakes:", "Velocity:", "Mass Reduction:", "Energy:", "Energy Generation:", "Health:", "Shield:", "Shield Regen:", "Shield Delay:", "Max Slomo:", "Slomo Regen:", "Slomo Usage", "Slomo Response:", "Max Repairs"};
	double valuearray[15][2];
	std::vector<Shape*> upgradebuttons;
	valuearray[0][0]=tempcharacter.power;
	valuearray[0][1]=tempcharacter.powerup;
	valuearray[1][0]=tempcharacter.brake;
	valuearray[1][1]=tempcharacter.brakeup;
	valuearray[2][0]=tempcharacter.maxVelocity;
	valuearray[2][1]=tempcharacter.maxVelocityup;
	valuearray[3][0]=tempcharacter.mass;
	valuearray[3][1]=tempcharacter.massdown;
	valuearray[4][0]=tempcharacter.energy;
	valuearray[4][1]=tempcharacter.energyup;
	valuearray[5][0]=tempcharacter.generation;
	valuearray[5][1]=tempcharacter.generationup;
	valuearray[6][0]=tempcharacter.health;
	valuearray[6][1]=tempcharacter.healthup;
	valuearray[7][0]=tempcharacter.shield;
	valuearray[7][1]=tempcharacter.shieldup;
	valuearray[8][0]=tempcharacter.regen;
	valuearray[8][1]=tempcharacter.regenup;
	valuearray[9][0]=tempcharacter.delay;
	valuearray[9][1]=tempcharacter.delayup;
	valuearray[10][0]=tempcharacter.slomomax;
	valuearray[10][1]=tempcharacter.slomomaxup;
	valuearray[11][0]=tempcharacter.slomoregen;
	valuearray[11][1]=tempcharacter.slomoregenup;
	valuearray[12][0]=tempcharacter.slomousage;
	valuearray[12][1]=tempcharacter.slomousageup;
	valuearray[13][0]=tempcharacter.slomoresponse;
	valuearray[13][1]=tempcharacter.slomoresponseup;
	valuearray[14][0]=tempcharacter.maxrepairs;
	valuearray[14][1]=tempcharacter.maxrepairsup;
	std::ostringstream strs;
	strs << "Materials Remaining: " << tempcharacter.safematerials;
	verybasicText(strs.str(), 2*SCREEN_WIDTH/6, 5*SCREEN_HEIGHT/8, 0.7);
	strs.str("");
	strs << "Upgrade Points: " << tempcharacter.upgradepoints;
	verybasicText(strs.str(), 4*SCREEN_WIDTH/6, 5*SCREEN_HEIGHT/8, 0.7);
	strs.str("");
	for(int i=0;i<15;i++){
		double height=SCREEN_HEIGHT/8+50+i*20;
		double current;
		double next;
		double mul=1;
		for(int j=0;j<valuearray[i][1];j++){
		mul*=pow(firstup, pow(deterioration, (j-1)));
		}
		if(i==3 || i==9 || i==12){//these values decrease when upgraded
			current=valuearray[i][0]/mul;
			next=valuearray[i][0]/(mul*pow(firstup, pow(deterioration, (valuearray[i][1]-1))));
		}else{
			current=valuearray[i][0]*mul;
			next=valuearray[i][0]*(mul*pow(firstup, pow(deterioration, (valuearray[i][1]-1))));
		}
		verybasicText(upgradenames[i], SCREEN_WIDTH/6, height, 0.7);
		strs << valuearray[i][0];
		verybasicText(strs.str(), 2*SCREEN_WIDTH/6, height, 0.7);//show base value
		strs.str("");
		strs << current;
		verybasicText(strs.str(), 3*SCREEN_WIDTH/6, height, 0.7);//show current value
		strs.str("");
		strs << valuearray[i][1] << " -> " << valuearray[i][1]+1;
		verybasicText(strs.str(), 4*SCREEN_WIDTH/6, height, 0.7);//text for button
		strs.str("");
		strs << next;
		verybasicText(strs.str(), 5*SCREEN_WIDTH/6, height, 0.7);//show next value
		strs.str("");
		if(tempcharacter.upgradepoints>0)
			addButton(&upgradebuttons, 2*SCREEN_WIDTH/3, height, SCREEN_WIDTH/6, 15);//add buttons
		if(i<14)//render lines to make it easier to read
			SDL_RenderDrawLine(gRenderer, 0.5*SCREEN_WIDTH/6, height+10, 5.5*SCREEN_WIDTH/6, height+10);
	}
	verybasicText("Purchase Weapon Upgrade Point", SCREEN_WIDTH/2, SCREEN_HEIGHT/8+50+15*20, 0.7);
	if(tempcharacter.upgradepoints>0)
		addButton(&upgradebuttons, SCREEN_WIDTH/2, SCREEN_HEIGHT/8+50+15*20, SCREEN_WIDTH/4, 20);
	//render upgrade buttons
	for(int i=0;i<upgradebuttons.size();i++){
        SDL_RenderDrawLine( gRenderer, upgradebuttons[i]->x[upgradebuttons[i]->x.size()-1], upgradebuttons[i]->y[upgradebuttons[i]->x.size()-1], upgradebuttons[i]->x[0], upgradebuttons[i]->y[0] );
        for(int j=0;j<upgradebuttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, upgradebuttons[i]->x[j], upgradebuttons[i]->y[j], upgradebuttons[i]->x[j+1], upgradebuttons[i]->y[j+1] );
    }
    //}
	//detect clicks
    while(loop){
        if(updateMouse()){//updateMouse returns true if quit signal is sent
			for(int i=0;i<upgradebuttons.size();i++)
				delete upgradebuttons.at(i);
			delete menubuttons.at(0);
            delete menubuttons.at(1);
            delete menubuttons.at(2);
            delete menubuttons.at(3);
            return 0;
        }
        if(LmouseClick){
			while(LmouseState){//wait for mouse to be released before continuing, prevents firing projectile as soon as game starts.
				if(updateMouse()){
					for(int i=0;i<upgradebuttons.size();i++)
						delete upgradebuttons.at(i);
					delete menubuttons.at(0);
					delete menubuttons.at(1);
					delete menubuttons.at(2);
					delete menubuttons.at(3);
					return 0;
				}
			}
			SDL_GetMouseState( &mouseX, &mouseY );
            for(int i=0; i<upgradebuttons.size();i++){
                if(isInShape(mouseX, mouseY, *upgradebuttons[i])){
					switch(i){
						case 0:
							tempcharacter.powerup++;
							break;
						case 1:
							tempcharacter.brakeup++;
							break;
						case 2:
							tempcharacter.maxVelocityup++;
							break;
						case 3:
							tempcharacter.massdown++;
							break;
						case 4:
							tempcharacter.energyup++;
							break;
						case 5:
							tempcharacter.generationup++;
							break;
						case 6:
							tempcharacter.healthup++;
							break;
						case 7:
							tempcharacter.shieldup++;
							break;
						case 8:
							tempcharacter.regenup++;
							break;
						case 9:
							tempcharacter.delayup++;
							break;
						case 10:
							tempcharacter.slomomaxup++;
							break;
						case 11:
							tempcharacter.slomoregenup++;
							break;
						case 12:
							tempcharacter.slomousageup++;
							break;
						case 13:
							tempcharacter.slomoresponseup++;
							break;
						case 14:
							tempcharacter.maxrepairsup++;
							break;
						case 15:
							tempcharacter.totalweaponpoints++;
							for(int j=0;j<tempcharacter.weaponUpgrades.size();j++){
								tempcharacter.weaponUpgrades[j].points++;
							}
							break;

					}
					loop=false;
					tempcharacter.upgradepoints--;
                }
            }
            for(int i=0; i<menubuttons.size();i++){
				if(isInShape(mouseX, mouseY, *menubuttons[i])){
					switch(i){
						case 0:
							if(tempcharacter.safematerials>=100){
								tempcharacter.safematerials-=100;
								tempcharacter.upgradepoints=10+(tempcharacter.exp/1000)*pointsperlevel;
								tempcharacter.brakeup=0;
								tempcharacter.delayup=0;
								tempcharacter.energyup=0;
								tempcharacter.generationup=0;
								tempcharacter.healthup=0;
								tempcharacter.massdown=0;
								tempcharacter.maxrepairsup=0;
								tempcharacter.maxVelocityup=0;
								tempcharacter.powerup=0;
								tempcharacter.regenup=0;
								tempcharacter.shieldup=0;
								tempcharacter.slomomaxup=0;
								tempcharacter.slomoregenup=0;
								tempcharacter.slomoresponseup=0;
								tempcharacter.slomousageup=0;
								tempcharacter.totalweaponpoints=0;
								//reset weapons
								for(int i=0;i<tempcharacter.weaponUpgrades.size();i++){
									tempcharacter.weaponUpgrades[i].burst=0;
									tempcharacter.weaponUpgrades[i].damage=0;
									tempcharacter.weaponUpgrades[i].energy=0;
									tempcharacter.weaponUpgrades[i].points=0;
									tempcharacter.weaponUpgrades[i].ROF=0;
									tempcharacter.weaponUpgrades[i].speed=0;
									tempcharacter.weaponUpgrades[i].spread=0;
								}
								loop=false;
							}
							break;
						case 1:
							tempcharacter=character;
							loop=false;
							break;
						case 2:
							character=tempcharacter;
							reloadCharacter();
							if(character.repairs>maxrepairs)
								character.repairs=maxrepairs;
							for(int i=0;i<upgradebuttons.size();i++)
								delete upgradebuttons.at(i);
							delete menubuttons.at(0);
							delete menubuttons.at(1);
							delete menubuttons.at(2);
							return 0;
						case 3:
							selectWeaponMenu(&tempcharacter);
							loop=false;
							break;
					}
				}
            }
		}
        SDL_RenderPresent( gRenderer );
        if((!keys[ pauseKey] && esckeystate)||(!keys[ menuKey] && menukeystate)){//when escape key released
			for(int i=0;i<upgradebuttons.size();i++)
				delete upgradebuttons.at(i);
			delete menubuttons.at(0);
			delete menubuttons.at(1);
			delete menubuttons.at(2);
			delete menubuttons.at(3);
			return 2;//continue game
        }
        esckeystate=keys[ pauseKey];
        menukeystate=keys[ menuKey];
	}
  }
}

uint8_t selectWeaponMenu(Character* tempcharacter){
	std::cout << "Select Weapon Menu\n";
	bool loop = true;
    while(loop){
        SDL_PumpEvents();
        if(!keys[menuKey])
            loop = false;
    }
    bool esckeystate = false;
    bool menukeystate = false;
  while (true){
	loop = true;
	//{render screen
	SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	SDL_RenderClear( gRenderer );
    SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
    std::vector<Shape*> menubuttons;
    std::vector<Shape*> upgradebuttons;
    std::vector<Shape*> equipbuttons;
    //add menu buttons
	addButton(&menubuttons, SCREEN_WIDTH/2, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	verybasicText("Back", SCREEN_WIDTH/2, 3*SCREEN_HEIGHT/4, 0.7);
	//render menu button
    for(int i=0;i<menubuttons.size();i++){
        SDL_RenderDrawLine( gRenderer, menubuttons[i]->x[menubuttons[i]->x.size()-1], menubuttons[i]->y[menubuttons[i]->x.size()-1], menubuttons[i]->x[0], menubuttons[i]->y[0] );
        for(int j=0;j<menubuttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, menubuttons[i]->x[j], menubuttons[i]->y[j], menubuttons[i]->x[j+1], menubuttons[i]->y[j+1] );
    }
    verybasicText("Weapon Menu", SCREEN_WIDTH/2, SCREEN_HEIGHT/8, 1);
	std::ostringstream strs;
	strs << "Materials Remaining: " << tempcharacter->safematerials;
	verybasicText(strs.str(), 2*SCREEN_WIDTH/6, 5*SCREEN_HEIGHT/8, 0.7);
	strs.str("");
	strs << "Total Points: " << tempcharacter->totalweaponpoints;
	verybasicText(strs.str(), 4*SCREEN_WIDTH/6, 5*SCREEN_HEIGHT/8, 0.7);
	strs.str("");
	for(int i=0;i<weapons.size();i++){
		double height=SCREEN_HEIGHT/8+50+i*20;
		verybasicText(weapons[i].name, SCREEN_WIDTH/6, height, 0.7);
		bool unlocked=false;
		int index;
		for(int j=0;j<tempcharacter->weaponUpgrades.size();j++){
			if(tempcharacter->weaponUpgrades[j].ID==weapons[i].ID){
				unlocked=true;
				index=j;
				j=tempcharacter->weaponUpgrades.size();
			}
		}
		addButton(&equipbuttons, 5*SCREEN_WIDTH/6, height, SCREEN_WIDTH/6-4, 15);
		if(unlocked){
			strs << tempcharacter->weaponUpgrades[index].points;
			verybasicText(strs.str(), 2*SCREEN_WIDTH/6, height, 0.7);//show unspent points for weapon
			strs.str("");
			verybasicText("select", 4*SCREEN_WIDTH/6, height, 0.7);
			if(weapons[i].primary){
				verybasicText("Equip Primary", 5*SCREEN_WIDTH/6, height, 0.5);
			}else{
				verybasicText("Equip Secondary", 5*SCREEN_WIDTH/6, height, 0.5);
			}
			if(weapons[i].ID!=tempcharacter->primaryID && weapons[i].ID!=tempcharacter->secondaryID){
				SDL_RenderDrawLine( gRenderer, equipbuttons[i]->x[equipbuttons[i]->x.size()-1], equipbuttons[i]->y[equipbuttons[i]->x.size()-1], equipbuttons[i]->x[0], equipbuttons[i]->y[0] );
					for(int j=0;j<equipbuttons[i]->x.size()-1;j++)
						SDL_RenderDrawLine( gRenderer, equipbuttons[i]->x[j], equipbuttons[i]->y[j], equipbuttons[i]->x[j+1], equipbuttons[i]->y[j+1] );
			}
		}else{
			if(weapons[i].primary){
				verybasicText("Primary Weapon", 5*SCREEN_WIDTH/6, height, 0.5);
			}else{
				verybasicText("Secondary Weapon", 5*SCREEN_WIDTH/6, height, 0.5);
			}
			verybasicText("LOCKED", 2*SCREEN_WIDTH/6, height, 0.7);
			strs << weapons[i].cost << "m";
			verybasicText(strs.str(), 4*SCREEN_WIDTH/6, height, 0.7);
			strs.str("");
		}
/*		strs << current;
		verybasicText(strs.str(), 3*SCREEN_WIDTH/6, height, 0.7);//show current value
		strs.str("");
		strs << valuearray[i][1] << " -> " << valuearray[i][1]+1;
		verybasicText(strs.str(), 4*SCREEN_WIDTH/6, height, 0.7);//text for button
		strs.str("");
		strs << next;
		verybasicText(strs.str(), 5*SCREEN_WIDTH/6, height, 0.7);//show next value
		strs.str("");
		if(tempcharacter.upgradepoints>0)*/
			addButton(&upgradebuttons, 2*SCREEN_WIDTH/3, height, SCREEN_WIDTH/6-4, 15);//add buttons
		if(i<weapons.size()-1)//render lines to make it easier to read
			SDL_RenderDrawLine(gRenderer, 0.5*SCREEN_WIDTH/6, height+10, 5.5*SCREEN_WIDTH/6, height+10);
	}
	//verybasicText("Purchase Weapon Upgrade Point", SCREEN_WIDTH/2, SCREEN_HEIGHT/8+50+15*20, 0.7);
	//if(tempcharacter.upgradepoints>0)
	//	addButton(&upgradebuttons, SCREEN_WIDTH/2, SCREEN_HEIGHT/8+50+15*20, SCREEN_WIDTH/4, 20);
	//render upgrade buttons
	for(int i=0;i<weapons.size();i++){
        SDL_RenderDrawLine( gRenderer, upgradebuttons[i]->x[upgradebuttons[i]->x.size()-1], upgradebuttons[i]->y[upgradebuttons[i]->x.size()-1], upgradebuttons[i]->x[0], upgradebuttons[i]->y[0] );
        for(int j=0;j<upgradebuttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, upgradebuttons[i]->x[j], upgradebuttons[i]->y[j], upgradebuttons[i]->x[j+1], upgradebuttons[i]->y[j+1] );
    }
    //}
	//detect clicks
    while(loop){
        if(updateMouse()){//updateMouse returns true if quit signal is sent
			for(int i=0;i<upgradebuttons.size();i++)
				delete upgradebuttons.at(i);
			for(int i=0;i<equipbuttons.size();i++)
				delete equipbuttons.at(i);
			delete menubuttons.at(0);
            return 0;
        }
        if(LmouseClick){
			while(LmouseState){//wait for mouse to be released before continuing, prevents firing projectile as soon as game starts.
				if(updateMouse()){
					for(int i=0;i<upgradebuttons.size();i++)
						delete upgradebuttons.at(i);
					for(int i=0;i<equipbuttons.size();i++)
						delete equipbuttons.at(i);
					delete menubuttons.at(0);
					return 0;
				}
			}
			SDL_GetMouseState( &mouseX, &mouseY );
            for(int i=0; i<upgradebuttons.size();i++){
                if(isInShape(mouseX, mouseY, *upgradebuttons[i])){
					bool unlocked=false;
					int index;
					for(int j=0;j<tempcharacter->weaponUpgrades.size();j++){
						if(tempcharacter->weaponUpgrades[j].ID==weapons[i].ID){
							unlocked=true;
							index=j;
							j=tempcharacter->weaponUpgrades.size();
						}
					}
					if(unlocked){
						upgradeWeaponMenu(&tempcharacter->weaponUpgrades[index], &tempcharacter->safematerials, tempcharacter);
						loop=false;
					}else{
						if(tempcharacter->safematerials>=weapons[i].cost){
							tempcharacter->safematerials-=weapons[i].cost;
							tempcharacter->weapons.push_back(weapons[i].ID);
							WeaponUpgrades temp;
							temp.burst=0;
							temp.damage=0;
							temp.energy=0;
							temp.ID=weapons[i].ID;
							temp.points=tempcharacter->totalweaponpoints;
							temp.ROF=0;
							temp.speed=0;
							temp.spread=0;
							temp.range=0;
							tempcharacter->weaponUpgrades.push_back(temp);
							loop=false;
						}
					}
                }
            }
            for(int i=0; i<menubuttons.size();i++){
				if(isInShape(mouseX, mouseY, *menubuttons[i])){
					switch(i){
						case 0:
							for(int i=0;i<upgradebuttons.size();i++)
								delete upgradebuttons.at(i);
							for(int i=0;i<equipbuttons.size();i++)
								delete equipbuttons.at(i);
							delete menubuttons.at(0);
							return 0;
					}
				}
            }
            for(int i=0;i<equipbuttons.size();i++){
				if(isInShape(mouseX, mouseY, *equipbuttons[i])){
					bool unlocked=false;
					for(int j=0;j<tempcharacter->weaponUpgrades.size();j++){
						if(tempcharacter->weaponUpgrades[j].ID==weapons[i].ID){
							unlocked=true;
							j=tempcharacter->weaponUpgrades.size();
						}
					}
					if(unlocked){
						if(weapons[i].primary){
							tempcharacter->primaryID=weapons[i].ID;
						}else{
							tempcharacter->secondaryID=weapons[i].ID;
						}
					}
					loop=false;
				}
            }
		}
        SDL_RenderPresent( gRenderer );
        if((!keys[ pauseKey] && esckeystate)||(!keys[ menuKey] && menukeystate)){//when escape key released
			for(int i=0;i<upgradebuttons.size();i++)
				delete upgradebuttons.at(i);
			for(int i=0;i<equipbuttons.size();i++)
				delete equipbuttons.at(i);
			delete menubuttons.at(0);
			return 0;
        }
        esckeystate=keys[ pauseKey];
        menukeystate=keys[ menuKey];
	}
	for(int i=0;i<upgradebuttons.size();i++)
		delete upgradebuttons.at(i);
	for(int i=0;i<equipbuttons.size();i++)
		delete equipbuttons.at(i);
	delete menubuttons.at(0);
  }
}

uint8_t upgradeWeaponMenu(WeaponUpgrades* upgrades, uint32_t* materials, Character* tempcharacter){
	std::cout << "Game Menu\n";
	WeaponUpgrades tempupgrades =  *upgrades;
	uint32_t weaponIndex;
	for(uint32_t i=0;i<weapons.size();i++)
		if(weapons[i].ID==tempupgrades.ID)
			weaponIndex=i;
	bool loop = true;
    while(loop){
        SDL_PumpEvents();
        if(!keys[menuKey])
            loop = false;
    }
    bool esckeystate = false;
    bool menukeystate = false;
  while (true){
	loop = true;
	//{render screen
	SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	SDL_RenderClear( gRenderer );
    SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );
    std::vector<Shape*> menubuttons;
    //add menu buttons
	addButton(&menubuttons, 1*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	addButton(&menubuttons, 11.0/3.0*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	addButton(&menubuttons, 5*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	if(tempcharacter->primaryID!=tempupgrades.ID && tempcharacter->secondaryID!=tempupgrades.ID)
		addButton(&menubuttons, 7.0/3.0*SCREEN_WIDTH/6, 6*SCREEN_HEIGHT/8, 1*SCREEN_WIDTH/6, SCREEN_HEIGHT/10);
	verybasicText("Reset - 100m", 1*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	verybasicText("Undo Changes", 11.0/3.0*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	verybasicText("Apply Changes", 5*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	if(weapons[weaponIndex].primary){
		verybasicText("Equip Primary", 7.0/3.0*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	}else{
		verybasicText("Equip Secondary", 7.0/3.0*SCREEN_WIDTH/6, 3*SCREEN_HEIGHT/4, 0.7);
	}
	//render menu buttons
    for(int i=0;i<menubuttons.size();i++){
        SDL_RenderDrawLine( gRenderer, menubuttons[i]->x[menubuttons[i]->x.size()-1], menubuttons[i]->y[menubuttons[i]->x.size()-1], menubuttons[i]->x[0], menubuttons[i]->y[0] );
        for(int j=0;j<menubuttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, menubuttons[i]->x[j], menubuttons[i]->y[j], menubuttons[i]->x[j+1], menubuttons[i]->y[j+1] );
    }
    verybasicText("Upgrade Weapon Menu", SCREEN_WIDTH/2, SCREEN_HEIGHT/8, 1);
	std::vector<std::string> upgradenames = {"Speed:","Reload:", "Spread:", "Burst:", "Damage:", "Energy:", "Range:"};
	double valuearray[7][2];
	std::vector<Shape*> upgradebuttons;
	valuearray[0][0]=weapons[weaponIndex].speed;
	valuearray[0][1]=tempupgrades.speed;
	valuearray[1][0]=weapons[weaponIndex].ROF;
	valuearray[1][1]=tempupgrades.ROF;
	valuearray[2][0]=weapons[weaponIndex].spread;
	valuearray[2][1]=tempupgrades.spread;
	valuearray[3][0]=weapons[weaponIndex].burst;
	valuearray[3][1]=tempupgrades.burst;
	valuearray[4][0]=weapons[weaponIndex].damage;
	valuearray[4][1]=tempupgrades.damage;
	valuearray[5][0]=weapons[weaponIndex].energy;
	valuearray[5][1]=tempupgrades.energy;
	valuearray[6][0]=weapons[weaponIndex].range;
	valuearray[6][1]=tempupgrades.range;
	std::ostringstream strs;
	strs << "Materials Remaining: " << *materials;
	verybasicText(strs.str(), 2*SCREEN_WIDTH/6, 5*SCREEN_HEIGHT/8, 0.7);
	strs.str("");
	strs << "Upgrade Points: " << tempupgrades.points;
	verybasicText(strs.str(), 4*SCREEN_WIDTH/6, 5*SCREEN_HEIGHT/8, 0.7);
	strs.str("");
	for(int i=0;i<7;i++){
		double height=SCREEN_HEIGHT/8+50+i*20;
		double current;
		double next;
		double mul=1;
		for(int j=0;j<valuearray[i][1];j++){
		mul*=pow(firstup, pow(deterioration, (j-1)));
		}
		if(i==1 || i==2 || i==5){//these values decrease when upgraded
			current=valuearray[i][0]/mul;
			next=valuearray[i][0]/(mul*pow(firstup, pow(deterioration, (valuearray[i][1]-1))));
		}else{
			current=valuearray[i][0]*mul;
			next=valuearray[i][0]*(mul*pow(firstup, pow(deterioration, (valuearray[i][1]-1))));
		}
		verybasicText(upgradenames[i], SCREEN_WIDTH/6, height, 0.7);
		strs << valuearray[i][0];
		verybasicText(strs.str(), 2*SCREEN_WIDTH/6, height, 0.7);//show base value
		strs.str("");
		strs << current;
		verybasicText(strs.str(), 3*SCREEN_WIDTH/6, height, 0.7);//show current value
		strs.str("");
		strs << valuearray[i][1] << " -> " << valuearray[i][1]+1;
		verybasicText(strs.str(), 4*SCREEN_WIDTH/6, height, 0.7);//text for button
		strs.str("");
		strs << next;
		verybasicText(strs.str(), 5*SCREEN_WIDTH/6, height, 0.7);//show next value
		strs.str("");
		if(tempupgrades.points>0)
			addButton(&upgradebuttons, 2*SCREEN_WIDTH/3, height, SCREEN_WIDTH/6, 15);//add buttons
		if(i<6)//render lines to make it easier to read
			SDL_RenderDrawLine(gRenderer, 0.5*SCREEN_WIDTH/6, height+10, 5.5*SCREEN_WIDTH/6, height+10);
	}
	//render upgrade buttons
	for(int i=0;i<upgradebuttons.size();i++){
        SDL_RenderDrawLine( gRenderer, upgradebuttons[i]->x[upgradebuttons[i]->x.size()-1], upgradebuttons[i]->y[upgradebuttons[i]->x.size()-1], upgradebuttons[i]->x[0], upgradebuttons[i]->y[0] );
        for(int j=0;j<upgradebuttons[i]->x.size()-1;j++)
            SDL_RenderDrawLine( gRenderer, upgradebuttons[i]->x[j], upgradebuttons[i]->y[j], upgradebuttons[i]->x[j+1], upgradebuttons[i]->y[j+1] );
    }
    //}
	//detect clicks
    while(loop){
        if(updateMouse()){//updateMouse returns true if quit signal is sent
			for(int i=0;i<upgradebuttons.size();i++)
				delete upgradebuttons.at(i);
			for(int i=0;i<menubuttons.size();i++)
				delete menubuttons.at(i);
			return 0;
        }
        if(LmouseClick){
			while(LmouseState){//wait for mouse to be released before continuing, prevents firing projectile as soon as game starts.
				if(updateMouse()){
					for(int i=0;i<upgradebuttons.size();i++)
						delete upgradebuttons.at(i);
					for(int i=0;i<menubuttons.size();i++)
						delete menubuttons.at(i);
					return 0;
				}
			}
			SDL_GetMouseState( &mouseX, &mouseY );
            for(int i=0; i<upgradebuttons.size();i++){
                if(isInShape(mouseX, mouseY, *upgradebuttons[i])){
					switch(i){
						case 0:
							tempupgrades.speed++;
							break;
						case 1:
							tempupgrades.ROF++;
							break;
						case 2:
							tempupgrades.spread++;
							break;
						case 3:
							tempupgrades.burst++;
							break;
						case 4:
							tempupgrades.damage++;
							break;
						case 5:
							tempupgrades.energy++;
							break;
						case 6:
							tempupgrades.range++;
							break;
					}
					loop=false;
					tempupgrades.points--;
                }
            }
            for(int i=0; i<menubuttons.size();i++){
				if(isInShape(mouseX, mouseY, *menubuttons[i])){
					switch(i){
						case 0:
							if(*materials>=100){
								*materials-=100;
								tempupgrades.points=tempcharacter->totalweaponpoints;
								tempupgrades.burst=0;
								tempupgrades.damage=0;
								tempupgrades.energy=0;
								tempupgrades.ROF=0;
								tempupgrades.speed=0;
								tempupgrades.spread=0;
								loop=false;
							}
							break;
						case 1:
							tempupgrades=*upgrades;
							loop=false;
							break;
						case 2:
							*upgrades=tempupgrades;
							for(int i=0;i<upgradebuttons.size();i++)
								delete upgradebuttons.at(i);
							for(int i=0;i<menubuttons.size();i++)
								delete menubuttons.at(i);
							return 0;
						case 3:
							if(weapons[weaponIndex].primary){
								tempcharacter->primaryID=tempupgrades.ID;
							}else{
								tempcharacter->secondaryID=tempupgrades.ID;
							}
							loop=false;
							break;
					}
				}
            }
		}
        SDL_RenderPresent( gRenderer );
        if((!keys[ pauseKey] && esckeystate)||(!keys[ menuKey] && menukeystate)){//when escape key released
			for(int i=0;i<upgradebuttons.size();i++)
				delete upgradebuttons.at(i);
			for(int i=0;i<menubuttons.size();i++)
				delete menubuttons.at(i);
								return 2;//continue game
        }
        esckeystate=keys[ pauseKey];
        menukeystate=keys[ menuKey];
	}
  }
}

void saveMenu(std::string* data){
    *data = "data/save/save1.game";
}

std::string loadMenu(){
    return "data/save/save1.game";
}

//}

void reloadCharacter(){
	std::cout << "Reloading Character\n";
	tanks[0]->loadfromcharacter();
	double mul=1;
	for(int i=0;i<character.slomomaxup;i++){
		mul*=pow(1.1, pow(0.9, (i-1)));
	}
	maxslomospeed=character.slomomax*mul;
	mul=1;
	for(int i=0;i<character.slomoregenup;i++){
		mul*=pow(1.1, pow(0.9, (i-1)));
	}
	slomoregen=character.slomoregen*mul;
	mul=1;
	for(int i=0;i<character.maxrepairsup;i++){
		mul*=pow(1.1, pow(0.9, (i-1)));
	}
	maxrepairs=floor(character.maxrepairs*mul);
	mul=1;
	for(int i=0;i<character.slomousageup;i++){
		mul*=pow(1.1, pow(0.9, (i-1)));
	}
	slomousage=character.slomousage/mul;
	mul=1;
	for(int i=0;i<character.slomoresponseup;i++){
		mul*=pow(1.1, pow(0.9, (i-1)));
	}
	slomoresponse=character.slomoresponse*mul;
}

//{Save and load structs
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
    strs << save.room << "\n";
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
    getline(*file,line);
    load.room=atoi(line.c_str());
    return load;
}
void saveShape(Shape save, std::ofstream* file){
    std::ostringstream strs;
    strs << save.room << "\n";
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
    load.room = atoi(line.c_str());
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
    strs << save.totaldist << "\n";
    strs << save.range << "\n";
    strs << save.lastid << "\n";
    strs << save.ownerID << "\n";
    if(save.primaryWeapon){
        strs << "t\n";
    }else{
        strs << "f\n";
    }
    strs << save.room << "\n";
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
    load.totaldist=atof(line.c_str());
    getline(*file,line);
    load.range=atof(line.c_str());
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
    getline(*file,line);
    load.room=atoi(line.c_str());
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
void saveCharacter(Character save, std::ofstream* file){
	std::ostringstream strs;
    strs << save.brake << "\n";
    strs << save.brakeup << "\n";
    strs << save.delay << "\n";
    strs << save.delayup << "\n";
    strs << save.energy << "\n";
    strs << save.energyup << "\n";
    strs << save.exp << "\n";
    strs << save.generation << "\n";
    strs << save.generationup << "\n";
    strs << save.health << "\n";
    strs << save.healthup << "\n";
    strs << save.mass << "\n";
    strs << save.massdown << "\n";
    strs << save.materials << "\n";
    strs << save.repairs << "\n";
    strs << save.maxrepairs << "\n";
    strs << save.maxrepairsup << "\n";
    strs << save.safematerials << "\n";
    strs << save.maxVelocity << "\n";
    strs << save.maxVelocityup << "\n";
    strs << save.name << "\n";
    strs << save.power << "\n";
    strs << save.powerup << "\n";
    strs << save.regen << "\n";
    strs << save.regenup << "\n";
    strs << save.shield << "\n";
    strs << save.shieldup << "\n";
    strs << save.slomomax << "\n";
    strs << save.slomomaxup << "\n";
    strs << save.slomoregen << "\n";
    strs << save.slomoregenup << "\n";
    strs << save.slomoresponse << "\n";
    strs << save.slomoresponseup << "\n";
    strs << save.slomousage << "\n";
    strs << save.slomousageup << "\n";
    strs << save.upgradepoints << "\n";
    strs << save.weapons.size() << "\n";
    for(int i=0;i<save.weapons.size();i++)
		strs << save.weapons[i] << "\n";
    //save weapon upgrades
    strs << save.primaryID << "\n";
    strs << save.secondaryID << "\n";
    strs << save.totalweaponpoints << "\n";
    *file << strs.str();
    for(int i=0;i<save.weaponUpgrades.size();i++)
		saveWeaponUpgrades(save.weaponUpgrades[i], file);
}
Character loadCharacter(std::ifstream* file){
	Character load;
	std::string line;
	getline(*file, line);
	load.brake=atof(line.c_str());
	getline(*file, line);
	load.brakeup=atoi(line.c_str());
	getline(*file, line);
	load.delay=atof(line.c_str());
	getline(*file, line);
	load.delayup=atoi(line.c_str());
	getline(*file, line);
	load.energy=atof(line.c_str());
	getline(*file, line);
	load.energyup=atoi(line.c_str());
	getline(*file, line);
	load.exp=atoi(line.c_str());
	getline(*file, line);
	load.generation=atof(line.c_str());
	getline(*file, line);
	load.generationup=atoi(line.c_str());
	getline(*file, line);
	load.health=atof(line.c_str());
	getline(*file, line);
	load.healthup=atoi(line.c_str());
	getline(*file, line);
	load.mass=atof(line.c_str());
	getline(*file, line);
	load.massdown=atoi(line.c_str());
	getline(*file, line);
	load.materials=atoi(line.c_str());
	getline(*file, line);
	load.repairs=atoi(line.c_str());
	getline(*file, line);
	load.maxrepairs=atoi(line.c_str());
	getline(*file, line);
	load.maxrepairsup=atoi(line.c_str());
	getline(*file, line);
	load.safematerials=atoi(line.c_str());
	getline(*file, line);
	load.maxVelocity=atof(line.c_str());
	getline(*file, line);
	load.maxVelocityup=atoi(line.c_str());
	getline(*file, line);
	load.name=line;
	getline(*file, line);
	load.power=atof(line.c_str());
	getline(*file, line);
	load.powerup=atoi(line.c_str());
	getline(*file, line);
	load.regen=atof(line.c_str());
	getline(*file, line);
	load.regenup=atoi(line.c_str());
	getline(*file, line);
	load.shield=atof(line.c_str());
	getline(*file, line);
	load.shieldup=atoi(line.c_str());
	getline(*file, line);
	load.slomomax=atof(line.c_str());
	getline(*file, line);
	load.slomomaxup=atoi(line.c_str());
	getline(*file, line);
	load.slomoregen=atof(line.c_str());
	getline(*file, line);
	load.slomoregenup=atoi(line.c_str());
	getline(*file, line);
	load.slomoresponse=atof(line.c_str());
	getline(*file, line);
	load.slomoresponseup=atoi(line.c_str());
	getline(*file, line);
	load.slomousage=atof(line.c_str());
	getline(*file, line);
	load.slomousageup=atoi(line.c_str());
	getline(*file, line);
	load.upgradepoints=atoi(line.c_str());
	getline(*file, line);
	int weaponssize=atoi(line.c_str());
	for(int i=0;i<weaponssize;i++){
		getline(*file, line);
		load.weapons.push_back(atoi(line.c_str()));
	}
	getline(*file, line);
	load.primaryID=atoi(line.c_str());
	getline(*file, line);
	load.secondaryID=atoi(line.c_str());
	getline(*file, line);
	load.totalweaponpoints=atoi(line.c_str());
	for(int i=0;i<weaponssize;i++){
		load.weaponUpgrades.push_back(loadWeaponUpgrades(file));
	}
	return load;
}
void saveWeaponUpgrades(WeaponUpgrades save, std::ofstream* file){
	std::ostringstream strs;
    strs << save.burst << "\n";
    strs << save.damage << "\n";
    strs << save.energy << "\n";
    strs << save.ID << "\n";
    strs << save.points << "\n";
    strs << save.ROF << "\n";
    strs << save.speed << "\n";
    strs << save.spread << "\n";
	strs << save.range << "\n";
    *file << strs.str();
}
WeaponUpgrades loadWeaponUpgrades(std::ifstream* file){
	WeaponUpgrades load;
	std::string line;
	getline(*file, line);
	load.burst=atoi(line.c_str());
	getline(*file, line);
	load.damage=atoi(line.c_str());
	getline(*file, line);
	load.energy=atoi(line.c_str());
	getline(*file, line);
	load.ID=atoi(line.c_str());
	getline(*file, line);
	load.points=atoi(line.c_str());
	getline(*file, line);
	load.ROF=atoi(line.c_str());
	getline(*file, line);
	load.speed=atoi(line.c_str());
	getline(*file, line);
	load.spread=atoi(line.c_str());
	getline(*file, line);
	load.range=atoi(line.c_str());
	return load;
}
void saveStatus(std::ofstream* file){
	std::ostringstream strs;
	strs << slomo << "\n";
	if(primaryWeaponAssistToggle)
		strs << "t\n";
	else
		strs << "f\n";
	*file << strs.str();
}
void loadStatus(std::ifstream* file){
	std::string line;
	getline(*file, line);
	slomo=atof(line.c_str());
	getline(*file, line);
	if(line.at(0)=='t')
		primaryWeaponAssistToggle=true;
	else
		primaryWeaponAssistToggle=false;
}
void savePortal(Portal save, std::ofstream* file){
	std::ostringstream strs;
	strs << save.room << "\n";
	strs << save.x << "\n";
	strs << save.y << "\n";
	strs << save.w << "\n";
	strs << save.h << "\n";
	strs << save.id << "\n";
	strs << save.toid << "\n";
	if(save.inwall)
		strs << "t\n";
	else
		strs << "f\n";
	*file << strs.str();
}
Portal loadPortal(std::ifstream* file){
	Portal load;
	std::string line;
	getline(*file, line);
	load.room = atoi(line.c_str());
	getline(*file, line);
	load.x = atoi(line.c_str());
	getline(*file, line);
	load.y = atoi(line.c_str());
	getline(*file, line);
	load.w = atoi(line.c_str());
	getline(*file, line);
	load.h = atoi(line.c_str());
	getline(*file, line);
	load.id = atoi(line.c_str());
	getline(*file, line);
	load.toid = atoi(line.c_str());
	getline(*file, line);
	if(line.at(0)=='t')
		load.inwall=true;
	else
		load.inwall=false;
	return load;
}
//}

bool loadSettings(char* filename){
    std::ifstream infile(filename);
    std::string line;
    bool skipMenu=false;
    while (std::getline(infile, line)){//read from file
        if(line.length()!=0){
            if(line.find("skipMenu")!=std::string::npos){
                if(line.substr(line.find("-")+1)=="true")
                    skipMenu=true;
            }else if(line.find("particleEffects")!=std::string::npos){
                if(line.substr(line.find("-")+1)=="true")
                    particleEffects=true;
            }else if(line.find("gamespeed")!=std::string::npos){
				gamespeed=atof(line.substr(line.find("-")+1).c_str());
            }else if(line.find("deterioration")!=std::string::npos){
				deterioration=atof(line.substr(line.find("-")+1).c_str());
            }else if(line.find("streakrenderres")!=std::string::npos){
				streakrenderres=atof(line.substr(line.find("-")+1).c_str());
            }else if(line.find("physics")!=std::string::npos){
				physics=atoi(line.substr(line.find("-")+1).c_str());
            }else if(line.find("firstup")!=std::string::npos){
				firstup=atof(line.substr(line.find("-")+1).c_str());
            }else if(line.find("savelevel")!=std::string::npos){
				savelevel=atoi(line.substr(line.find("-")+1).c_str());
			}else if(line.compare("theflash")==0){
				theflash=true;
            }else if(line.find("fps")!=std::string::npos){
				MAX_FPS=atof(line.substr(line.find("-")+1).c_str());
				FPS=MAX_FPS;
            }else if(line.find("mul")!=std::string::npos){
				mul=atof(line.substr(line.find("-")+1).c_str());
            }
        }
    }
    return skipMenu;
}

void loadTankTemplates(char* filename){
    Tank* temp = new Tank("unnamed", 0, 0, 0, 0);
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

    uint32_t expgiven=0;
    uint32_t materialsgiven=0;

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
                t.positionReset(line.substr(1), 0, 0, 0, 0, true);
            }else if(line.at(0)=='a'){//add template and start a new one
                t.setDimensions(length, width, turretradius, barrellength, barrelwidth, pprojlength, pprojwidth, sprojlength, sprojwidth);
                t.setOther(ID, power, brakeforce, maxvel, mass, maxEnergy, energyGen, expgiven, materialsgiven);
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
                expgiven=0;
                materialsgiven=0;
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
                }else if(line.at(1)=='x'){
					expgiven = atoi(line.substr(2).c_str());
                }else if(line.at(1)=='a'){
					materialsgiven=atoi(line.substr(2).c_str());
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
    double a=0;
    uint16_t r=0;
    bool templated = false;
    while (std::getline(infile, line)){
        if(line.length()!=0){
            if(line.at(0)=='n'){//set to name
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
            }else if(line.at(0)=='i'){//set to ID
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
            }else if(line.at(0)=='+'){//add
                if(templated){
                    t->positionReset("", x, y, a, r, true);
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
            }else if(line.at(0)=='a'){//angle
                a = atof(line.substr(1).c_str());
            }else if(line.at(0)=='r'){//room
				r = atoi(line.substr(1).c_str());
            }
        }
    }
    for(int i=0;i<tanks.size();i++){
        tanks[i]->setIndex(i);
	}
}

void setMap(char* filename){//modify barrier generation to allow for inwall portals
    loadMap(&shapes, filename);
	//set barriers for each room
	int t=shapes.size();
	for(int i=0;i<roomDimensions.size();i++){
		Shape* shape = new Shape;
		shape->room = roomDimensions[i].room;
		std::vector<double>* x = new std::vector<double>;
		std::vector<double>* y = new std::vector<double>;
		//top barrier
		bool toggle = true;
		for(int j=0;j<portals.size();j++){
			if(portals[j].room==roomDimensions[i].room && portals[j].inwall && portals[j].y<0){
				toggle=false;
			}
		}
		if(!toggle){
			int start = -250;
			int stop = roomDimensions[i].mapwidth+250;
			int id;
            while(start<roomDimensions[i].mapwidth+250){
				stop = roomDimensions[i].mapwidth+250;
				for(int j=0;j<portals.size();j++){
					if(portals[j].room==roomDimensions[i].room && portals[j].inwall && portals[j].y<0 && portals[j].x<stop && portals[j].x>start){
						id=j;
						stop=portals[j].x;
					}
				}
				delete x;
				delete y;
				x = new std::vector<double>;
				y = new std::vector<double>;
				x->push_back(start);
				x->push_back(stop);
				x->push_back(stop);
				x->push_back(start);
				y->push_back(-250);
				y->push_back(-250);
				y->push_back(0);
				y->push_back(0);
				sortpoints(x, y);
				shape->x = *x;
				shape->y = *y;
				shapes.push_back(shape);
				//std::cout << shapes.size()-1 << "\n";
				//SDL_Delay(1000);
				shape = new Shape;
				shape->room = roomDimensions[i].room;
				if(stop==roomDimensions[i].mapwidth+250)
					start=stop;
				else{
					start=portals[id].x+portals[id].w;
				}
            }
		}else{
			x->push_back(-250);
			x->push_back(roomDimensions[i].mapwidth+250);
			x->push_back(roomDimensions[i].mapwidth+250);
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
			shape->room = roomDimensions[i].room;
		}

		//bottom barrier
		toggle = true;
		for(int j=0;j<portals.size();j++){
			if(portals[j].room==roomDimensions[i].room && portals[j].inwall && (portals[j].y+portals[j].h)>roomDimensions[i].mapheight){
				toggle=false;
			}
		}
		if(!toggle){
			int start = -250;
			int stop = roomDimensions[i].mapwidth+250;
			int id;
            while(start<roomDimensions[i].mapwidth+250){
				stop = roomDimensions[i].mapwidth+250;
				for(int j=0;j<portals.size();j++){
					if(portals[j].room==roomDimensions[i].room && portals[j].inwall && (portals[j].y+portals[j].h)>roomDimensions[i].mapheight && portals[j].x<stop && portals[j].x>start){
						id=j;
						stop=portals[j].x;
					}
				}
				delete x;
				delete y;
				x = new std::vector<double>;
				y = new std::vector<double>;
				x->push_back(start);
				x->push_back(stop);
				x->push_back(stop);
				x->push_back(start);
				y->push_back(roomDimensions[i].mapheight);
				y->push_back(roomDimensions[i].mapheight);
				y->push_back(roomDimensions[i].mapheight+250);
				y->push_back(roomDimensions[i].mapheight+250);
				sortpoints(x, y);
				shape->x = *x;
				shape->y = *y;
				shapes.push_back(shape);
				//std::cout << shapes.size()-1 << "\n";
				//SDL_Delay(1000);
				shape = new Shape;
				shape->room = roomDimensions[i].room;
				if(stop==roomDimensions[i].mapwidth+250)
					start=stop;
				else{
					start=portals[id].x+portals[id].w;
				}
            }
		}else{
			delete x;
			delete y;
			x = new std::vector<double>;
			y = new std::vector<double>;
			x->push_back(-250);
			x->push_back(roomDimensions[i].mapwidth+250);
			x->push_back(roomDimensions[i].mapwidth+250);
			x->push_back(-250);
			y->push_back(roomDimensions[i].mapheight+250);
			y->push_back(roomDimensions[i].mapheight+250);
			y->push_back(roomDimensions[i].mapheight);
			y->push_back(roomDimensions[i].mapheight);
			sortpoints(x, y);
			shape->x = *x;
			shape->y = *y;
			shapes.push_back(shape);
			shape = new Shape;
			shape->room = roomDimensions[i].room;
		}

		//left barrier
		toggle = true;
		for(int j=0;j<portals.size();j++){
			if(portals[j].room==roomDimensions[i].room && portals[j].inwall && portals[j].x<0){
				toggle=false;
			}
		}
		if(!toggle){
			int start = -250;
			int stop = roomDimensions[i].mapheight+250;
			int id;
            while(start<roomDimensions[i].mapheight+250){
				stop = roomDimensions[i].mapheight+250;
				for(int j=0;j<portals.size();j++){
					if(portals[j].room==roomDimensions[i].room && portals[j].inwall && portals[j].x<0 && portals[j].y<stop && portals[j].y>start){
						id=j;
						stop=portals[j].y;
					}
				}
				delete x;
				delete y;
				x = new std::vector<double>;
				y = new std::vector<double>;
				x->push_back(-250);
				x->push_back(0);
				x->push_back(0);
				x->push_back(-250);
				y->push_back(start);
				y->push_back(start);
				y->push_back(stop);
				y->push_back(stop);
				sortpoints(x, y);
				shape->x = *x;
				shape->y = *y;
				shapes.push_back(shape);
				//std::cout << shapes.size()-1 << "\n";
				//SDL_Delay(1000);
				shape = new Shape;
				shape->room = roomDimensions[i].room;
				if(stop==roomDimensions[i].mapheight+250)
					start=stop;
				else{
					start=portals[id].y+portals[id].h;
				}
            }
		}else{
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
			y->push_back(roomDimensions[i].mapheight+250);
			y->push_back(roomDimensions[i].mapheight+250);
			sortpoints(x, y);
			shape->x = *x;
			shape->y = *y;
			shapes.push_back(shape);
			shape = new Shape;
			shape->room = roomDimensions[i].room;
		}

		//right barrier
		toggle = true;
		for(int j=0;j<portals.size();j++){
			if(portals[j].room==roomDimensions[i].room && portals[j].inwall && portals[j].x>roomDimensions[i].mapwidth){
				toggle=false;
			}
		}
		if(!toggle){
			int start = -250;
			int stop = roomDimensions[i].mapheight+250;
			int id;
            while(start<roomDimensions[i].mapheight+250){
				stop = roomDimensions[i].mapheight+250;
				for(int j=0;j<portals.size();j++){
					if(portals[j].room==roomDimensions[i].room && portals[j].inwall && portals[j].x>roomDimensions[i].mapwidth && portals[j].y<stop && portals[j].y>start){
						id=j;
						stop=portals[j].y;
					}
				}
				delete x;
				delete y;
				x = new std::vector<double>;
				y = new std::vector<double>;
				x->push_back(roomDimensions[i].mapwidth);
				x->push_back(roomDimensions[i].mapwidth+250);
				x->push_back(roomDimensions[i].mapwidth+250);
				x->push_back(roomDimensions[i].mapwidth);
				y->push_back(start);
				y->push_back(start);
				y->push_back(stop);
				y->push_back(stop);
				sortpoints(x, y);
				shape->x = *x;
				shape->y = *y;
				shapes.push_back(shape);
				//std::cout << shapes.size()-1 << "\n";
				//SDL_Delay(1000);
				shape = new Shape;
				shape->room = roomDimensions[i].room;
				if(stop==roomDimensions[i].mapheight+250)
					start=stop;
				else{
					start=portals[id].y+portals[id].h;
				}
            }
		}else{
			delete x;
			delete y;
			x = new std::vector<double>;
			y = new std::vector<double>;
			x->push_back(roomDimensions[i].mapwidth);
			x->push_back(roomDimensions[i].mapwidth+250);
			x->push_back(roomDimensions[i].mapwidth+250);
			x->push_back(roomDimensions[i].mapwidth);
			y->push_back(-250);
			y->push_back(-250);
			y->push_back(roomDimensions[i].mapheight+250);
			y->push_back(roomDimensions[i].mapheight+250);
			sortpoints(x, y);
			shape->x = *x;
			shape->y = *y;
			shapes.push_back(shape);
			delete x;
			delete y;
		}
	}
	for(int i=t;i<shapes.size();i++){
		shapes[i]->isRectangle=true;
	}

	for(int i=0;i<shapes.size();i++){//preload internal angles for collision detection
		preLoadAngles(shapes[i]);
	}
	std::cout << "MAP SET\n" << shapes.size() << "\n";
}

void loadMap(std::vector<Shape*>* shapestemp, char* filename){
    uint16_t room = 0;
    std::vector<double>* x = new std::vector<double>;
    std::vector<double>* y = new std::vector<double>;
    Portal portal;
    portal.inwall=false;
    Shape* shape;
    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)){//read from file
        if(line.length()!=0){
            if(line.at(0)=='w'){//map width
				bool tmp = true;
                for(int i=0;i<roomDimensions.size();i++)
					if(roomDimensions[i].room==room){
						roomDimensions[i].mapwidth = atoi(line.substr(1).c_str());
						tmp = false;
					}
				if(tmp){
					RoomDimension dimension;
					dimension.room = room;
					dimension.mapwidth = atoi(line.substr(1).c_str());
					roomDimensions.push_back(dimension);
				}
            }else if(line.at(0)=='h'){//map height
                bool tmp = true;
                for(int i=0;i<roomDimensions.size();i++)
					if(roomDimensions[i].room==room){
						roomDimensions[i].mapheight = atoi(line.substr(1).c_str());
						tmp = false;
					}
				if(tmp){
					RoomDimension dimension;
					dimension.room = room;
					dimension.mapheight = atoi(line.substr(1).c_str());
					roomDimensions.push_back(dimension);
				}
            }else if(line.at(0)=='x')//x coordinate of corner
                x->push_back(atof(line.substr(1).c_str()));
            else if(line.at(0)=='y')//y coordinate of corner
                y->push_back(atof(line.substr(1).c_str()));
            else if(line.at(0)=='s'){//new shape
                if(x->size()==y->size() && x->size()>1){
					sortpoints(x, y);
					shape = new Shape;
					if(x->size()==4 && x->at(0)==x->at(1) && x->at(2)==x->at(3) && y->at(0)==y->at(3) && y->at(1)==y->at(2)){
						shape->isRectangle=true;
					}
					shape->x = *x;
					shape->y = *y;
					shape->room = room;
					shapestemp->push_back(shape);
                }
                delete x;
                delete y;
                x = new std::vector<double>;
                y = new std::vector<double>;
            }else if(line.at(0)=='r')
				room=atoi(line.substr(1).c_str());
			else if(line.at(0)=='p'){
				if(line.at(1)=='x')
					portal.x=atof(line.substr(2).c_str());
				else if(line.at(1)=='y')
					portal.y=atof(line.substr(2).c_str());
				else if(line.at(1)=='w')
					portal.w=atof(line.substr(2).c_str());
				else if(line.at(1)=='h')
					portal.h=atof(line.substr(2).c_str());
				else if(line.at(1)=='i')
					portal.id=atoi(line.substr(2).c_str());
				else if(line.at(1)=='t')
					portal.toid=atoi(line.substr(2).c_str());
				else if(line.at(1)=='n'){
					portal.inwall=true;
				}else if(line.at(1)=='a'){
					portal.room=room;
					portals.push_back(portal);
					portal.inwall=false;
				}
			}
        }
    }
    delete x;
    delete y;
}

Character loadCharacter(char* filename){
	Character load;
	load.slomoregenup=0;
	load.slomomaxup=0;
	load.slomoresponseup=0;
	load.slomousageup=0;
	load.brakeup=0;
	load.delayup=0;
	load.energyup=0;
	load.exp=0;
	load.generationup=0;
	load.healthup=0;
	load.massdown=0;
	load.maxVelocityup=0;
	load.powerup=0;
	load.regenup=0;
	load.shieldup=0;
	load.maxrepairsup=0;
	load.materials=-1;
	load.totalweaponpoints=0;
	load=updateCharacter(filename, load);
	return load;
}

Character updateCharacter(char* filename, Character load){
	std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)){//read from file
        if(line.length()!=0){
			if(line.at(0)=='b')
				load.brake=atof(line.substr(1).c_str());
			else if(line.at(0)=='d')
				load.delay=atof(line.substr(1).c_str());
			else if(line.at(0)=='e')
				load.energy=atof(line.substr(1).c_str());
			else if(line.at(0)=='g')
				load.generation=atof(line.substr(1).c_str());
			else if(line.at(0)=='h')
				load.health=atof(line.substr(1).c_str());
			else if(line.at(0)=='m')
				load.mass=atof(line.substr(1).c_str());
			else if(line.at(0)=='t'){
				if(load.materials==-1)
					load.materials=atoi(line.substr(1).c_str());
			}else if(line.at(0)=='v')
				load.maxVelocity=atof(line.substr(1).c_str());
			else if(line.at(0)=='n'){
				load.name=line.substr(1);
			}else if(line.at(0)=='p')
				load.power=atof(line.substr(1).c_str());
			else if(line.at(0)=='r')
				load.regen=atof(line.substr(1).c_str());
			else if(line.at(0)=='s')
				load.shield=atof(line.substr(1).c_str());
			else if(line.at(0)=='u')
				load.upgradepoints=atoi(line.substr(1).c_str());
			else if(line.at(0)=='w')
				load.weapons.push_back(atoi(line.substr(1).c_str()));
			else if(line.at(0)=='0')
				load.primaryID=atoi(line.substr(1).c_str());
			else if(line.at(0)=='1')
				load.secondaryID=atoi(line.substr(1).c_str());
			else if(line.at(0)=='l'){
				line=line.substr(1);
				if(line.at(0)=='r')
					load.slomoregen=atof(line.substr(1).c_str());
				else if(line.at(0)=='u')
					load.slomousage=atof(line.substr(1).c_str());
				else if(line.at(0)=='m')
					load.slomomax=atof(line.substr(1).c_str());
				else if(line.at(0)=='s')
					load.slomoresponse=atof(line.substr(1).c_str());
			}else if(line.at(0)=='f'){
				load.maxrepairs=atoi(line.substr(1).c_str());
			}
        }
	}
	WeaponUpgrades noupgrades;
	noupgrades.burst=0;
	noupgrades.damage=0;
	noupgrades.energy=0;
	noupgrades.points=0;
	noupgrades.ROF=0;
	noupgrades.speed=0;
	noupgrades.spread=0;
	noupgrades.range=0;
	for(int i=0;i<load.weapons.size();i++){
		noupgrades.ID=load.weapons[i];
		load.weaponUpgrades.push_back(noupgrades);
	}
	return load;
}
std::vector<Weapon> loadWeapons(char* filename){
	std::vector<Weapon> load;
	Weapon tmp;
	std::ifstream infile(filename);
    std::string line;
    tmp.passesshields=false;
    tmp.primary=false;
    while (std::getline(infile, line)){//read from file
        if(line.length()!=0){
			if(line.at(0)=='b')
				tmp.burst=atoi(line.substr(1).c_str());
			else if(line.at(0)=='d')
				tmp.damage=atof(line.substr(1).c_str());
			else if(line.at(0)=='e')
				tmp.energy=atof(line.substr(1).c_str());
			else if(line.at(0)=='i')
				tmp.ID=atoi(line.substr(1).c_str());
			else if(line.at(0)=='n')
				tmp.name=line.substr(1);
			else if(line.at(0)=='p')
				tmp.passesshields=true;
			else if(line.at(0)=='1')
				tmp.primary=true;
			else if(line.at(0)=='w')
				tmp.ROF=atoi(line.substr(1).c_str());
			else if(line.at(0)=='g')
				tmp.speed=atof(line.substr(1).c_str());
			else if(line.at(0)=='s')
				tmp.spread=atof(line.substr(1).c_str());
			else if(line.at(0)=='c')
				tmp.cost=atoi(line.substr(1).c_str());
			else if(line.at(0)=='r')
				tmp.range=atoi(line.substr(1).c_str());
			else if(line.at(0)=='a'){
				load.push_back(tmp);
				tmp.passesshields=false;
				tmp.primary=false;
			}
		}
	}
	return load;
}

void fire(bool pfire, bool sfire, Tank* tank, std::vector<Projectile*>* projectiles){
    if(pfire && tank->primaryReloaded()){
        for(int i=0; i<tank->getPburst(); i++){
			if(tank->getEnergy()>=tank->getPEnergy()){
				Projectile* t = new Projectile;
				projectiles->push_back(t);
				tank->shootPrimary(t);
			}
        }
    }
   	if(sfire && tank->secondaryReloaded()){
        for(int i=0; i<tank->getSburst(); i++){
			if(tank->getEnergy()>=tank->getSEnergy()){
				Projectile* t = new Projectile;
				projectiles->push_back(t);
				tank->shootSecondary(t);
            }
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
    part.room = proj->room;
    }
    proj->dist++;
    //detect collision
	for(int i=0; i<tanks.size(); i++){
	if(tanks[i]->getCurrentRoom()==proj->room){
        Shape temp = tanks[i]->getShape();
        if((ret == false)&&(passedovershape(proj->x, proj->y, proj->x+proj->xvel*stepSize, proj->y+proj->yvel*stepSize, temp) || isInShape(proj->x, proj->y, temp))){
            //std::cout << "Tank " << i << " hit by a projectile\n";
            if(proj->ownerID==tanks[0]->getID()){
				uint32_t exp=(tanks[i]->takeDamage(proj->damage, proj->passesShield));
				character.exp+=exp;
				if(character.exp%1000<exp)//must have gained level
					character.upgradepoints+=pointsperlevel;
				tanks[i]->pointTo(tanks[0]->getX(), tanks[0]->getY());
            }else
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
	}
    for(int i=0; i<shapes.size(); i++){
	if(shapes[i]->room==proj->room){
        Shape* temp = shapes[i];
        if((ret == false)&&(passedovershape(proj->x, proj->y, proj->x+proj->xvel*stepSize, proj->y+proj->yvel*stepSize, *temp) || isInShape(proj->x, proj->y, *temp))){
            //std::cout << "Shape " << i << " hit by a projectile\n";
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
	}
	if(proj->x<-250 || proj->y<-250 || proj->x>getRoomDimension(proj->room).mapwidth+250 || proj->y>getRoomDimension(proj->room).mapheight+250){
		ret=true;
		//std::cout << "Projectile left room\n";
	}
	if(proj->range<proj->totaldist && proj->range!=0)
		ret=true;
	if(!ret){
	//render projectile
    SDL_Rect renderposition = {xToDispX(proj->x+proj->xvel*stepSize-proj->width/2), yToDispY(proj->y+proj->yvel*stepSize-proj->length/2), proj->width*xzoom, proj->length*yzoom};						//x-width/2 is used, so that the x and y coordinates refer to the center of the tank
	SDL_RenderCopyEx( gRenderer, proj->look, proj->clip, &renderposition, proj->angle+90, NULL, SDL_FLIP_NONE);
	}
    proj->x += proj->xvel*stepSize;
    proj->y += proj->yvel*stepSize;
    proj->totaldist+=sqrt(pow(proj->xvel, 2)+pow(proj->yvel, 2))*stepSize;
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
	if(sqrt(pow(tanks[tank1]->getX()-tanks[tankn]->getX(), 2)+pow(tanks[tank1]->getY()-tanks[tankn]->getY(), 2))>tanks[tank1]->getViewDistance(true)){//if out of sight
		return false;
	}
	if(tanks[tankn]->isDead())//invisible tanks are ignored
		return false;
	uint16_t room = tanks[tank1]->getCurrentRoom();
	if(room!=tanks[tankn]->getCurrentRoom())
		return false;
    //Find angle and distance to tank
    //Rotate clockwise from angle and check if collision with given tank and nothing else
    //same but clockwise
    double x = tanks[tank1]->getX();    //coordinates of tank
    double y = tanks[tank1]->getY();
    double tx = tanks[tankn]->getX();   //coordinates of target
    double ty = tanks[tankn]->getY();
//   	bool visible = false;
   	bool tvisible = true;   //keeps track of whether the target is visible
   	for(int i=0; i<tanks.size()&&tvisible==true; i++){  //check through all shapes that are not the tank or target; if none are blocking the view tvisible is left true
        if(i!=tankn && i!=tank1 && tanks[i]->getCurrentRoom()==room){
            Shape temp = tanks[i]->getShape();
            if(passedovershape(x, y, tx, ty, temp)){// || isInShape(x, y, temp)){//wait why check if in shape???
                tvisible = false;
            }
        }
	}
    for(int i=0; i<shapes.size()&&tvisible==true; i++){
		if(shapes[i]->room==room){
			Shape* temp = shapes[i];
			if(passedovershape(x, y, tx, ty, *temp)){// || isInShape(x, y, *temp)){
				tvisible = false;
			}
		}
	}
	if(tvisible == true){   //if target is visible return true
        return true;
	}
	//no direct line of sight to centre of target...
	if((frameCount+tank1)%precisevisionfrequency!=0){//only do complex once every few frames (add tank1 so each tank is done in a different frame)
		return false;
	}
	double angle = atan2(y, x, ty, tx); //set angle to target
	double tangle = angle;  //angle changed to check if target is visible
	bool run = true;    //variable for while loop
	int checks = 0; //track the number of angles checked
	double acdist = distanceto(x, y, angle, tankn, 0, 10000);   //calculate distance to front face of target to allow for later optimisations
	double dist = acdist; //distance to the target at tangle
	while(run){
        checks++;   //increment the counter
        tangle+=scanangleincrement; //increment tangle
        dist = distanceto(x, y, tangle, tankn, dist-30, dist+30);   //find exact distance to target, use last distance to guess in approximate location to speed things up
        double c = 0 - cos(tangle); //set cos and sin of angle
        double s = sin(tangle);
        tx = x + c * dist;  //coordinates of edge of target at tangle from the tank
        ty = y + s * dist;
//        std::cout << "acdist: " << acdist << "\ndistance: " << dist << "\nangle: " << tangle << "\nx: " << tx << "\ny: " << ty << "\n";
        if(dist == 0)   //if distance is 0, target is not found at angle tangle from the tank within the given range
            run = false;
        else{
        tvisible = true;    //perform the same check as before but with a different angle
           	for(int i=0; i<tanks.size()&&tvisible; i++){
                if(i!=tankn && i!=tank1 && tanks[i]->getCurrentRoom()==room){
                    Shape temp = tanks[i]->getShape();
                    if(passedovershape(x, y, tx, ty, temp)){
                        tvisible = false;
                    }
                }
            }
            for(int i=0; i<shapes.size()&&tvisible; i++){
				if(shapes[i]->room==room){
					Shape* temp = shapes[i];
					if(passedovershape(x, y, tx, ty, *temp)){
						tvisible = false;
					}
				}
            }
        if(tvisible)
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
	dist=acdist;
	while(run){
        checks++;
        tangle-=scanangleincrement;
        dist = distanceto(x, y, tangle, tankn, dist-30, dist+30);
        double c = 0 - cos(tangle);
        double s = sin(tangle);
        tx = x + c * dist;
        ty = y + s * dist;
//        std::cout << "distance: " << dist << "\nangle: " << tangle << "\nx: " << tx << "\ny: " << ty << "\n";
        if(dist == 0){
            run = false;
		}
        else{
        tvisible = true;
           	for(int i=0; i<tanks.size()&&tvisible; i++){
                if(i!=tankn && i!=tank1){
                    Shape temp = tanks[i]->getShape();
                    if(passedovershape(x, y, tx, ty, temp)){
                        tvisible = false;
                    }
                }
            }
            for(int i=0; i<shapes.size()&&tvisible; i++){
                Shape* temp = shapes[i];
                if(passedovershape(x, y, tx, ty, *temp)){
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
	std::cout << "CHECKS: " << checks << "\n";
	if(vispos || visneg){   //if target is visible return true
        return true;
    }
    return false;
}

bool process(std::vector<double>* xPath, std::vector<double>* yPath, std::vector<uint16_t>* roomPath){
	if(!processingAvailable){
		return false;
	}
	processingAvailable=false;
	//process - DO THIS
	int size1 = xPath->size();
	int size2 = 0;
	while(size1!=size2){
		size1 = xPath->size();
		for(int s=2;s<xPath->size()-1;s++){
			for(int i=0;i<xPath->size()-s;i++){
				if(roomPath->at(i) == roomPath->at(i+s) && noobstacle(xPath->at(i),yPath->at(i),xPath->at(i+s),yPath->at(i+s), roomPath->at(i))){
					for(int j=i+1;j<i+s-1;j++){
						xPath->erase(xPath->begin()+j);
						yPath->erase(yPath->begin()+j);
						roomPath->erase(roomPath->begin()+j);
					}
				}
			}
		}
		size2 = xPath->size();
	}
	return true;
}

bool noobstacle(double x1, double y1, double x2, double y2, uint16_t room){//could improve to account for width of tank
   	bool visible = true;   //keeps track of whether the target is visible
   	for(int i=0; i<tanks.size()&&visible==true; i++){  //check through all shapes that are not the tank or target; if none are blocking the view tvisible is left true
        if(tanks[i]->getCurrentRoom()==room){
            Shape temp = tanks[i]->getShape();
            if(passedovershape(x1, y1, x2, y2, temp)){// || isInShape(x, y, temp)){//wait why check if in shape???
                visible = false;
            }
        }
	}
    for(int i=0; i<shapes.size()&&visible==true; i++){
		if(shapes[i]->room==room){
			Shape* temp = shapes[i];
			if(passedovershape(x1, y1, x2, y2, *temp)){// || isInShape(x, y, *temp)){
				visible = false;
			}
		}
	}
	if(visible == true){   //if target is visible return true
        return true;
	}
	return false;
}

double distanceto(double x, double y, double angle, int tank, double mini, double maxi){//return distance until edge of target tank
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
    if(x1!=x2){
		double ma = (yb-ya)/(xb-xa);
		double m1 = (y2-y1)/(x2-x1);
		double ca = ya - ma * xa;
		double c1 = y1 - m1 * x1;

		double x = (ca-c1)/(m1-ma);
		double y = m1 * x + c1;
		return ((x<xa != x<xb) && (x<x1 != x<x2));
    }else if(y1!=y2){
		double ma = (xb-xa)/(yb-ya);
		double m1 = (x2-x1)/(y2-y1);
		double ca = xa - ma * ya;
		double c1 = x1 - m1 * y1;

		double y = (ca-c1)/(m1-ma);
		double x = m1 * y + c1;
		return ((y<ya != y<yb) && (y<y1 != y<y2));
    }else{//xa==xb and x1==x2
		return false;
	}

}

void sortpoints(std::vector<double>* xs, std::vector<double>* ys){
//    std::cout << "sortpoints\n";
//	auto t1 = Clock::now();
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
        if(smallest==-1){
            std::cout << "SMALLEST IS -1, x0: " << x0 << " i: " << i << "\n";
            for(int k=0;k<xs->size();k++){
				std::cout << "x" << k << " = " << xs->at(i) << " y" << k << " = " << ys->at(i);
				if(mark[k])
					std::cout << " marked\n";
				else
					std::cout << " unmarked\n";
            }
            SDL_Delay(10000);
        }else{
			x->push_back(xs->at(smallest));
            y->push_back(ys->at(smallest));
            mark[smallest] = false;
        }

    }
    *xs = *x;
    *ys = *y;
    delete x;
    delete y;
//    if(timing){
//		auto t2 = Clock::now();
//		std::cout << "Sortpoints execution time: "
//				<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()
//				<< " nanoseconds" << std::endl;
//	}
}

Shape portalToShape(Portal p){
	Shape temp;
	std::vector<double> xvec;
	std::vector<double> yvec;

	xvec.push_back(p.x+p.w);
	xvec.push_back(p.x+p.w);
	xvec.push_back(p.x);
	xvec.push_back(p.x);

	yvec.push_back(p.y);
	yvec.push_back(p.y+p.h);
	yvec.push_back(p.y+p.h);
	yvec.push_back(p.y);

//	sortpoints(&xvec, &yvec);
	temp.x=xvec;
	temp.y=yvec;
	return temp;
}

Portal portalEnter(int index){
	for(int i=0;i<portals.size();i++){
		if(portals[i].room==tanks[index]->getCurrentRoom() && checkcollision(portalToShape(portals[i]), tanks[index]->getShape()))
			return portals[i];
	}
	Portal temp;
	temp.id=0;
	return temp;
}

char sideof(Portal p){
	if(p.inwall){
		if(p.y+p.h/2<0)
			return 'n';
		if(p.x+p.w/2>getRoomDimension(p.room).mapwidth)
			return 'e';
		if(p.y+p.h/2>getRoomDimension(p.room).mapheight)
			return 's';
		if(p.x+p.w/2<0)
			return 'w';
	}
	return 'f';//not in wall
}

RoomDimension getRoomDimension(int room){
	for(int i=0;i<roomDimensions.size();i++){
		if(roomDimensions[i].room==room)
			return roomDimensions[i];
	}
}

bool collision(int index){
    for(int i=0;i<tanks.size();i++){
        if(i==index || tanks[i]->getCurrentRoom()!=tanks[index]->getCurrentRoom())
            ;
        else{
            if(checkcollision(tanks[index]->getShape(), tanks[i]->getShape(), tanks[index]->getX()-tanks[index]->getOldX(), tanks[index]->getY()-tanks[index]->getOldY())){//crash is triggered from here
                if(index==0 && tanks[i]->isDead() && !tanks[i]->isStripped()){//strip enemy tanks
					character.materials+=tanks[i]->getmaterials();
					tanks[i]->strip();
					if(character.repairs<maxrepairs)
						character.repairs++;
                }
                return true;
			}
        }
    }
    for(int i=0;i<shapes.size();i++){
	if(shapes[i]->room==tanks[index]->getCurrentRoom()){
		Shape temp = *shapes[i];
		//std::cout << "SHAPE " << i << "\n";
        if(checkcollision(tanks[index]->getShape(), temp, tanks[index]->getX()-tanks[index]->getOldX(), tanks[index]->getY()-tanks[index]->getOldY())){//, gradient, setgradient)){
            if(index==0)
				shapecollide=i;
            return true;
        }
        //if(i==8)
		//	SDL_Delay(10000);
    }}
    return false;
}

bool checkcollision(Shape shape1, Shape shape2){
	Shape tmp;
	return checkcollision(shape1, shape2, 0, 0);
}

bool checkcollision(Shape shape1, Shape shape2, double xdelta, double ydelta){//, double* gradient, bool* setgradient){//shape1 is the moving tank, shape 2 is the shape
//5.3 microseconds avg-0.02
	for(int i=0;i<shape1.x.size();i++){
		if(isInShape(shape1.x[i], shape1.y[i], shape2)){
			if(xdelta!=0 || ydelta!=0)
				globaltemp=findCollisionAngle(shape1.x[i]-xdelta, shape1.y[i]-ydelta, shape1.x[i], shape1.y[i], shape2);
			//int point1 = findSide(shape1.x[i], shape1.y[i], shape2);
			return true;
        }
    }
    for(int i=0;i<shape2.x.size();i++){
        if(isInShape(shape2.x[i], shape2.y[i], shape1)){
			if(xdelta!=0 || ydelta!=0)
				globaltemp=findCollisionAngle(shape2.x[i]+xdelta, shape2.y[i]+ydelta, shape2.x[i], shape2.y[i], shape1);//could cause problems on corners
            return true;
        }
    }
    return false;
}

bool isInShape(double x, double y, Shape shape){
//0.07 micro with rectangles, 0.15 without-0.02
auto t1 = Clock::now();
	/*if(shape.isCircle){
		if(sqrt(pow((shape.x0-x), 2)+pow((shape.y0-y), 2))<shape.radius){
			return true;
		}else{
			return false;
		}
	}*/
	bool preloaded;
	if(shape.nextangle.size()==shape.x.size())
		preloaded=true;
	else
		preloaded=false;
	if(shape.isRectangle){
		if(shape.x[0]>x && shape.x[2]<x && shape.y[0]<y && shape.y[1]>y){
			return true;
		}else{
			return false;
		}
    }
    int im;
    int ip;
    double Im;
	double Ip;
	if(preloaded){
		Im = shape.prevangle[0];
		Ip = shape.nextangle[0];
	}else{
		im = shape.x.size()-1;
		ip = 1;
		Im = atan2(shape.x[0], shape.y[0], shape.x[im], shape.y[im]);
		Ip = atan2(shape.x[0], shape.y[0], shape.x[ip], shape.y[ip]);
		if(Ip - Im > PI)
			Ip -= 2.0 * PI;
	}
    double IT = atan2(shape.x[0], shape.y[0], x, y);
    if((IT < Im == IT < Ip) && (IT-2.0*PI < Im == IT-2.0*PI < Ip)){ //Sketchy maths that actually works
if(timing){
auto t2 = Clock::now();
otherruntimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count());
if(otherruntimes.size()>10)
otherruntimes.erase(otherruntimes.begin()+0);
}
        return false;
    }
    for(int i=1; i<shape.x.size(); i++){
		if(preloaded){
			Im = shape.prevangle[i];
			Ip = shape.nextangle[i];
		}else{
			im = i-1;
			ip = i+1;
			if(ip==shape.x.size())
				ip=0;
			Im = atan2(shape.x[i], shape.y[i], shape.x[im], shape.y[im]);
			Ip = atan2(shape.x[i], shape.y[i], shape.x[ip], shape.y[ip]);
		}
        IT = atan2(shape.x[i], shape.y[i], x, y);
        if(Ip - Im > PI)
            Ip -= 2.0 * PI;
        if((IT < Im == IT < Ip) && (IT-2.0*PI < Im == IT-2.0*PI < Ip)){ //Sketchy maths that actually works
if(timing){
auto t2 = Clock::now();
otherruntimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count());
if(otherruntimes.size()>10)
otherruntimes.erase(otherruntimes.begin()+0);
}
            return false;
        }
    }
if(timing){
auto t2 = Clock::now();
otherruntimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count());
if(otherruntimes.size()>10)
otherruntimes.erase(otherruntimes.begin()+0);
}
    return true;
}

void preLoadAngles(Shape* shape){
	shape->prevangle.clear();
	shape->nextangle.clear();
	shape->prevangle.push_back(atan2(shape->x[0], shape->y[0], shape->x[shape->x.size()-1], shape->y[shape->x.size()-1]));
	for(int i=0;i<shape->x.size()-1;i++){
		shape->nextangle.push_back(atan2(shape->x[i], shape->y[i], shape->x[i+1], shape->y[i+1]));
		shape->prevangle.push_back(atan2(shape->x[i+1], shape->y[i+1], shape->x[i], shape->y[i]));
	}
	shape->nextangle.push_back(atan2(shape->x[shape->x.size()-1], shape->y[shape->x.size()-1], shape->x[0], shape->y[0]));
	for(int i=0;i<shape->x.size();i++){
		if(shape->nextangle[i] - shape->prevangle[i] > PI)
            shape->nextangle[i] -= 2.0 * PI;
	}
}

double findCollisionAngle(double point00, double point01, double point10, double point11, Shape shape){
//variable - up to a few microseconds
	if(passedover(point00, point01, point10, point11, shape.x[shape.x.size()-1], shape.y[shape.x.size()-1], shape.x[0], shape.y[0])){
		return atan2(shape.x[shape.x.size()-1], shape.y[shape.x.size()-1], shape.x[0], shape.y[0]);
	}
	for(int i=0;i<shape.x.size()-1;i++){
		if(passedover(point00, point01, point10, point11, shape.x[i], shape.y[i], shape.x[i+1], shape.y[i+1])){
			return atan2(shape.x[i], shape.y[i], shape.x[i+1], shape.y[i+1]);
		}
	}
	std::cout << "OH FUCK BAD COLLISION\n";
	std::cout << point00 << " " << point01 << " " << point10 << " " << point11 << "\n";
	//if it reaches here then there isn't actually a collision
	return -1;
}

double atan2(double x1, double y1, double x2, double y2){
//0.06-0.02 microseconds
    if(x2 > x1)
        if(y2 > y1)
            return (PI - atan((x1-x2)/(y1-y2)));
        else
            return (0 - atan((x1-x2)/(y1-y2)));
    else
        if(y2 > y1)
            return (PI - atan((x1-x2)/(y1-y2)));
        else
            return (2.0 * PI - atan((x1-x2)/(y1-y2)));
}

int findSide(double x, double y, Shape shape){//superseded by findCollisionAngle
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

bool updateMouse(bool capzoom){
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
        } else if( e.type == SDL_MOUSEWHEEL ){
			if(e.wheel.y>0){
				for(int i=0;i<e.wheel.y;i++){
					xzoom*=1.1;
					yzoom*=1.1;
				}
			}else if(e.wheel.y<0){
				for(int i=0;i>e.wheel.y;i--){
					xzoom/=1.1;
					yzoom/=1.1;
				}
			}
        }
    }
    if(capzoom){
		double xmin=SCREEN_WIDTH/(getRoomDimension(tanks[0]->getCurrentRoom()).mapwidth+200.0);
		double ymin=SCREEN_HEIGHT/(getRoomDimension(tanks[0]->getCurrentRoom()).mapheight+200.0);
		if(xzoom<xmin && yzoom<ymin){
			if(xmin<ymin){
				xzoom=xmin;
				yzoom=xmin;
			}else{
				xzoom=ymin;
				yzoom=ymin;
			}
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
    /*if( keys[ SDL_SCANCODE_E ] && keys[ SDL_SCANCODE_Q]){
        player->accelerate(10000);
    } else if( keys[ SDL_SCANCODE_Z ] && keys[ SDL_SCANCODE_C]){
        player->accelerate(-10000);
    }*/
    //slomo
    if( keys[ slomoKey] && slomo>0){
        slomo-=4.25*30*stepSize*slomousage;
        if(slomospeed<maxslomospeed){
            slomospeed*=1+0.1*(90*gamespeed/(FPS*physics))*slomoresponse;
        }else{
            slomospeed=maxslomospeed;
        }
    }else if(slomospeed>1){
        slomospeed/=1+0.1*(90*gamespeed/(FPS*physics))*slomoresponse;
    }else{
        slomospeed=1;
    }
    if(slomo<100){
        slomo+=0.25*30*stepSize*slomoregen;
        //if(player->getVel()<5)
        //    slomo+=0.25*30*stepSize;
    }
    //menus
    if( keys[quickMenuKey]){
        freeze = true;
    }else{
        freeze = false;
    }
    if( keys[pauseKey])//pause menu overrides game menu
        pause = true;
	else if( keys[menuKey])
		menu = true;
	//zoom
	if( keys[zoomKey]){
		xzoom=1.0;
		yzoom=1.0;
	}
    //set key and toggle states:
    if(keys[primaryWeaponAssistKey]==true && primaryWeaponAssistKeyState==false)
        primaryWeaponAssistToggle=!primaryWeaponAssistToggle;
    primaryWeaponAssistKeyState=keys[primaryWeaponAssistKey];

    portalKeyPress = false;
    if(keys[portalKey]){
		if(!portalKeyState){
			portalKeyPress = true;
		}
		portalKeyState = true;
    }else{
		portalKeyState = false;
    }
}

double xToDispX(double x){
	return (x-camX)*xzoom+SCREEN_WIDTH/2;
}

double yToDispY(double y){
	return (y-camY)*yzoom+SCREEN_HEIGHT/2;
}

int SCROLL_RenderDrawLine(SDL_Renderer* renderer, int x1, int y1, int x2, int y2){
    return SDL_RenderDrawLine(renderer, (x1-camX)*xzoom+SCREEN_WIDTH/2, (y1-camY)*yzoom+SCREEN_HEIGHT/2, (x2-camX)*xzoom+SCREEN_WIDTH/2, (y2-camY)*yzoom+SCREEN_HEIGHT/2);
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

void verybasicText(std::string text, int x, int y, double scale){
	SDL_Color textColour = {0, 0, 0};
    SDL_Surface* textSurface = TTF_RenderText_Solid( gFont, text.c_str(), textColour );
    SDL_Texture* mTexture = SDL_CreateTextureFromSurface( gRenderer, textSurface );
    SDL_Rect temp = {x-textSurface->w*scale/2.0, y-textSurface->h*scale/2.0, textSurface->w*scale, textSurface->h*scale};
    SDL_FreeSurface( textSurface );
	SDL_RenderCopyEx( gRenderer, mTexture, NULL, &temp, 0, NULL, SDL_FLIP_NONE);
}
//}
