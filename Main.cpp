#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <iostream>

//CONSTANTS
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 960;
const int PI = 3.14159265358979323846;

//VARIABLES
double GRAVITY = 9.81;

//STRUCTS
struct PercentRect{
	double x;
	double y;
	double w;
	double h;
};

//ENUMERATIONS
enum command { ACCELERATE, BRAKE, TURN, TURNTURRET, BRAKETURRET };

//CLASSES
class Tank{
	public:
		Tank(double, double, double);
		~Tank();
		void step();
		void render();
		void free();
		void loadLook(std::string, std::string, std::string, PercentRect*, PercentRect*, PercentRect*, SDL_Rect*, SDL_Rect*, SDL_Rect*);
		void setDimensions(double, double, double, double, double, double);
		void setMass(double, double, double);
		void setOther(double, double, double, double, double, double);
				//commands
		void accelerate(double);	//accelerate with power
		void brake(double);		//brake with force
		void turn(double);		//set power distribution between tracks (to steer)
		void turnturret(double);	//rotate turret with power
		void braketurret(double);	//brake turret with torque
				//accessors
		double getX();			//get x position
		double getY();			//get y position
		double getRot();		//get rotation of tank
		double getTurretrot();		//get rotation of turret
		double getTurretvel();		//get rotational velocity of turret
	private:
				//graphics
		SDL_Texture* body;		//image of tank body
		SDL_Rect bodyclip;		//clip of body image file
		SDL_Texture* turret;		//image of turret
		SDL_Rect turretclip;		//clip of turret image file
		SDL_Texture* barrel;		//image of barrel
		SDL_Rect barrelclip;		//clip of barrel
				//position
		double x;			//x position
		double y;			//y position
		double xvel;			//x velocity
		double yvel;			//y velocity
		double tankrot;			//rotation of tank
		double tankrotvel;		//rotational velocity of tank
		double turretrot;		//rotation of turret relative to tank
		double turretvel;		//rotational velocity of turret relative to tank
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
					//other
		double power;			//power of engine (continous gearing is used)
		double brakeforce;		//force of brakes
		double turretpower;		//power of turret motor
		double turretbraketorque;	//braking torque of turret
		double strackgrip;		//static friction of tracks
		double dtrackgrip;		//dynamic friction of tracks;

		double commands [5];		//array containing values of commands passed to tank
};

//METHODS
void init();
void loadMedia();
void close();
void handleInput(Tank*);	//Reads keyboard input, and sends controls to the Tank passed in

//VARIABLES
SDL_Texture* loadTexture( std::string path );
SDL_Window* gWindow = NULL;
SDL_Renderer* gRenderer = NULL;

void Tank::step(){
	double tankrotrad = (tankrot+90) * PI / 180;    //convert from visual rotation to rotation for calculation
	double velx = xvel / cos(tankrotrad);	//calculations so we can account for drifting
	double vely = yvel / sin(tankrotrad);
	double vel;
	velx > vely ? vel = vely : vel = velx;
        //if velx and vely are 2 different calculations for finding the foward velocity of the tracks assuming it is not drifting
        //if it is drifting, then the smaller one can be assumed to be the foward velocity

	double dx;  //change in x
	double dy;  //change in y
	double step = 0.01;
//	std::cout << vel << "\n";
	double usedpower = commands[ACCELERATE] * power / 100;  //calculate power used		            		//w - 1d
	double drag = vel * vel / 10;   //drag so that tank can coast to a stop
	double braking = commands[BRAKE] * brakeforce / 100 + 10;   //strength of brakes being applied			//n - scalar
	double totalmass = mass + turretmass + barrelmass;  //total mass of tank            			    	//kg - scalar
	double thrust = usedpower / ( abs(vel) + 1 );	//calculate thrust at current velocity  				//n - 1d, +1 is to prevent infinite power at zero velocity, which can not be used with a limited number of calculations per second

											//calculate acceleration provided to tracks by engine and brakes
	if( abs( thrust ) > braking ){							//tracks are powered by the engine
		vel > 0 ? thrust -= braking : thrust += braking;			//calculate net thrust
		dx = cos(tankrotrad) * step * thrust / totalmass;				//calculate accelerations in x and y;
		dy = sin(tankrotrad) * step * thrust / totalmass;
//		std::cout << cos(tankrotrad) << " " << sin(tankrotrad) << " meme\n";
	}else{										//tracks are braking
		thrust = braking - abs(thrust);						//calculate net braking
		dx = cos(tankrotrad) * step * thrust / totalmass;				//calculate accelerations in x and y;
		dy = sin(tankrotrad) * step * thrust / totalmass;
		xvel > 0 ? dx *= -1 : dx = dx;						//make accelerations negative relative to velocity
		yvel > 0 ? dy *= -1 : dy = dy;
	}

	if( abs( velx - vely ) > 1){							//tank is drifting sideways significantly
		double trackvel = 0;
		double force = 0;
		double frictionvector = atan( yvel / xvel );
		double totalforce = GRAVITY * dtrackgrip;
		while(trackvel*force < usedpower){
			trackvel += 0.1;
			double relx = cos( tankrotrad ) * trackvel - xvel;  //relative x velocity between tracks and ground
			double rely = sin( tankrotrad ) * trackvel - yvel;  //relative y velocity between tracks and ground
			frictionvector = atan( rely / relx );   //angle of friction drag between tracks and ground
			double xforce = cos( frictionvector ) * totalforce; //acceleration in x direction
			double yforce = sin( frictionvector ) * totalforce; //acceleration in y direction
			double xtemp = xforce/cos(tankrotrad);  //acceleration of tracks relative to ground
			double ytemp = yforce/sin(tankrotrad);  //acceleration of tracks relative to ground
			xtemp > ytemp ? force = ytemp : force = xtemp;
		}
		xvel > 0 ? xvel -= cos(frictionvector)*totalforce : xvel += cos(frictionvector)*totalforce;
		yvel > 0 ? yvel += sin(frictionvector)*totalforce : yvel -= sin(frictionvector)*totalforce;
		std::cout << cos(frictionvector) << " " << sin(frictionvector) << "\n";
		std::cout << "xvel: " << xvel << "\n" << "yvel: " << yvel << "\n";
		std::cout << "drifting\n";
	}else{
		xvel = cos(tankrotrad) * vel;
		yvel = sin(tankrotrad) * vel;
		if( sqrt(dx*dx + dy*dy) > step * GRAVITY * strackgrip ){			//burnout!!!
			double mult = GRAVITY * dtrackgrip / sqrt(dx*dx + dy*dy);
			xvel += mult * dx;
			yvel += mult * dy;
			std::cout << "burnout\n";
		}else{									//no burnout
			xvel += dx;
			yvel += dy;
			std::cout << sqrt(dx*dx + dy*dy)/step << " " << GRAVITY*strackgrip << " " << "normal\n";
		}
	}
	double temp = atan((yvel+0.0001)/(xvel+0.0001));    //direction of travel
	double temp2 = step*step*drag;  //drag force
	xvel > 0 ? xvel -= cos(temp)*temp2 : xvel += cos(temp)*temp2;
	yvel > 0 ? yvel -= sin(temp)*temp2 : yvel += sin(temp)*temp2;
	std::cout << temp << " " << temp2 << "\n";
//	if(vel>0)
//		vel-=stepsize*usedbraking/totalmass;
//	else if(vel<0)
//		vel+=stepsize*usedbraking/totalmass;
	x+=step*xvel;
	y+=step*yvel;
//	std::cout << dx << " " << dy << "\n";
//	std::cout << xvel << "\n";
}

Tank::Tank(double xpos, double ypos, double rot){
	x = xpos;
	y = ypos;
	tankrot = rot;
	turretrot = 0;
	turretvel = 0;
	xvel = 0;
	yvel = 0;
	tankrotvel = 0;
}

Tank::~Tank(){
	free();
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

void Tank::setMass(double nmass, double nturretmass, double nbarrelmass){
	if( nmass != 0 )
		mass = nmass;
	if( nturretmass != 0 )
		turretmass = nturretmass;
	if( nbarrelmass != 0 )
		barrelmass = nbarrelmass;
}
void Tank::setDimensions(double nlength, double nwidth, double ntrackwidth, double nturretradius, double nbarrellength, double nbarrelwidth){
	int mul = 1;
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
}

void Tank::free(){
	if(body!=NULL)
		SDL_DestroyTexture(body);
	if(turret!=NULL)
		SDL_DestroyTexture(turret);
	if(barrel!=NULL)
		SDL_DestroyTexture(barrel);
}

void Tank::loadLook(std::string bodypath = "body.png", std::string turretpath = "turret.png", std::string barrelpath = "barrel.png", PercentRect* bodysource = NULL, PercentRect* turretsource = NULL, PercentRect* barrelsource = NULL, SDL_Rect* bodystretchn = NULL, SDL_Rect* turretstretchn = NULL, SDL_Rect* barrelstretchn = NULL){
	free();

	SDL_Surface* bodyimage = IMG_Load( bodypath.c_str() );						//Load images
	SDL_Surface* turretimage = IMG_Load( turretpath.c_str() );
	SDL_Surface* barrelimage = IMG_Load( barrelpath.c_str() );

	SDL_SetColorKey( bodyimage, SDL_TRUE, SDL_MapRGB( bodyimage->format, 0xFF, 0xFF, 0xFF ) );	//Colour key loaded images
	SDL_SetColorKey( turretimage, SDL_TRUE, SDL_MapRGB( turretimage->format, 0xFF, 0xFF, 0xFF ) );
	SDL_SetColorKey( barrelimage, SDL_TRUE, SDL_MapRGB( barrelimage->format, 0xFF, 0xFF, 0xFF ) );

	body = SDL_CreateTextureFromSurface( gRenderer, bodyimage );					//Create texture
	turret = SDL_CreateTextureFromSurface( gRenderer, turretimage );
	barrel = SDL_CreateTextureFromSurface( gRenderer, barrelimage );

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

	SDL_FreeSurface( bodyimage );									//Free surfaces
	SDL_FreeSurface( turretimage );
	SDL_FreeSurface( barrelimage );
}

void Tank::render(){											//Render tank at current position with set dimentions
	SDL_Rect temp = {x-width/2, y-length/2, width, length};						//x-width/2 is used, so that the x and y coordinates refer to the center of the tank
	SDL_RenderCopyEx( gRenderer, body, &bodyclip, &temp, tankrot, NULL, SDL_FLIP_NONE);

	temp = {x-turretradius, y-turretradius, turretradius*2, turretradius*2};
	SDL_RenderCopyEx( gRenderer, turret, &turretclip, &temp, tankrot+turretrot, NULL, SDL_FLIP_NONE);

	SDL_Point center = {barrelwidth/2, 1-turretradius};
	temp = {x-barrelwidth/2, y+turretradius, barrelwidth, barrellength};
	SDL_RenderCopyEx( gRenderer, barrel, &barrelclip, &temp, tankrot+turretrot, &center, SDL_FLIP_NONE);
}

double Tank::getX(){
	return x;
}

double Tank::getY(){
	return y;
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

void init(){
	SDL_Init( SDL_INIT_VIDEO );
	SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" );
	gWindow = SDL_CreateWindow( "Game", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
	gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED );
	SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
	IMG_Init( IMG_INIT_PNG );
}

void loadMedia(){

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
	loadMedia();
	Tank* player = new Tank(250, 100, 1-45);
	player->setDimensions(20, 10, 2, 4, 10, 3);
	player->setMass(100, 50, 10);
	player->setOther(250, 1245.69, 10, 10, 0.8, 0.5);
	player->loadLook();
	bool quit = false;
	SDL_Event e;
	while( !quit ){
		while( SDL_PollEvent( &e ) != 0 ){
			if( e.type == SDL_QUIT ){
				quit = true;
			}
		}
		handleInput(player);
		player->step();
		SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
		SDL_RenderClear( gRenderer );
		player->render();
		SDL_RenderPresent( gRenderer );
	}
	close();
	return 0;
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
}
